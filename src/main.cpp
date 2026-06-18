// ─────────────────────────────────────────────────────────────────────────────
//  bridge_main.cpp  –  ESP32 WROOM USB-Serial ↔ ESP-NOW bridge
//
//  Serial protocol (newline-delimited JSON):
//
//  App → Bridge (commands):
//    {"cmd":"set_state","state":2,"filename":"plan1.json"}
//    {"cmd":"get_heads"}
//    {"cmd":"get_files","mac":"AA:BB:CC:DD:EE:FF"}
//    {"cmd":"file_start","name":"plan1.json","size":2100,"chunks":17,"targets":["AA:BB:CC:DD:EE:FF"]}
//    {"cmd":"file_chunk","idx":0,"data":"<base64>","crc":1234}
//    {"cmd":"file_end","name":"plan1.json","total_crc":5678}
//    {"cmd":"file_delete","name":"plan1.json","mac":"AA:BB:CC:DD:EE:FF"}
//
//  Bridge → App (responses):
//    {"type":"log","msg":"..."}
//    {"type":"error","msg":"..."}
//    {"type":"head_connected","mac":"...","name":"..."}
//    {"type":"head_list","heads":[...]}
//    {"type":"status","mac":"...","state":2}
//    {"type":"file_list","mac":"...","free_bytes":1234,"total_bytes":5678,"files":[...]}
//    {"type":"chunk_ack","idx":0,"success":true,"failed":[]}
//    {"type":"transfer_complete","name":"plan1.json","failed":["AA:BB"]}
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Configuration
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t  ESPNOW_CHANNEL      = 1;
static constexpr uint32_t SERIAL_BAUD         = 115200;
static constexpr size_t   SERIAL_BUF_SIZE     = 2048; // larger for base64 chunks
static constexpr uint32_t CHUNK_ACK_TIMEOUT   = 1000; // ms per chunk ACK
static constexpr uint8_t  CHUNK_MAX_RETRIES   = 3;
static constexpr uint8_t  MAX_HEADS           = 8;
static constexpr uint8_t  MAX_TRANSFER_HEADS  = 8;
// Head liveness: STATUS_INTERVAL_MS on the head is 2 s; 10 missed beats = 20 s
static constexpr uint32_t HEAD_TIMEOUT_MS     = 20000;
// State retry: resend state to reconnecting heads in case of packet loss
static constexpr uint8_t  STATE_RETRY_COUNT   = 2;
static constexpr uint32_t STATE_RETRY_MS      = 150;

static const uint8_t k_broadcast[6] = { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF };

// ─────────────────────────────────────────────────────────────────────────────
//  Head registry
// ─────────────────────────────────────────────────────────────────────────────

struct HeadEntry
{
    uint8_t  mac[6]      = {};
    bool     active      = false;
    int      state       = -1;
    char     name[32]    = {};
    uint32_t lastSeenMs  = 0;
};

static HeadEntry s_heads[MAX_HEADS];
static uint8_t   s_headCount = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  State retry — resend state to a head that just reconnected
// ─────────────────────────────────────────────────────────────────────────────

struct StateRetry {
    bool     active     = false;
    uint8_t  mac[6]     = {};
    uint32_t nextSendMs = 0;
    uint8_t  remaining  = 0;
};
static StateRetry s_stateRetries[MAX_HEADS];

// ─────────────────────────────────────────────────────────────────────────────
//  File transfer state
// ─────────────────────────────────────────────────────────────────────────────

struct TransferState
{
    bool     active       = false;
    char     name[64]     = {};
    uint16_t totalChunks  = 0;
    uint16_t currentChunk = 0;  // chunk currently waiting ACK for

    // Target heads
    uint8_t  targetMacs[MAX_TRANSFER_HEADS][6] = {};
    uint8_t  targetCount  = 0;

    // Per-head ACK tracking for current chunk
    bool     acked[MAX_TRANSFER_HEADS]  = {};
    bool     failed[MAX_TRANSFER_HEADS] = {}; // permanently failed heads

    // Retry state
    uint8_t  retryCount   = 0;
    uint32_t chunkSentMs  = 0;

    // Last chunk data (for retry)
    char     chunkData[400] = {}; // base64 encoded chunk
    uint16_t chunkCrc       = 0;

    void reset()
    {
        active       = false;
        name[0]      = '\0';
        totalChunks  = 0;
        currentChunk = 0;
        targetCount  = 0;
        retryCount   = 0;
        chunkSentMs  = 0;
        memset(acked,  false, sizeof(acked));
        memset(failed, false, sizeof(failed));
    }

    bool allAcked() const
    {
        for (uint8_t i = 0; i < targetCount; ++i)
            if (!acked[i] && !failed[i])
                return false;
        return true;
    }

    void resetAcks()
    {
        for (uint8_t i = 0; i < targetCount; ++i)
            acked[i] = false;
    }
};

static TransferState s_transfer;

// ─────────────────────────────────────────────────────────────────────────────
//  Last known state — replayed to heads that reconnect
// ─────────────────────────────────────────────────────────────────────────────

static int      s_currentState    = -1;
static char     s_currentFilename[64] = {};
static uint32_t s_playOffsetMs   = 0; // track position (ms) at last play/jump
static uint32_t s_playStartMs    = 0; // millis() at that moment

// ─────────────────────────────────────────────────────────────────────────────
//  Serial input buffer
// ─────────────────────────────────────────────────────────────────────────────

static char   s_serialBuf[SERIAL_BUF_SIZE];
static size_t s_serialLen = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────

static void sendToApp(JsonDocument& doc)
{
    String json;
    serializeJson(doc, json);
    Serial.println(json);
}

static void logToApp(const char* msg)
{
    JsonDocument doc;
    doc["type"] = "log";
    doc["msg"]  = msg;
    sendToApp(doc);
}

static void errorToApp(const char* msg)
{
    JsonDocument doc;
    doc["type"] = "error";
    doc["msg"]  = msg;
    sendToApp(doc);
}

static void macToStr(const uint8_t* mac, char* out, size_t size)
{
    snprintf(out, size, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool strToMac(const char* str, uint8_t* out)
{
    return sscanf(str, "%hhX:%hhX:%hhX:%hhX:%hhX:%hhX",
                  &out[0], &out[1], &out[2],
                  &out[3], &out[4], &out[5]) == 6;
}

static HeadEntry* findOrAddHead(const uint8_t* mac)
{
    for (uint8_t i = 0; i < s_headCount; ++i)
        if (memcmp(s_heads[i].mac, mac, 6) == 0)
            return &s_heads[i];

    if (s_headCount >= MAX_HEADS)
        return nullptr;

    memcpy(s_heads[s_headCount].mac, mac, 6);
    s_heads[s_headCount].active     = false; // activated on first announce
    s_heads[s_headCount].lastSeenMs = millis();
    return &s_heads[s_headCount++];
}

static void ensurePeer(const uint8_t* mac)
{
    if (esp_now_is_peer_exist(mac))
        return;
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

static void sendToHead(const uint8_t* mac, JsonDocument& doc)
{
    String json;
    serializeJson(doc, json);
    ensurePeer(mac);
    esp_now_send(mac,
                 reinterpret_cast<const uint8_t*>(json.c_str()),
                 json.length());
}

// ─────────────────────────────────────────────────────────────────────────────
//  File transfer — send current chunk to all non-failed targets
// ─────────────────────────────────────────────────────────────────────────────

static void sendCurrentChunk()
{
    JsonDocument doc;
    doc["cmd"]  = "file_chunk";
    doc["idx"]  = s_transfer.currentChunk;
    doc["data"] = s_transfer.chunkData;
    doc["crc"]  = s_transfer.chunkCrc;

    for (uint8_t i = 0; i < s_transfer.targetCount; ++i)
    {
        if (s_transfer.failed[i] || s_transfer.acked[i])
            continue;
        sendToHead(s_transfer.targetMacs[i], doc);
    }

    s_transfer.chunkSentMs = millis();
}

static void reportFailedHeads()
{
    JsonDocument doc;
    doc["type"] = "transfer_complete";
    doc["name"] = s_transfer.name;

    JsonArray failed = doc["failed"].to<JsonArray>();
    for (uint8_t i = 0; i < s_transfer.targetCount; ++i)
    {
        if (s_transfer.failed[i])
        {
            char macStr[18];
            macToStr(s_transfer.targetMacs[i], macStr, sizeof(macStr));
            failed.add(macStr);
        }
    }
    sendToApp(doc);
}

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW callbacks
// ─────────────────────────────────────────────────────────────────────────────

static void onEspNowSent(const uint8_t* mac, esp_now_send_status_t status)
{
    // Only log failures — successes are too frequent during file transfer
    if (status != ESP_NOW_SEND_SUCCESS)
    {
        char macStr[18];
        macToStr(mac, macStr, sizeof(macStr));
        Serial.printf("[Bridge] Send failed to %s\n", macStr);
    }
}

static void onEspNowReceived(const uint8_t* mac,
                              const uint8_t* data,
                              int            len)
{
    ensurePeer(mac);
    HeadEntry* head = findOrAddHead(mac);

    char macStr[18];
    macToStr(mac, macStr, sizeof(macStr));

    // Capture before any mutation: true for new heads and those that timed out.
    const bool wasInactive = !head || !head->active;

    if (head)
        head->lastSeenMs = millis();

    JsonDocument doc;
    if (deserializeJson(doc, data, len) != DeserializationError::Ok)
        return;

    const char* type = doc["type"] | "";

    // ── Announce ──────────────────────────────────────────────────────────────
    if (strcmp(type, "announce") == 0)
    {
        if (head)
        {
            strlcpy(head->name, doc["name"] | macStr, sizeof(head->name));
            head->active = true;
        }

        // Only do full initialisation for new / recovered heads.
        // Periodic announces from already-active heads are silently ignored —
        // acting on every announce would replay the state and restart the sequencer.
        if (wasInactive)
        {
            JsonDocument out;
            out["type"] = "head_connected";
            out["mac"]  = macStr;
            out["name"] = doc["name"] | macStr;
            sendToApp(out);

            JsonDocument req;
            req["cmd"] = "get_files";
            sendToHead(mac, req);

            if (s_currentState >= 0)
            {
                JsonDocument stateMsg;
                stateMsg["cmd"]   = "set_state";
                stateMsg["state"] = s_currentState;
                if (s_currentState == 2 && s_currentFilename[0] != '\0')
                    stateMsg["filename"] = s_currentFilename;
                sendToHead(mac, stateMsg);

                if (s_currentState == 2)
                {
                    uint32_t currentMs = s_playOffsetMs + (millis() - s_playStartMs);
                    JsonDocument jumpMsg;
                    jumpMsg["cmd"]          = "jump";
                    jumpMsg["timeInTenths"] = (int)(currentMs / 100);
                    sendToHead(mac, jumpMsg);
                }

                scheduleStateRetry(mac);
            }
        }
    }

    // ── Status ────────────────────────────────────────────────────────────────
    else if (strcmp(type, "status") == 0)
    {
        if (head) head->state = doc["state"] | -1;

        JsonDocument out;
        out["type"]  = "status";
        out["mac"]   = macStr;
        out["state"] = doc["state"] | -1;
        sendToApp(out);
    }

    // ── File list response ────────────────────────────────────────────────────
    else if (strcmp(type, "file_list") == 0)
    {
        // Forward directly to app with mac added
        doc["mac"] = macStr;
        sendToApp(doc);
    }

    // ── Chunk ACK ─────────────────────────────────────────────────────────────
    else if (strcmp(type, "chunk_ack") == 0)
    {
        if (!s_transfer.active)
            return;

        const uint16_t idx = doc["idx"] | 0xFFFF;
        if (idx != s_transfer.currentChunk)
            return;

        // Mark this head as acked
        for (uint8_t i = 0; i < s_transfer.targetCount; ++i)
        {
            if (memcmp(s_transfer.targetMacs[i], mac, 6) == 0)
            {
                s_transfer.acked[i] = true;
                break;
            }
        }
    }

    // ── File transfer ACKs (start/end) ────────────────────────────────────────
    else if (strcmp(type, "file_start_ack") == 0 ||
             strcmp(type, "file_end_ack")   == 0)
    {
        // Handled the same as chunk_ack for simplicity
        for (uint8_t i = 0; i < s_transfer.targetCount; ++i)
        {
            if (memcmp(s_transfer.targetMacs[i], mac, 6) == 0)
            {
                s_transfer.acked[i] = true;
                break;
            }
        }
    }

    // ── Delete ACK ────────────────────────────────────────────────────────────
    else if (strcmp(type, "delete_ack") == 0)
    {
        // Request updated file list after delete
        JsonDocument req;
        req["cmd"] = "get_files";
        sendToHead(mac, req);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Transfer update — called every loop() while transfer is active
// ─────────────────────────────────────────────────────────────────────────────

static void updateTransfer()
{
    if (!s_transfer.active)
        return;

    // Check if all heads acked current chunk
    if (s_transfer.allAcked())
    {
        // Report chunk success to app
        JsonDocument out;
        out["type"]    = "chunk_ack";
        out["idx"]     = s_transfer.currentChunk;
        out["success"] = true;
        sendToApp(out);

        // Move to next chunk — app will send it
        s_transfer.currentChunk++;
        s_transfer.retryCount = 0;
        s_transfer.resetAcks();
        return;
    }

    // Check for timeout
    if (millis() - s_transfer.chunkSentMs < CHUNK_ACK_TIMEOUT)
        return;

    // Timeout — retry or fail
    if (s_transfer.retryCount >= CHUNK_MAX_RETRIES)
    {
        // Mark timed-out heads as failed
        for (uint8_t i = 0; i < s_transfer.targetCount; ++i)
        {
            if (!s_transfer.acked[i] && !s_transfer.failed[i])
            {
                s_transfer.failed[i] = true;
                char macStr[18];
                macToStr(s_transfer.targetMacs[i], macStr, sizeof(macStr));
                Serial.printf("[Bridge] Head %s failed chunk %u\n",
                              macStr, s_transfer.currentChunk);
            }
        }

        // If all remaining heads have now failed or acked, move on
        if (s_transfer.allAcked())
        {
            JsonDocument out;
            out["type"]    = "chunk_ack";
            out["idx"]     = s_transfer.currentChunk;
            out["success"] = false;
            sendToApp(out);

            s_transfer.currentChunk++;
            s_transfer.retryCount = 0;
            s_transfer.resetAcks();
        }
        return;
    }

    // Retry
    s_transfer.retryCount++;
    Serial.printf("[Bridge] Retry chunk %u  attempt %u/%u\n",
                  s_transfer.currentChunk,
                  s_transfer.retryCount,
                  CHUNK_MAX_RETRIES);
    sendCurrentChunk();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Command handlers
// ─────────────────────────────────────────────────────────────────────────────

static void handleSetState(JsonDocument& doc)
{
    const int stateVal = doc["state"] | -1;
    if (stateVal < 0 || stateVal > 5)
    {
        errorToApp("Invalid state value");
        return;
    }

    JsonDocument msg;
    msg["cmd"]   = "set_state";
    msg["state"] = stateVal;

    if (stateVal == 2) // Play
    {
        const char* filename = doc["filename"] | "";
        if (strlen(filename) == 0)
        {
            errorToApp("Play state requires a filename");
            return;
        }
        msg["filename"] = filename;
    }

    // Remember for heads that reconnect later
    s_currentState = stateVal;
    if (stateVal == 2)
    {
        strlcpy(s_currentFilename, doc["filename"] | "", sizeof(s_currentFilename));
        s_playOffsetMs = 0;
        s_playStartMs  = millis();
    }
    else
        s_currentFilename[0] = '\0';

    String json;
    serializeJson(msg, json);
    ensurePeer(k_broadcast);
    esp_now_send(k_broadcast,
                 reinterpret_cast<const uint8_t*>(json.c_str()),
                 json.length());
}

static void handleJump(JsonDocument& doc) {
    if (doc["timeInTenths"].isNull()) {
        errorToApp("jump: missing timeInTenths field");
        return;
    }

    const int timeInTenths = doc["timeInTenths"].as<int>();

    s_playOffsetMs = (uint32_t)timeInTenths * 100;
    s_playStartMs  = millis();

    JsonDocument msg;
    msg["cmd"]          = "jump";
    msg["timeInTenths"] = timeInTenths;

    String json;
    serializeJson(msg, json);
    ensurePeer(k_broadcast);
    esp_now_send(k_broadcast,
            reinterpret_cast<const uint8_t*>(json.c_str()),
            json.length());
}

static void handleGetHeads(JsonDocument&)
{
    JsonDocument out;
    out["type"]     = "head_list";
    JsonArray heads = out["heads"].to<JsonArray>();

    for (uint8_t i = 0; i < s_headCount; ++i)
    {
        if (!s_heads[i].active) continue;
        char macStr[18];
        macToStr(s_heads[i].mac, macStr, sizeof(macStr));
        JsonObject h = heads.add<JsonObject>();
        h["mac"]    = macStr;
        h["name"]   = s_heads[i].name;
        h["active"] = s_heads[i].active;
        h["state"]  = s_heads[i].state;
    }
    sendToApp(out);
}

static void handleGetFiles(JsonDocument& doc)
{
    const char* macStr = doc["mac"] | "";
    uint8_t mac[6];
    if (!strToMac(macStr, mac))
    {
        errorToApp("Invalid MAC address");
        return;
    }
    JsonDocument req;
    req["cmd"] = "get_files";
    sendToHead(mac, req);
}

static void handleFileStart(JsonDocument& doc)
{
    if (s_transfer.active)
    {
        errorToApp("Transfer already in progress");
        return;
    }

    // Build target list
    JsonArray targets = doc["targets"].as<JsonArray>();
    if (targets.isNull() || targets.size() == 0)
    {
        errorToApp("No target heads specified");
        return;
    }

    s_transfer.reset();
    strlcpy(s_transfer.name, doc["name"] | "", sizeof(s_transfer.name));
    s_transfer.totalChunks  = doc["chunks"] | 0;
    s_transfer.currentChunk = 0;

    // Parse target MACs
    for (JsonVariant t : targets)
    {
        if (s_transfer.targetCount >= MAX_TRANSFER_HEADS)
            break;
        if (strToMac(t.as<const char*>(),
                     s_transfer.targetMacs[s_transfer.targetCount]))
            ++s_transfer.targetCount;
    }

    // Send file_start to all targets and wait for ACKs
    JsonDocument msg;
    msg["cmd"]    = "file_start";
    msg["name"]   = s_transfer.name;
    msg["size"]   = doc["size"] | 0;
    msg["chunks"] = s_transfer.totalChunks;

    for (uint8_t i = 0; i < s_transfer.targetCount; ++i)
        sendToHead(s_transfer.targetMacs[i], msg);

    s_transfer.chunkSentMs = millis();
    s_transfer.active      = true;

    JsonDocument ack;
    ack["type"] = "file_start_ack";
    sendToApp(ack);

    Serial.printf("[Bridge] Transfer started  name=%s  chunks=%u  heads=%u\n",
                  s_transfer.name, s_transfer.totalChunks, s_transfer.targetCount);
}

static void handleFileChunk(JsonDocument& doc)
{
    if (!s_transfer.active)
    {
        errorToApp("No active transfer");
        return;
    }

    // Store chunk data for retries
    strlcpy(s_transfer.chunkData,
            doc["data"] | "",
            sizeof(s_transfer.chunkData));
    s_transfer.chunkCrc = doc["crc"] | 0;

    s_transfer.resetAcks();
    s_transfer.retryCount = 0;
    sendCurrentChunk();
}

static void handleFileEnd(JsonDocument& doc)
{
    if (!s_transfer.active)
    {
        errorToApp("No active transfer");
        return;
    }

    // Send file_end to all non-failed targets
    JsonDocument msg;
    msg["cmd"]       = "file_end";
    msg["name"]      = s_transfer.name;
    msg["total_crc"] = doc["total_crc"] | 0;

    for (uint8_t i = 0; i < s_transfer.targetCount; ++i)
    {
        if (!s_transfer.failed[i])
            sendToHead(s_transfer.targetMacs[i], msg);
    }

    // Report completion and failed heads to app
    reportFailedHeads();

    // Refresh file list on all successful heads
    for (uint8_t i = 0; i < s_transfer.targetCount; ++i)
    {
        if (!s_transfer.failed[i])
        {
            JsonDocument req;
            req["cmd"] = "get_files";
            sendToHead(s_transfer.targetMacs[i], req);
        }
    }

    s_transfer.reset();
    Serial.println("[Bridge] Transfer complete");
}

static void handleFileDelete(JsonDocument& doc)
{
    const char* macStr  = doc["mac"]  | "";
    const char* name    = doc["name"] | "";

    uint8_t mac[6];
    if (!strToMac(macStr, mac))
    {
        errorToApp("Invalid MAC address");
        return;
    }

    JsonDocument msg;
    msg["cmd"]  = "file_delete";
    msg["name"] = name;
    sendToHead(mac, msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Serial processing
// ─────────────────────────────────────────────────────────────────────────────

static void processSerialLine(const char* line)
{
    JsonDocument doc;
    if (deserializeJson(doc, line) != DeserializationError::Ok)
    {
        errorToApp("Invalid JSON from app");
        return;
    }

    const char* cmd = doc["cmd"] | "";

    if      (strcmp(cmd, "set_state")   == 0) handleSetState(doc);
    else if (strcmp(cmd, "jump")        == 0) handleJump(doc);
    else if (strcmp(cmd, "get_heads")   == 0) handleGetHeads(doc);
    else if (strcmp(cmd, "get_files")   == 0) handleGetFiles(doc);
    else if (strcmp(cmd, "file_start")  == 0) handleFileStart(doc);
    else if (strcmp(cmd, "file_chunk")  == 0) handleFileChunk(doc);
    else if (strcmp(cmd, "file_end")    == 0) handleFileEnd(doc);
    else if (strcmp(cmd, "file_delete") == 0) handleFileDelete(doc);
    else errorToApp("Unknown command");
}

static void readSerial()
{
    while (Serial.available())
    {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r')
        {
            if (s_serialLen > 0)
            {
                s_serialBuf[s_serialLen] = '\0';
                processSerialLine(s_serialBuf);
                s_serialLen = 0;
            }
        }
        else if (s_serialLen < SERIAL_BUF_SIZE - 1)
        {
            s_serialBuf[s_serialLen++] = c;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  State retry — called every loop()
// ─────────────────────────────────────────────────────────────────────────────

static void scheduleStateRetry(const uint8_t* mac)
{
    for (auto& r : s_stateRetries)
    {
        if (!r.active || memcmp(r.mac, mac, 6) == 0)
        {
            memcpy(r.mac, mac, 6);
            r.active     = true;
            r.nextSendMs = millis() + STATE_RETRY_MS;
            r.remaining  = STATE_RETRY_COUNT;
            return;
        }
    }
}

static void updateStateRetries()
{
    if (s_currentState < 0) return;

    const uint32_t now = millis();
    for (auto& r : s_stateRetries)
    {
        if (!r.active || now < r.nextSendMs) continue;

        JsonDocument stateMsg;
        stateMsg["cmd"]   = "set_state";
        stateMsg["state"] = s_currentState;
        if (s_currentState == 2 && s_currentFilename[0] != '\0')
            stateMsg["filename"] = s_currentFilename;
        sendToHead(r.mac, stateMsg);

        if (s_currentState == 2)
        {
            uint32_t currentMs = s_playOffsetMs + (now - s_playStartMs);
            JsonDocument jumpMsg;
            jumpMsg["cmd"]          = "jump";
            jumpMsg["timeInTenths"] = (int)(currentMs / 100);
            sendToHead(r.mac, jumpMsg);
        }

        if (--r.remaining == 0)
            r.active = false;
        else
            r.nextSendMs = now + STATE_RETRY_MS;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Head heartbeat — called every loop()
// ─────────────────────────────────────────────────────────────────────────────

static void updateHeartbeat()
{
    const uint32_t now = millis();
    for (uint8_t i = 0; i < s_headCount; ++i)
    {
        if (!s_heads[i].active) continue;
        if (now - s_heads[i].lastSeenMs < HEAD_TIMEOUT_MS) continue;

        s_heads[i].active = false;

        char macStr[18];
        macToStr(s_heads[i].mac, macStr, sizeof(macStr));
        Serial.printf("[Bridge] Head %s timed out\n", macStr);

        JsonDocument out;
        out["type"] = "head_inactive";
        out["mac"]  = macStr;
        sendToApp(out);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup / loop
// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[Bridge] ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(onEspNowSent);
    esp_now_register_recv_cb(onEspNowReceived);
    ensurePeer(k_broadcast);

    logToApp("Bridge ready");
}

void loop()
{
    readSerial();
    updateTransfer();
    updateStateRetries();
    updateHeartbeat();
    delay(1);
}