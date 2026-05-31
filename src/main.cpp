// ─────────────────────────────────────────────────────────────────────────────
//  bridge_main.cpp  –  ESP32 WROOM USB-Serial ↔ ESP-NOW bridge
//
//  Receives JSON commands from Android app via USB Serial,
//  forwards them to Heltec heads via ESP-NOW.
//  Receives status from heads via ESP-NOW,
//  forwards them back to Android app via USB Serial.
//
//  Serial protocol (newline-delimited JSON):
//    App → Bridge: {"cmd":"set_state","state":1,"filename":"plan1.json"}
//    Bridge → App: {"type":"status","addr":"AA:BB:CC:DD:EE:FF","state":1}
//    Bridge → App: {"type":"log","msg":"Head connected"}
//    Bridge → App: {"type":"error","msg":"Send failed"}
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Configuration
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t  ESPNOW_CHANNEL   = 1;
static constexpr uint32_t SERIAL_BAUD      = 115200;
static constexpr size_t   SERIAL_BUF_SIZE  = 512;

// Broadcast MAC address — sends to all ESP-NOW peers simultaneously
static const uint8_t BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ─────────────────────────────────────────────────────────────────────────────
//  State enum — must match the head firmware
// ─────────────────────────────────────────────────────────────────────────────

enum class State : uint8_t
{
    Preparation = 0,
    Armement    = 1,
    Play        = 2,
    Pause       = 3,
    Stop        = 4,
    Fin         = 5,
};

// ─────────────────────────────────────────────────────────────────────────────
//  Known heads registry
//  Heads announce themselves via ESP-NOW on boot — we add them here
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint8_t MAX_HEADS = 8;

struct HeadEntry
{
    uint8_t mac[6]   = {};
    bool    active   = false;
    uint8_t state    = 0;
    char    name[32] = {};
};

static HeadEntry s_heads[MAX_HEADS];
static uint8_t   s_headCount = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Serial input buffer
// ─────────────────────────────────────────────────────────────────────────────

static char    s_serialBuf[SERIAL_BUF_SIZE];
static size_t  s_serialLen = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Send a JSON object to the Android app via Serial (newline terminated)
static void sendToApp(JsonDocument& doc)
{
    String json;
    serializeJson(doc, json);
    Serial.println(json);
}

/// Send a log message to the app
static void logToApp(const char* msg)
{
    JsonDocument doc;
    doc["type"] = "log";
    doc["msg"]  = msg;
    sendToApp(doc);
}

/// Send an error message to the app
static void errorToApp(const char* msg)
{
    JsonDocument doc;
    doc["type"] = "error";
    doc["msg"]  = msg;
    sendToApp(doc);
}

/// Convert MAC bytes to string
static void macToStr(const uint8_t* mac, char* out, size_t outSize)
{
    snprintf(out, outSize, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/// Find or register a head by MAC address
static HeadEntry* findOrAddHead(const uint8_t* mac)
{
    // Search existing
    for (uint8_t i = 0; i < s_headCount; ++i)
    {
        if (memcmp(s_heads[i].mac, mac, 6) == 0)
            return &s_heads[i];
    }

    // Add new
    if (s_headCount >= MAX_HEADS)
        return nullptr;

    memcpy(s_heads[s_headCount].mac, mac, 6);
    s_heads[s_headCount].active = true;
    return &s_heads[s_headCount++];
}

/// Register a MAC as ESP-NOW peer if not already registered
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

// ─────────────────────────────────────────────────────────────────────────────
//  ESP-NOW callbacks
// ─────────────────────────────────────────────────────────────────────────────

static void onEspNowSent(const uint8_t* mac, esp_now_send_status_t status)
{
    char macStr[18];
    macToStr(mac, macStr, sizeof(macStr));

    JsonDocument doc;
    doc["type"]    = "send_status";
    doc["mac"]     = macStr;
    doc["success"] = (status == ESP_NOW_SEND_SUCCESS);
    sendToApp(doc);
}

static void onEspNowReceived(const uint8_t* mac,
                              const uint8_t* data,
                              int            len)
{
    // Register peer if new
    ensurePeer(mac);
    HeadEntry* head = findOrAddHead(mac);

    // Parse the received JSON
    JsonDocument doc;
    if (deserializeJson(doc, data, len) != DeserializationError::Ok)
    {
        errorToApp("Received invalid JSON from head");
        return;
    }

    const char* type = doc["type"] | "";
    char macStr[18];
    macToStr(mac, macStr, sizeof(macStr));

    if (strcmp(type, "announce") == 0)
    {
        // Head announcing itself
        if (head)
        {
            strlcpy(head->name, doc["name"] | macStr, sizeof(head->name));
            head->active = true;
        }

        JsonDocument out;
        out["type"] = "head_connected";
        out["mac"]  = macStr;
        out["name"] = doc["name"] | macStr;
        sendToApp(out);

        logToApp("Head connected");
    }
    else if (strcmp(type, "status") == 0)
    {
        // Status update from head
        if (head)
            head->state = doc["state"] | 0;

        JsonDocument out;
        out["type"]  = "status";
        out["mac"]   = macStr;
        out["state"] = doc["state"] | 0;
        if (doc.containsKey("lat"))
        {
            out["lat"] = doc["lat"];
            out["lng"] = doc["lng"];
        }
        sendToApp(out);
    }
    else
    {
        // Forward any other message directly to the app
        doc["mac"] = macStr;
        sendToApp(doc);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Command handlers (called when app sends a command)
// ─────────────────────────────────────────────────────────────────────────────

static void handleSetState(JsonDocument& doc)
{
    const uint8_t stateVal = doc["state"] | 255;
    if (stateVal > 5)
    {
        errorToApp("Invalid state value");
        return;
    }

    // Build ESP-NOW message
    JsonDocument msg;
    msg["cmd"]   = "set_state";
    msg["state"] = stateVal;

    if (stateVal == static_cast<uint8_t>(State::Play))
    {
        const char* filename = doc["filename"] | "";
        if (strlen(filename) == 0)
        {
            errorToApp("Play state requires a filename");
            return;
        }
        msg["filename"] = filename;
    }

    String json;
    serializeJson(msg, json);

    // Ensure broadcast peer is registered
    ensurePeer(BROADCAST_MAC);

    // Broadcast to all heads simultaneously
    const esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t*>(json.c_str()),
        json.length()
    );

    if (result != ESP_OK)
    {
        errorToApp("ESP-NOW send failed");
        return;
    }

    Serial.printf("[Bridge] Broadcast state=%u\n", stateVal);
}

static void handleGetHeads(JsonDocument& /*doc*/)
{
    // Return list of known heads to the app
    JsonDocument out;
    out["type"]       = "head_list";
    JsonArray   heads = out["heads"].to<JsonArray>();

    for (uint8_t i = 0; i < s_headCount; ++i)
    {
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

// ─────────────────────────────────────────────────────────────────────────────
//  Serial input processing
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

    if (strcmp(cmd, "set_state") == 0)
        handleSetState(doc);
    else if (strcmp(cmd, "get_heads") == 0)
        handleGetHeads(doc);
    else
        errorToApp("Unknown command");
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
//  setup / loop
// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(500);

    // Initialise WiFi in station mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Initialise ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[Bridge] ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(onEspNowSent);
    esp_now_register_recv_cb(onEspNowReceived);

    // Register broadcast peer
    ensurePeer(BROADCAST_MAC);

    logToApp("Bridge ready");
    Serial.printf("[Bridge] MAC: %s\n", WiFi.macAddress().c_str());
}

void loop()
{
    readSerial();
    delay(1);
}