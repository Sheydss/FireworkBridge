# Fireworks Bridge — CLAUDE.md

## Project overview

This is the firmware for an **ESP32 WROOM** acting as a **USB-Serial ↔ ESP-NOW bridge**
between the Android Flutter app and the Heltec head boards.

The phone connects via USB-C OTG cable. The bridge translates JSON commands
from the app into ESP-NOW packets sent to the heads, and forwards ESP-NOW
messages from heads back to the app.

---

## Hardware

| Component | Details |
|---|---|
| Board | Any ESP32 WROOM dev board with USB |
| Connection to phone | USB-C OTG cable (data cable, not charge-only) |
| Connection to heads | ESP-NOW (WiFi 2.4GHz, channel 1) |

### Power note
The board can be powered via the phone's USB OTG port.
If using external 5V power simultaneously, add a physical switch on the
external 5V line to avoid backfeed conflicts during normal operation
(upload still works via USB regardless).

---

## File structure

```
src/
└── bridge_main.cpp   # Single file — entire bridge firmware
```

---

## Serial protocol

**All messages are newline-delimited JSON** (`\n` terminated).
Baud rate: **115200**.

### App → Bridge

| Command | Description |
|---|---|
| `{"cmd":"set_state","state":2,"filename":"plan1.json"}` | Change state on all heads |
| `{"cmd":"get_heads"}` | Request list of known heads |
| `{"cmd":"get_files","mac":"AA:BB:CC:DD:EE:FF"}` | Request file list from one head |
| `{"cmd":"file_start","name":"...","size":1712,"chunks":14,"targets":["AA:BB..."]}` | Start file transfer |
| `{"cmd":"file_chunk","idx":0,"data":"<base64>","crc":12345}` | Send one chunk |
| `{"cmd":"file_end","name":"...","total_crc":19542}` | Finalise transfer |
| `{"cmd":"file_delete","name":"plan1.json","mac":"AA:BB..."}` | Delete file from head |
| `{"cmd":"jump","timeInTenths":150}` | Jump to time position |

### Bridge → App

| Message | Description |
|---|---|
| `{"type":"log","msg":"..."}` | Debug log |
| `{"type":"error","msg":"..."}` | Error message |
| `{"type":"head_connected","mac":"...","name":"..."}` | New head detected |
| `{"type":"head_list","heads":[...]}` | Response to get_heads |
| `{"type":"status","mac":"...","state":2}` | Head status update |
| `{"type":"file_list","mac":"...","free_bytes":1234,"total_bytes":5678,"files":[...]}` | Head file list |
| `{"type":"file_start_ack"}` | All heads ready for chunks |
| `{"type":"chunk_ack","idx":0,"success":true}` | All heads ACKed chunk |
| `{"type":"transfer_complete","name":"...","failed":["AA:BB"]}` | Transfer finished |
| `{"type":"delete_ack"}` | File deleted |

---

## ESP-NOW protocol

### Channel
Fixed channel **1**. Must match all head boards.

### Head discovery
Heads broadcast `{"type":"announce","mac":"..."}` on boot and every 10s.
Bridge learns head MAC from sender field, registers as peer, requests file list.

### File transfer flow
```
App sends file_start → bridge forwards to each target head
Bridge waits for file_start_ack from all heads → sends file_start_ack to app
App sends first chunk → bridge forwards to all non-failed heads
Bridge waits for chunk_ack from all heads (timeout 1s, max 3 retries)
Failed heads after 3 retries → marked permanently failed, transfer continues
App sends next chunk only after receiving chunk_ack
After last chunk → app sends file_end → bridge forwards → reports transfer_complete
```

### Chunk ACK timeout
- Per-chunk timeout: **1000ms**
- Max retries: **3**
- After 3 retries, head is marked failed and excluded from remaining chunks
- Failed heads reported in `transfer_complete`

---

## Key constants (in bridge_main.cpp)

```cpp
ESPNOW_CHANNEL     = 1
SERIAL_BAUD        = 115200
SERIAL_BUF_SIZE    = 2048    // larger to fit base64 chunks
CHUNK_ACK_TIMEOUT  = 1000    // ms
CHUNK_MAX_RETRIES  = 3
MAX_HEADS          = 8
MAX_TRANSFER_HEADS = 8
```

---

## Key decisions & known issues

- Serial buffer is 2048 bytes (not 512) — needed for base64 chunk JSON
- Garbage bytes from ESP32 boot log appear on Serial at startup —
  the Flutter app filters them by checking for non-printable characters
- ESP-NOW broadcast peer must be registered before sending announces
- `onEspNowReceived` uses the **ESP-IDF 4.4.x** callback signature:
  `(const uint8_t* mac, const uint8_t* data, int len)` — NOT the 5.x
  `esp_now_recv_info_t*` version (board uses IDF 4.4.6)
- `esp_wifi_set_channel()` requires `#include <esp_wifi.h>`
- File list is automatically requested when a head announces itself

---

## platformio.ini key settings

```ini
platform = espressif32@6.6.0
board = esp32dev
framework = arduino
lib_deps =
    bblanchon/ArduinoJson@^7.0.0
monitor_speed = 115200
```