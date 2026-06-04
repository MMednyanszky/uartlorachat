
# LoRa Point-to-Point Chat

A simple CSMA-based LoRa text chat for two or more ESP32 nodes. Messages are typed over USB serial and displayed on a small OLED screen.

---

## Hardware

| Component | Part |
|-----------|------|
| Microcontroller | Ideaspark ESP32 (0.96" OLED on-board) |
| LoRa module | DX-LR22-433T22D (LLCC68 chip, 433 MHz) |

### Pin connections

| Signal | GPIO |
|--------|------|
| OLED SDA | 21 |
| OLED SCL | 22 |
| LoRa UART RX | 16 |
| LoRa UART TX | 17 |
| LoRa M0, M1 | GND (transparent Mode 0) |

---

## Dependencies

Install via the Arduino Library Manager:

- **U8g2** by olikraus

---

## Building & Flashing

1. Open `lora_chat.ino` in the Arduino IDE (or PlatformIO).
2. Select board: **ESP32 Dev Module** (or the Ideaspark variant).
3. Set upload speed to **921600** and USB CDC on boot to **Enabled**.
4. Flash. Open the Serial Monitor at **115200 baud**.

---

## Usage

Connect via any serial terminal at 115200 baud.

| Input | Action |
|-------|--------|
| Any text + Enter | Send message to all nodes |
| `/id` | Print own MAC address |
| `/status` | Print TX/RX counters and uptime |

Received messages appear both in the serial monitor and on the OLED.

---

## Protocol

Every packet is a length-framed ASCII frame:

```
|PPPP|T|SS|HH|LL|<payload>
```

| Field | Width | Description |
|-------|-------|-------------|
| `PPPP` | 4 hex chars | Payload length |
| `T` | 1 char | Frame type: `M` = message |
| `SS` | 2 hex chars | Sequence number (wraps at 0xFF) |
| `HH` | 2 hex chars | Source MAC high byte |
| `LL` | 2 hex chars | Source MAC low byte |
| payload | up to 80 bytes | UTF-8 text |

The fixed 17-character header makes framing straightforward: read 17 bytes, parse payload length, then read exactly that many more bytes.

Long messages are automatically split into 80-byte segments with a 200 ms gap between them. Segments are reassembled in order on the receiving side.

---

## MAC address

On startup the firmware queries the module with `AT+MAC`. If the query fails or returns `FF,FF`, the last two bytes of the ESP32's eFuse MAC are used as a fallback.

---

## Channel access (CSMA)

Before transmitting, the firmware listens on the UART. Transmission starts only after **50 ms of silence**. If the channel stays busy for more than **3 seconds** the message is dropped and a warning is printed to serial.

---

## Duplicate filtering

Each node keeps a ring buffer of the last 8 `(srcHH, srcLL, seq)` tuples. Packets matching an existing entry are silently discarded, preventing duplicate display when a node re-sends a segment.

---

## OLED display

```
AB,CD  00:12:34          ← own MAC + uptime
────────────────────────
[AB,CD]:hello            ← sent messages
[12,34]:hi there         ← received messages
...
```

The display holds the last 4 messages in a ring buffer. The status bar is always visible at the top.

---

## Limitations & known issues

- No acknowledgement or retransmission — delivery is best-effort.
- The duplicate filter (depth 8) can miss retransmits after the buffer wraps.
- CSMA detects activity only via UART idle time, not via an actual carrier-sense pin; simultaneous transmissions from two nodes will collide.
- Messages longer than 80 characters are split but not explicitly reassembled — the receiver shows each segment separately.