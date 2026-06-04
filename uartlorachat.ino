/*  LoRa Point-to-Point Chat  –  simple CSMA (send if silence present)
 *  Hardware : Ideaspark ESP32 0.96" OLED  +  DX-LR22-433T22D (LLCC68)
 *  OLED     : SDA=GPIO21  SCL=GPIO22  (I2C, U8g2)
 *  LoRa UART: RX=GPIO16   TX=GPIO17   (Serial2, 9600 8N1)
 *  LoRa M0+M1 tied to GND → transparent MODE0
 *
 *  Serial commands (115200 baud):
 *    any text + Enter          →  send
 *    /id                       →  own MAC
 *    /status                   →  statistics
 *
 *  Protocol header (17 char, length-framed):
 *    |PPPP|T|SS|HH|LL|
 *  *    PPPP = payload length (hex4)
 *    T    = M(essage) | B(roadcast) -- currently only M
 *    SS   = serial number of the message (hex2, overflow wraparound)
 *    HH,LL= source MAC (hex2)
 *
 *  Send: CSMA – 50ms silence need to be on this channel, else waiting
 *  Duplicate filter: last 8 (srcHH, srcLL, seq) megjegyezve
 *
 *  Dependency: U8g2 (olikraus/U8g2)
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// ═══════════════════════════════════════════════════════════════
//  CONFIGURATION
// ═══════════════════════════════════════════════════════════════
#define LORA_BAUD        9600
#define LORA_RX          16
#define LORA_TX          17
#define USB_BAUD         115200

#define HEADER_LEN       17      // |PPPP|T|SS|HH|LL|  = 17 char
#define MAX_PAYLOAD      80
#define MAX_PACKET       (HEADER_LEN + MAX_PAYLOAD + 1)

#define CSMA_QUIET_MS    50      // ? ms silence need to send 
#define CSMA_TIMEOUT_MS  3000   // if it doesn't work, throws it away

#define DUP_HISTORY      8
#define OLED_MSG_ROWS    4

// ═══════════════════════════════════════════════════════════════
//  OLED
// ═══════════════════════════════════════════════════════════════
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE, 22, 21);

// ═══════════════════════════════════════════════════════════════
//  TÍPUSOK
// ═══════════════════════════════════════════════════════════════
struct Packet {
    uint16_t payloadLen;
    char     type;
    uint8_t  seq;
    uint8_t  srcHH;
    uint8_t  srcLL;
    char     payload[MAX_PAYLOAD + 1];
};

struct DupEntry {
    uint8_t hh, ll, seq;
    bool    valid;
};

struct OledMsg {
    char text[48];
    bool valid;
};

// ═══════════════════════════════════════════════════════════════
//  GLOBÁLISOK
// ═══════════════════════════════════════════════════════════════
static uint8_t  ownHH = 0, ownLL = 0;
static uint8_t  txSeq = 0;
static uint32_t txCount = 0, rxCount = 0;

// CSMA: when was the last activity on the LoRa UART
static unsigned long lastLoraActivity = 0;

// RX state machine
static char     rxBuf[MAX_PACKET];
static uint16_t rxPos     = 0;
static bool     rxHdrDone = false;
static uint16_t rxExpPay  = 0;

// Duplicate filtering
static DupEntry dupHistory[DUP_HISTORY];
static uint8_t  dupIdx = 0;

// OLED message puffer (ring)
static OledMsg  oledMsgs[OLED_MSG_ROWS];
static uint8_t  oledNext = 0;

// ═══════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════
bool     macInit();
uint16_t proto_peek_plen(const char *h);
int      proto_build(char *out, char type, uint8_t seq,
                     uint8_t srcHH, uint8_t srcLL,
                     const char *pay, uint16_t plen);
bool     proto_parse(const char *raw, uint16_t rawLen, Packet &pkt);
bool     lora_receive(Packet &pkt);
bool     lora_send_csma(const char *raw, uint16_t len);
void     send_message(const char *text, uint16_t len);
void     handle_serial_cmd(const char *line);
bool     dup_check(uint8_t hh, uint8_t ll, uint8_t seq);
void     dup_record(uint8_t hh, uint8_t ll, uint8_t seq);
void     oled_push(const char *sender, const char *text);
void     oled_redraw();

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(USB_BAUD);
    Serial2.begin(LORA_BAUD, SERIAL_8N1, LORA_RX, LORA_TX);

    Wire.begin(21, 22);
    oled.begin();
    oled.setFont(u8g2_font_5x7_tr);
    oled.clearBuffer();
    oled.drawStr(0, 10, "LoRa Chat");
    oled.drawStr(0, 22, "Init...");
    oled.sendBuffer();

    for (int i = 0; i < DUP_HISTORY;  i++) dupHistory[i].valid = false;
    for (int i = 0; i < OLED_MSG_ROWS; i++) oledMsgs[i].valid  = false;

    lastLoraActivity = millis();

    if (!macInit()) {
        uint64_t chipId = ESP.getEfuseMac();
        ownHH = (uint8_t)((chipId >> 8) & 0xFF);
        ownLL = (uint8_t)(chipId & 0xFF);
        Serial.printf("[WARN] AT+MAC failed, chip ID: %02X,%02X\n", ownHH, ownLL);
    }

    char buf[32];
    snprintf(buf, sizeof(buf), "ID: %02X,%02X", ownHH, ownLL);
    oled.clearBuffer();
    oled.drawStr(0, 7,  "LoRa Chat");
    oled.drawStr(0, 17, buf);
    oled.drawStr(0, 27, "Done.");
    oled.sendBuffer();

    Serial.printf("[INFO] Own MAC: %02X,%02X\n", ownHH, ownLL);
    Serial.println("[INFO] Write something + Enter to send.");
    Serial.println("[INFO] Commands: /id  /status");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════
static char    usbLine[256];
static uint16_t usbLen = 0;

void loop()
{
    // ── USB serial input ────────────────────────────────────────
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (usbLen > 0) {
                usbLine[usbLen] = '\0';
                handle_serial_cmd(usbLine);
                usbLen = 0;
            }
        } else if (usbLen < 255) {
            usbLine[usbLen++] = c;
        }
    }

    // ── LoRa transmit ─────────────────────────────────────────────
    Packet pkt;
    if (lora_receive(pkt)) {
        // own echo filter
        if (pkt.srcHH == ownHH && pkt.srcLL == ownLL) {
        } else if (pkt.type == 'M') {
            if (!dup_check(pkt.srcHH, pkt.srcLL, pkt.seq)) {
                dup_record(pkt.srcHH, pkt.srcLL, pkt.seq);
                rxCount++;
                char sender[12];
                snprintf(sender, sizeof(sender), "%02X,%02X", pkt.srcHH, pkt.srcLL);
                oled_push(sender, pkt.payload);
                Serial.printf("[RX] [%02X,%02X]: %s\n", pkt.srcHH, pkt.srcLL, pkt.payload);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════
//  MAC INIT
// ═══════════════════════════════════════════════════════════════
bool macInit()
{
    delay(200);
    Serial2.print("+++\r\n");
    delay(500);
    while (Serial2.available()) Serial2.read();

    Serial2.print("AT+MAC\r\n");
    delay(400);

    char resp[64];
    uint16_t rLen = 0;
    unsigned long t = millis();
    while (millis() - t < 600 && rLen < 63) {
        if (Serial2.available())
            resp[rLen++] = (char)Serial2.read();
    }
    resp[rLen] = '\0';

    Serial2.print("+++\r\n");
    delay(300);
    while (Serial2.available()) Serial2.read();

    if (strstr(resp, "ff,ff") || strstr(resp, "FF,FF") || rLen < 4)
        return false;

    unsigned a = 0, b = 0;
    for (int i = (int)rLen - 5; i >= 0; i--) {
        if (sscanf(resp + i, "%2X,%2X", &a, &b) == 2) {
            ownHH = (uint8_t)a;
            ownLL = (uint8_t)b;
            return true;
        }
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════
//  PROTOCOL
// ═══════════════════════════════════════════════════════════════
uint16_t proto_peek_plen(const char *h)
{
    if (h[0] != '|') return 0xFFFF;
    unsigned v = 0;
    if (sscanf(h + 1, "%4X", &v) != 1) return 0xFFFF;
    return (uint16_t)v;
}

int proto_build(char *out, char type, uint8_t seq,
                uint8_t srcHH, uint8_t srcLL,
                const char *pay, uint16_t plen)
{
    if (plen > MAX_PAYLOAD) return -1;
    // |PPPP|T|SS|HH|LL|  = 17 karakter
    int hlen = snprintf(out, HEADER_LEN + 1,
        "|%04X|%c|%02X|%02X|%02X|",
        plen, type, seq, srcHH, srcLL);
    if (hlen != HEADER_LEN) return -1;
    memcpy(out + HEADER_LEN, pay, plen);
    out[HEADER_LEN + plen] = '\0';
    return HEADER_LEN + plen;
}

bool proto_parse(const char *raw, uint16_t rawLen, Packet &pkt)
{
    if (rawLen < (uint16_t)HEADER_LEN) return false;
    if (raw[0] != '|' || raw[HEADER_LEN - 1] != '|') return false;

    unsigned plen, seq, hh, ll;
    char type;
    // |PPPP|T|SS|HH|LL|
    int n = sscanf(raw, "|%4X|%c|%2X|%2X|%2X|",
                   &plen, &type, &seq, &hh, &ll);
    if (n != 5) return false;
    if (rawLen != (uint16_t)(HEADER_LEN + plen)) return false;
    if (plen > MAX_PAYLOAD) return false;

    pkt.payloadLen = (uint16_t)plen;
    pkt.type       = type;
    pkt.seq        = (uint8_t)seq;
    pkt.srcHH      = (uint8_t)hh;
    pkt.srcLL      = (uint8_t)ll;
    memcpy(pkt.payload, raw + HEADER_LEN, plen);
    pkt.payload[plen] = '\0';
    return true;
}

// ═══════════════════════════════════════════════════════════════
//  LoRa UART – length-framed RX
// ═══════════════════════════════════════════════════════════════
bool lora_receive(Packet &pkt)
{
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        lastLoraActivity = millis();    // CSMA: saving the last activity

        if (rxPos == 0 && c != '|') continue;  // sync

        rxBuf[rxPos++] = c;

        if (!rxHdrDone && rxPos == HEADER_LEN) {
            rxExpPay = proto_peek_plen(rxBuf);
            if (rxExpPay > MAX_PAYLOAD) {
                rxPos = 0; rxHdrDone = false; continue;
            }
            rxHdrDone = true;
            if (rxExpPay == 0) {
                bool ok = proto_parse(rxBuf, HEADER_LEN, pkt);
                rxPos = 0; rxHdrDone = false;
                if (ok) return true;
            }
        } else if (rxHdrDone) {
            if ((rxPos - HEADER_LEN) >= rxExpPay) {
                bool ok = proto_parse(rxBuf, HEADER_LEN + rxExpPay, pkt);
                rxPos = 0; rxHdrDone = false;
                if (ok) return true;
            }
        }

        if (rxPos >= MAX_PACKET) { rxPos = 0; rxHdrDone = false; }
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════
//  CSMA transmit
// ═══════════════════════════════════════════════════════════════
bool lora_send_csma(const char *raw, uint16_t len)
{
    unsigned long start = millis();
    while (true) {
        // Is there any activity on the channel?
        // Flush any incoming bytes and update lastLoraActivity
        Packet dummy;
        lora_receive(dummy);  // just to update activity

        unsigned long quiet = millis() - lastLoraActivity;
        if (quiet >= CSMA_QUIET_MS) {
            // There is silence - transmit
            Serial2.write((const uint8_t *)raw, len);
            return true;
        }

        if (millis() - start > CSMA_TIMEOUT_MS) {
            Serial.println("[CSMA] Timeout – csatorna foglalt, uzenet elvetve.");
            return false;
        }
        delay(5);
    }
}

// ═══════════════════════════════════════════════════════════════
//  TEXT SENDING (with segmentation)
// ═══════════════════════════════════════════════════════════════
void send_message(const char *text, uint16_t totalLen)
{
    uint8_t segTotal = (totalLen + MAX_PAYLOAD - 1) / MAX_PAYLOAD;
    if (segTotal == 0) segTotal = 1;

    for (uint8_t i = 0; i < segTotal; i++) {
        uint16_t offset = i * MAX_PAYLOAD;
        uint16_t chunk  = totalLen - offset;
        if (chunk > MAX_PAYLOAD) chunk = MAX_PAYLOAD;

        // Sliceing on the UTF-8 border
        while (chunk > 0 && offset + chunk < totalLen &&
               (text[offset + chunk] & 0xC0) == 0x80) chunk--;

        char buf[MAX_PACKET];
        int len = proto_build(buf, 'M', txSeq,
                              ownHH, ownLL,
                              text + offset, chunk);
        if (len < 0) continue;

        if (lora_send_csma(buf, (uint16_t)len)) {
            txCount++;
            Serial.printf("[TX] seq=%02X chunk=%d/%d\n", txSeq, i + 1, segTotal);
        }
        txSeq++;

        // Short break between segments
        if (segTotal > 1) delay(200);
    }
}

// ═══════════════════════════════════════════════════════════════
//  SERIAL COMMAND PROCESSING
// ═══════════════════════════════════════════════════════════════
void handle_serial_cmd(const char *line)
{
    if (line[0] == '/') {
        if (strcmp(line, "/id") == 0) {
            Serial.printf("[ID] %02X,%02X\n", ownHH, ownLL);
        } else if (strcmp(line, "/status") == 0) {
            uint32_t up = millis() / 1000;
            Serial.printf("[STATUS] MAC=%02X,%02X  TX=%lu  RX=%lu  uptime=%02lu:%02lu:%02lu\n",
                ownHH, ownLL, txCount, rxCount,
                up / 3600, (up % 3600) / 60, up % 60);
        } else {
            Serial.println("[ERR] Ismeretlen parancs. /id  /status");
        }
        return;
    }

    // Any other text → send
    uint16_t mlen = (uint16_t)strlen(line);
    if (mlen == 0) return;

    char sender[10];
    snprintf(sender, sizeof(sender), "%02X,%02X", ownHH, ownLL);
    oled_push(sender, line);
    send_message(line, mlen);
}

// ═══════════════════════════════════════════════════════════════
//  DUPLICATE FILTERING
// ═══════════════════════════════════════════════════════════════
bool dup_check(uint8_t hh, uint8_t ll, uint8_t seq)
{
    for (int i = 0; i < DUP_HISTORY; i++) {
        if (dupHistory[i].valid &&
            dupHistory[i].hh  == hh &&
            dupHistory[i].ll  == ll &&
            dupHistory[i].seq == seq)
            return true;
    }
    return false;
}

void dup_record(uint8_t hh, uint8_t ll, uint8_t seq)
{
    dupHistory[dupIdx] = {hh, ll, seq, true};
    dupIdx = (dupIdx + 1) % DUP_HISTORY;
}

// ═══════════════════════════════════════════════════════════════
//  OLED
// ═══════════════════════════════════════════════════════════════
void oled_push(const char *sender, const char *text)
{
    OledMsg &m = oledMsgs[oledNext];
    snprintf(m.text, sizeof(m.text), "[%s]:%s", sender, text);
    m.valid = true;
    oledNext = (oledNext + 1) % OLED_MSG_ROWS;
    oled_redraw();
}

void oled_redraw()
{
    oled.clearBuffer();
    oled.setFont(u8g2_font_5x7_tr);

    // Status line
    char status[32];
    uint32_t up = millis() / 1000;
    snprintf(status, sizeof(status), "%02X,%02X  %02lu:%02lu:%02lu",
             ownHH, ownLL,
             up / 3600, (up % 3600) / 60, up % 60);
    oled.drawStr(0, 7, status);
    oled.drawHLine(0, 9, 128);

    // Text line (last to newest)
    const uint8_t rowH = 13;
    uint8_t idx = oledNext;
    for (uint8_t r = 0; r < OLED_MSG_ROWS; r++) {
        if (oledMsgs[idx].valid)
            oled.drawStr(0, 11 + (r + 1) * rowH, oledMsgs[idx].text);
        idx = (idx + 1) % OLED_MSG_ROWS;
    }

    oled.sendBuffer();
}
