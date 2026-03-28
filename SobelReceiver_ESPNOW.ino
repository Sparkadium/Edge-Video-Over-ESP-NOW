/*
 * SobelReceiver_ESPNOW.ino — CYD wireless edge-detection display
 * 
 * Receives RLE-compressed 1-bit Sobel edge frames over ESP-NOW
 * from SobelSender.ino (XIAO ESP32-S3 Sense) and draws to TFT.
 *
 * The rleDecode() and drawFrame() are identical to the WiFi test
 * version — only the data source changed from HTTP to ESP-NOW.
 *
 * Board: ESP32 Dev Module (for ESP32-2432S028 / CYD)
 * Requires: TFT_eSPI (configured for ILI9341 320x240)
 */

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <TFT_eSPI.h>

// ── CONFIG ─────────────────────────────────────────────
// Display colors — same options as the WiFi test version
const uint16_t COLOR_EDGE = TFT_WHITE;
const uint16_t COLOR_BG   = TFT_BLACK;
// Try: TFT_GREEN/TFT_BLACK for CRT, TFT_CYAN/TFT_DARKGREY for blueprint

#define FRAME_W       80
#define FRAME_H       60
#define DISPLAY_W     320
#define DISPLAY_H     240
#define SCALE         4
#define PACKED_SIZE   (FRAME_W * FRAME_H / 8)  // 600
#define RLE_BUF_SIZE  2048
#define BACKLIGHT_PIN 21
#define ESPNOW_CHUNK  240

// Packet header sizes (must match sender)
#define HEADER_SIZE_FIRST  4
#define HEADER_SIZE_CONT   2

TFT_eSPI tft = TFT_eSPI();

// ── DOUBLE BUFFER ──────────────────────────────────────
// Two RLE buffers: one being filled by ESP-NOW, one being drawn.
// This prevents tearing and lets receive + draw overlap.
uint8_t  rleBufA[RLE_BUF_SIZE];
uint8_t  rleBufB[RLE_BUF_SIZE];
uint8_t* rleReceiving = rleBufA;    // ESP-NOW callback writes here
uint8_t* rleDrawing   = rleBufB;    // main loop reads from here

uint8_t  packedBuf[PACKED_SIZE];
uint16_t lineBuf[DISPLAY_W];

// Frame assembly state (accessed from ESP-NOW callback)
volatile int      expectedPackets = 0;
volatile int      receivedPackets = 0;
volatile int      totalRleLen     = 0;
volatile int      rleWriteOffset  = 0;
volatile bool     frameReady      = false;

// Stats
unsigned long frameCount    = 0;
unsigned long lastFpsTime   = 0;
float         currentFps    = 0;
unsigned long lastFrameTime = 0;

// ── RLE DECODE ─────────────────────────────────────────
// Identical to WiFi test version.
void rleDecode(const uint8_t* rle, int rleLen, uint8_t* out, int outLen) {
    int ri = 0, oi = 0;
    while (ri < rleLen - 1 && oi < outLen) {
        uint8_t run = rle[ri++];
        uint8_t val = rle[ri++];
        for (int j = 0; j < run && oi < outLen; j++) {
            out[oi++] = val;
        }
    }
    while (oi < outLen) out[oi++] = 0;
}

// ── DRAW FRAME ─────────────────────────────────────────
// 80×60 → 4× upscale to 320×240 TFT. Each source pixel becomes a 4×4 block.
void drawFrame() {
    tft.startWrite();
    for (int y = 0; y < FRAME_H; y++) {
        int rowOffset = y * FRAME_W;
        // Build one display row: each source pixel 4× horizontally
        for (int x = 0; x < FRAME_W; x++) {
            int bitIdx  = rowOffset + x;
            int byteIdx = bitIdx >> 3;
            int bitPos  = 7 - (bitIdx & 7);
            uint16_t color = (packedBuf[byteIdx] >> bitPos) & 1 ? COLOR_EDGE : COLOR_BG;
            int dx = x * SCALE;
            lineBuf[dx]     = color;
            lineBuf[dx + 1] = color;
            lineBuf[dx + 2] = color;
            lineBuf[dx + 3] = color;
        }
        // Push same row 4 times for vertical scaling
        int dy = y * SCALE;
        tft.pushImage(0, dy,     DISPLAY_W, 1, lineBuf);
        tft.pushImage(0, dy + 1, DISPLAY_W, 1, lineBuf);
        tft.pushImage(0, dy + 2, DISPLAY_W, 1, lineBuf);
        tft.pushImage(0, dy + 3, DISPLAY_W, 1, lineBuf);
    }
    tft.endWrite();
}

// ── ESP-NOW RECEIVE CALLBACK ───────────────────────────
// Called from WiFi task — keep it fast, just copy data.
void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len < 2) return;

    uint8_t pktIdx   = data[0];
    uint8_t pktTotal = data[1];

    if (pktIdx == 0) {
        // First packet of new frame
        if (len < HEADER_SIZE_FIRST) return;

        // Read total RLE length from header
        totalRleLen     = data[2] | (data[3] << 8);
        expectedPackets = pktTotal;
        receivedPackets = 0;
        rleWriteOffset  = 0;

        // Sanity check
        if (totalRleLen > RLE_BUF_SIZE) {
            totalRleLen = 0;
            return;
        }

        // Copy payload from first packet
        int payloadLen = len - HEADER_SIZE_FIRST;
        if (rleWriteOffset + payloadLen <= RLE_BUF_SIZE) {
            memcpy(rleReceiving + rleWriteOffset, data + HEADER_SIZE_FIRST, payloadLen);
            rleWriteOffset += payloadLen;
        }
        receivedPackets = 1;

    } else {
        // Continuation packet
        if (expectedPackets == 0) return;  // no frame in progress
        if (pktIdx != receivedPackets) {
            // Out of order — drop this frame
            expectedPackets = 0;
            return;
        }

        int payloadLen = len - HEADER_SIZE_CONT;
        if (rleWriteOffset + payloadLen <= RLE_BUF_SIZE) {
            memcpy(rleReceiving + rleWriteOffset, data + HEADER_SIZE_CONT, payloadLen);
            rleWriteOffset += payloadLen;
        }
        receivedPackets++;
    }

    // Frame complete?
    if (receivedPackets == expectedPackets && expectedPackets > 0) {
        // Swap buffers
        uint8_t* tmp  = rleDrawing;
        rleDrawing    = rleReceiving;
        rleReceiving  = tmp;
        frameReady    = true;
        expectedPackets = 0;
    }
}

// ── STATUS DISPLAY ─────────────────────────────────────
void drawStatus(const char* msg, uint16_t color) {
    tft.fillRect(0, 0, DISPLAY_W, 20, TFT_BLACK);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(msg, 4, 4, 2);
}

void drawFps() {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.1f fps | ESP-NOW", currentFps);
    tft.fillRect(DISPLAY_W - 130, 0, 130, 16, TFT_BLACK);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(buf, DISPLAY_W - 4, 2, 1);
}

// ── SETUP ──────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Sobel Receiver (ESP-NOW) ===");

    // Backlight
    pinMode(BACKLIGHT_PIN, OUTPUT);
    digitalWrite(BACKLIGHT_PIN, HIGH);

    // Display
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    // WiFi station mode (required for ESP-NOW, no connection needed)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Read real MAC from hardware (WiFi.macAddress() can return 00:00:00:00:00:00)
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char macStr[20];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    Serial.printf("\n>> RECEIVER MAC: %s <<\n", macStr);
    Serial.println("   Copy this into SobelSender.ino receiverMAC[]\n");
    Serial.printf("   receiverMAC[] = {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X};\n\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Show MAC on screen too
    char macMsg[40];
    snprintf(macMsg, sizeof(macMsg), "MAC: %s", macStr);
    drawStatus(macMsg, TFT_CYAN);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        drawStatus("ESP-NOW init FAILED", TFT_RED);
        Serial.println("ESP-NOW init failed!");
        while (true) delay(1000);
    }

    esp_now_register_recv_cb(onReceive);

    Serial.println("ESP-NOW ready, waiting for frames...");

    // Show waiting message
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Waiting for sender...", DISPLAY_W / 2, DISPLAY_H / 2, 2);

    lastFpsTime = millis();
}

// ── LOOP ───────────────────────────────────────────────
void loop() {
    if (frameReady) {
        frameReady = false;

        // Decompress and draw — same pipeline as WiFi test
        rleDecode(rleDrawing, totalRleLen, packedBuf, PACKED_SIZE);
        drawFrame();

        frameCount++;
        lastFrameTime = millis();

        // FPS every 2 seconds
        unsigned long now = millis();
        if (now - lastFpsTime >= 2000) {
            currentFps = (float)frameCount / ((now - lastFpsTime) / 1000.0);
            frameCount = 0;
            lastFpsTime = now;
            Serial.printf("FPS: %.1f\n", currentFps);
            drawFps();
        }
    }

    // Show "no signal" if nothing received for 5 seconds
    if (lastFrameTime > 0 && millis() - lastFrameTime > 5000) {
        drawStatus("Signal lost...", TFT_RED);
        lastFrameTime = millis();  // reset so we don't spam
    }

    delay(1);  // yield
}
