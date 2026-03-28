/*
 * SobelSender.ino — XIAO ESP32-S3 Sense edge-detection camera sender
 * 
 * 160x120 capture → Sobel → 1-bit → RLE → ESP-NOW to CYD.
 * CYD upscales 2× to fill 320×240 TFT — chunky pixel art look.
 *
 * Board: XIAO_ESP32S3 ("XIAO ESP32S3 Sense")
 * Tools → PSRAM: "OPI PSRAM"
 */

#include "esp_camera.h"
#include <esp_now.h>
#include <WiFi.h>

// ── CONFIG ─────────────────────────────────────────────
uint8_t receiverMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // replace with CYD MAC
int SOBEL_THRESHOLD = 80;
bool INVERT = false;

// ── CAMERA PINS (XIAO ESP32-S3 Sense) ─────────────────
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// ── FRAME CONSTANTS ────────────────────────────────────
#define CAP_W         160   // camera capture (QQVGA)
#define CAP_H         120
#define FRAME_W       80    // after 2x2 downsample
#define FRAME_H       60
#define PIXEL_COUNT   (FRAME_W * FRAME_H)  // 4800
#define PACKED_SIZE   (PIXEL_COUNT / 8)     // 600
#define RLE_BUF_SIZE  2048
#define ESPNOW_CHUNK  240

#define HEADER_SIZE_FIRST  4
#define HEADER_SIZE_CONT   2

// ── BUFFERS ────────────────────────────────────────────
uint8_t* downBuf   = nullptr;   // downsampled grayscale (80x60)
uint8_t* edgeBuf   = nullptr;
uint8_t* packedBuf = nullptr;
uint8_t* rleBuf    = nullptr;
uint8_t  pktBuf[250];

unsigned long frameCount = 0;
unsigned long lastStatsTime = 0;
volatile bool sendBusy = false;

void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
    sendBusy = false;
}

// ── DOWNSAMPLE ─────────────────────────────────────────
// Average 2×2 blocks from 160×120 capture → 80×60. Also reduces noise.
void downsample(const uint8_t* src) {
    for (int y = 0; y < FRAME_H; y++) {
        for (int x = 0; x < FRAME_W; x++) {
            int sx = x * 2;
            int sy = y * 2;
            int si = sy * CAP_W + sx;
            uint16_t avg = (uint16_t)src[si] + src[si + 1] 
                         + src[si + CAP_W] + src[si + CAP_W + 1];
            downBuf[y * FRAME_W + x] = avg >> 2;
        }
    }
}

// ── SOBEL FILTER ───────────────────────────────────────
void sobelFilter() {
    memset(edgeBuf, 0, PIXEL_COUNT);
    for (int y = 1; y < FRAME_H - 1; y++) {
        for (int x = 1; x < FRAME_W - 1; x++) {
            int i = y * FRAME_W + x;

            int gx = -downBuf[i - FRAME_W - 1] + downBuf[i - FRAME_W + 1]
                    - 2 * downBuf[i - 1]         + 2 * downBuf[i + 1]
                    - downBuf[i + FRAME_W - 1]   + downBuf[i + FRAME_W + 1];

            int gy = -downBuf[i - FRAME_W - 1] - 2 * downBuf[i - FRAME_W] - downBuf[i - FRAME_W + 1]
                    + downBuf[i + FRAME_W - 1]  + 2 * downBuf[i + FRAME_W] + downBuf[i + FRAME_W + 1];

            int mag = abs(gx) + abs(gy);

            uint8_t edge = (mag > SOBEL_THRESHOLD * 2) ? 1 : 0;
            if (INVERT) edge = 1 - edge;
            edgeBuf[i] = edge;
        }
    }
}

// ── BIT PACK ───────────────────────────────────────────
void bitPack() {
    memset(packedBuf, 0, PACKED_SIZE);
    for (int i = 0; i < PIXEL_COUNT; i++) {
        if (edgeBuf[i]) {
            packedBuf[i >> 3] |= 1 << (7 - (i & 7));
        }
    }
}

// ── RLE ENCODE ─────────────────────────────────────────
int rleEncode() {
    int ri = 0, i = 0;
    while (i < PACKED_SIZE && ri < RLE_BUF_SIZE - 1) {
        uint8_t val = packedBuf[i];
        int run = 1;
        while (i + run < PACKED_SIZE && packedBuf[i + run] == val && run < 255) run++;
        rleBuf[ri++] = (uint8_t)run;
        rleBuf[ri++] = val;
        i += run;
    }
    return ri;
}

// ── SEND FRAME ─────────────────────────────────────────
void sendFrame(int rleLen) {
    int totalPackets = 1;
    int remaining = rleLen - (ESPNOW_CHUNK - HEADER_SIZE_FIRST);
    if (remaining > 0) {
        totalPackets += (remaining + (ESPNOW_CHUNK - HEADER_SIZE_CONT) - 1) / (ESPNOW_CHUNK - HEADER_SIZE_CONT);
    }
    if (totalPackets > 255) totalPackets = 255;

    int rleOffset = 0;

    for (int pkt = 0; pkt < totalPackets; pkt++) {
        int headerSize, payloadMax, pktLen;

        if (pkt == 0) {
            pktBuf[0] = 0;
            pktBuf[1] = (uint8_t)totalPackets;
            pktBuf[2] = rleLen & 0xFF;
            pktBuf[3] = (rleLen >> 8) & 0xFF;
            headerSize = HEADER_SIZE_FIRST;
        } else {
            pktBuf[0] = (uint8_t)pkt;
            pktBuf[1] = (uint8_t)totalPackets;
            headerSize = HEADER_SIZE_CONT;
        }
        payloadMax = ESPNOW_CHUNK - headerSize;

        int dataLen = min(payloadMax, rleLen - rleOffset);
        memcpy(pktBuf + headerSize, rleBuf + rleOffset, dataLen);
        pktLen = headerSize + dataLen;
        rleOffset += dataLen;

        unsigned long waitStart = millis();
        while (sendBusy && (millis() - waitStart < 50)) {
            delayMicroseconds(50);
        }

        sendBusy = true;
        esp_err_t result = esp_now_send(receiverMAC, pktBuf, pktLen);
        if (result != ESP_OK) {
            sendBusy = false;
            return;
        }

        delayMicroseconds(100);
    }
}

// ── SETUP ──────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== Sobel Sender 80x60 ===\n");

    downBuf   = (uint8_t*)ps_malloc(PIXEL_COUNT);
    edgeBuf   = (uint8_t*)ps_malloc(PIXEL_COUNT);
    packedBuf = (uint8_t*)ps_malloc(PACKED_SIZE);
    rleBuf    = (uint8_t*)ps_malloc(RLE_BUF_SIZE);

    if (!downBuf || !edgeBuf || !packedBuf || !rleBuf) {
        Serial.println("PSRAM allocation failed!");
        while (true) delay(1000);
    }

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = FRAMESIZE_QQVGA;      // 160x120
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init FAILED: 0x%x\n", err);
        while (true) delay(1000);
    }
    Serial.println("Camera OK! (160x120, downsampled to 80x60)");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.printf("Sender MAC: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        while (true) delay(1000);
    }
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, receiverMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add peer!");
        while (true) delay(1000);
    }

    Serial.printf("Threshold: %d | Invert: %s\n", SOBEL_THRESHOLD, INVERT ? "yes" : "no");
    Serial.println("\nSending frames...\n");
    lastStatsTime = millis();
}

// ── LOOP ───────────────────────────────────────────────
void loop() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { delay(10); return; }

    downsample(fb->buf);
    esp_camera_fb_return(fb);

    sobelFilter();

    bitPack();
    int rleLen = rleEncode();
    sendFrame(rleLen);

    frameCount++;

    unsigned long now = millis();
    if (now - lastStatsTime >= 3000) {
        float fps = (float)frameCount / ((now - lastStatsTime) / 1000.0);
        int packets = 1;
        int rem = rleLen - (ESPNOW_CHUNK - HEADER_SIZE_FIRST);
        if (rem > 0) packets += (rem + (ESPNOW_CHUNK - HEADER_SIZE_CONT) - 1) / (ESPNOW_CHUNK - HEADER_SIZE_CONT);

        Serial.printf("FPS: %.1f | RLE: %d bytes | Packets: %d | PSRAM free: %d\n",
            fps, rleLen, packets, ESP.getFreePsram());

        frameCount = 0;
        lastStatsTime = now;
    }
}
