#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>

// === USER SETTINGS ===
#ifndef FB_COUNT
#define FB_COUNT 18  // queue of frames
#endif

static const uint32_t CAPTURE_INTERVAL_MS = 420; //~2.4fps

static const uint16_t UDP_PORT      = 5000;
static const uint16_t UDP_PAYLOAD   = 1000;
static const uint16_t UDP_PACING_MS = 4;

const char* RX_SSID = "RX_CAM";
const char* RX_PASS = "12345678";
IPAddress RX_IP(192,168,4,1);

// OV2640 Camera Pin Mapping
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y2_GPIO_NUM       11
#define Y3_GPIO_NUM       9
#define Y4_GPIO_NUM       8
#define Y5_GPIO_NUM       10
#define Y6_GPIO_NUM       12
#define Y7_GPIO_NUM       18
#define Y8_GPIO_NUM       17
#define Y9_GPIO_NUM       16
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

WiFiUDP udp;

// ---- UDP packet format ----
enum : uint8_t { PKT_START=1, PKT_DATA=2, PKT_END=3 };

#pragma pack(push, 1)
struct PktStart   { uint8_t type; uint32_t img_id; uint32_t total_len; };
struct PktDataHdr { uint8_t type; uint32_t img_id; uint16_t seq; uint16_t len; };
struct PktEnd     { uint8_t type; uint32_t img_id; uint32_t total_len; };
#pragma pack(pop)

// ---------- RSSI->distance rough model ----------
static const float TX_POWER_AT_1M_DBM = -45.0f;
static const float PATH_LOSS_N        = 2.2f;

static float estimateDistanceMetersFromRssi(int rssi_dbm) {
  float expv = (TX_POWER_AT_1M_DBM - (float)rssi_dbm) / (10.0f * PATH_LOSS_N);
  float d = powf(10.0f, expv);
  if (!isfinite(d)) d = -1.0f;
  return d;
}

// ---------- Frame item passed between tasks ----------
struct FrameItem {
  uint32_t img_id;
  uint32_t len;
  uint8_t* buf;   // PSRAM malloc
};

static QueueHandle_t qUDP = nullptr;

// ---- WiFi: long-range STA tuning ----
static void wifiTuneSTA_LongRange() {
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // 20 MHz bandwidth
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);

  // Force 11b for range/robustness (slower)
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

  Serial.println("✅ TX range mode: 11b + HT20 + max TX power");
}

static bool initCameraRangeFriendly() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size   = FRAMESIZE_UXGA; // 1600x1200
  config.jpeg_quality = 12;
  config.fb_count     = FB_COUNT;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init failed: 0x%x\n", err);
    return false;
  }

  Serial.printf("✅ Camera initialized (UXGA q=%d fb_count=%d)\n", config.jpeg_quality, config.fb_count);
  return true;
}

// ================= UDP SEND =================
static void udpSendFrame(const FrameItem& it) {
  PktStart s{PKT_START, it.img_id, it.len};
  udp.beginPacket(RX_IP, UDP_PORT);
  udp.write((uint8_t*)&s, sizeof(s));
  udp.endPacket();

  uint32_t remaining = it.len;
  const uint8_t* p = it.buf;
  uint16_t seq = 0;

  while (remaining > 0) {
    uint16_t chunk = (remaining > UDP_PAYLOAD) ? UDP_PAYLOAD : (uint16_t)remaining;
    PktDataHdr h{PKT_DATA, it.img_id, seq, chunk};

    udp.beginPacket(RX_IP, UDP_PORT);
    udp.write((uint8_t*)&h, sizeof(h));
    udp.write(p, chunk);
    udp.endPacket();

    p += chunk;
    remaining -= chunk;
    seq++;
    delay(UDP_PACING_MS);
  }

  PktEnd e{PKT_END, it.img_id, it.len};
  udp.beginPacket(RX_IP, UDP_PORT);
  udp.write((uint8_t*)&e, sizeof(e));
  udp.endPacket();
}

// ================= TASKS =================
static void taskCapture(void* pv) {
  uint32_t img_id = 0;

  for (;;) {
    img_id++;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("❌ Capture failed");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    uint32_t len = fb->len;
    Serial.printf("📸 img=%lu len=%lu bytes\n", (unsigned long)img_id, (unsigned long)len);

    uint8_t* bufUDP = (uint8_t*)ps_malloc(len);

    if (!bufUDP) {
      Serial.println("❌ PSRAM alloc failed (drop frame)");
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));
      continue;
    }

    memcpy(bufUDP, fb->buf, len);
    esp_camera_fb_return(fb);

    FrameItem b{img_id, len, bufUDP};

    if (xQueueSend(qUDP, &b, 0) != pdTRUE) {
      free(bufUDP);
      Serial.println("⚠️ qUDP full: dropped UDP frame");
    }

    vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));
  }
}

static void taskUDP(void* pv) {
  const int N = 10;
  float hist[N] = {0};
  int idx = 0, count = 0;

  uint32_t lastInfoMs = 0;

  for (;;) {
    FrameItem it;
    if (xQueueReceive(qUDP, &it, portMAX_DELAY) == pdTRUE) {

      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("⚠️ WiFi not connected; dropped UDP frame");
        free(it.buf);
        continue;
      }

      uint32_t t0 = millis();
      udpSendFrame(it);
      uint32_t dt = millis() - t0;
      if (dt == 0) dt = 1;

      float frameMbps = (it.len * 8.0f) / (dt * 1000.0f);

      hist[idx] = frameMbps;
      idx = (idx + 1) % N;
      if (count < N) count++;

      float avg = 0;
      for (int i = 0; i < count; i++) avg += hist[i];
      avg /= (float)count;

      Serial.printf("📡 TX img=%lu bytes=%lu dt=%lums thr=%.2f Mbps avg=%.2f Mbps\n",
                    (unsigned long)it.img_id,
                    (unsigned long)it.len,
                    (unsigned long)dt,
                    frameMbps, avg);

      free(it.buf);

      uint32_t now = millis();
      if (now - lastInfoMs >= 1000) {
        lastInfoMs = now;
        int rssi = WiFi.RSSI();
        float d_m = estimateDistanceMetersFromRssi(rssi);
        Serial.printf("📶 TX RSSI=%d dBm | estDist=%.1f m (VERY rough)\n", rssi, d_m);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);

  setCpuFrequencyMhz(240);
  Serial.printf("CPU MHz: %u\n", (unsigned)getCpuFrequencyMhz());

  Serial.printf("psramFound(): %s | size=%u\n",
                psramFound() ? "YES" : "NO", (unsigned)ESP.getPsramSize());

  if (!initCameraRangeFriendly()) {
    Serial.println("❌ Camera init failed");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  wifiTuneSTA_LongRange();

  WiFi.begin(RX_SSID, RX_PASS);
  Serial.print("Connecting to RX AP");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }

  Serial.println("\n✅ Connected");
  Serial.print("TX IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(0);

  qUDP = xQueueCreate(2, sizeof(FrameItem));
  if (!qUDP) {
    Serial.println("❌ Queue create failed");
    while (true) delay(1000);
  }

  xTaskCreatePinnedToCore(taskCapture, "capture", 8192, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskUDP,     "udp",     8192, nullptr, 2, nullptr, 0);

  Serial.printf("✅ TX started (UDP only) | FB_COUNT=%d\n", FB_COUNT);
}

void loop() {
  delay(1000);
}
