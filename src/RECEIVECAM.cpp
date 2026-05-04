#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include "SD_MMC.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <Preferences.h>

// ================= SD PINS (1-bit SDMMC) =================
#define SD_CLK 38
#define SD_CMD 39
#define SD_D0  40

// ================= RX AP =================
const char* RX_SSID = "RX_CAM";
const char* RX_PASS = "12345678";

IPAddress RX_IP(192,168,4,1);
IPAddress RX_GW(192,168,4,1);
IPAddress RX_MASK(255,255,255,0);

static const uint16_t UDP_PORT = 5000;

// Change channel here (1 / 6 / 11)
static const uint8_t AP_CHANNEL = 6;

WiFiUDP udp;
WebServer server(80);

// ---- UDP packet format (MUST MATCH TX) ----
enum : uint8_t { PKT_START=1, PKT_DATA=2, PKT_END=3 };

#pragma pack(push, 1)
struct PktStart   { uint8_t type; uint32_t img_id; uint32_t total_len; };
struct PktDataHdr { uint8_t type; uint32_t img_id; uint16_t seq; uint16_t len; };
struct PktEnd     { uint8_t type; uint32_t img_id; uint32_t total_len; };
#pragma pack(pop)

// ---------- TUNING ----------
static const uint32_t FRAME_TIMEOUT_MS = 1500;
static const uint32_t MAX_JPEG_BYTES   = 3 * 1024 * 1024;
static const uint16_t UDP_PAYLOAD_MAX  = 1200;   // TX uses 1000

// ---------- Latest image (served over HTTP) ----------
static uint8_t* g_latest = nullptr;   // PSRAM buffer
static size_t   g_latest_len = 0;
static uint32_t g_latest_id  = 0;
static SemaphoreHandle_t g_latest_mutex;

// ---------- SD run folder ----------
Preferences prefs;
static char RUN_DIR[32] = "/DCIM/run_000000";

// ---------- SD save queue ----------
struct SaveItem {
  uint32_t img_id;
  uint32_t len;
  uint8_t* buf; // owned by SD task
};
static QueueHandle_t qSave = nullptr;

// ---------- RX frame reassembly state ----------
static uint32_t cur_id = 0;
static uint32_t cur_total = 0;
static uint32_t cur_got = 0;
static uint8_t* cur_buf = nullptr;
static uint32_t cur_last_ms = 0;

// ---------- metrics ----------
static bool last_save_ok = false;
static String last_save_path = "";
static float last_mbps = 0.0f;
static uint32_t frame_start_ms = 0;

// ================= SD =================
static bool initSD() {
  Serial.println("Mounting RX SD...");
  SD_MMC.setPins(SD_CMD, SD_CLK, SD_D0);
  if (!SD_MMC.begin("/sdcard", true)) return false;
  if (SD_MMC.cardType() == CARD_NONE) return false;
  if (!SD_MMC.exists("/DCIM")) SD_MMC.mkdir("/DCIM");
  Serial.println("RX SD mounted");
  return true;
}

static void makeNewRunFolder() {
  if (!SD_MMC.exists("/DCIM")) SD_MMC.mkdir("/DCIM");

  prefs.begin("rx", false);
  uint32_t run = prefs.getUInt("run", 0) + 1;
  prefs.putUInt("run", run);
  prefs.end();

  snprintf(RUN_DIR, sizeof(RUN_DIR), "/DCIM/run_%06lu", (unsigned long)run);
  if (!SD_MMC.exists(RUN_DIR)) SD_MMC.mkdir(RUN_DIR);
  Serial.printf("RX run folder: %s\n", RUN_DIR);
}

static void sdWriteJpeg(const SaveItem& it) {
  char path[96];
  snprintf(path, sizeof(path), "%s/rx_%06lu.jpg", RUN_DIR, (unsigned long)it.img_id);

  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("RX SD open failed: %s\n", path);
    last_save_ok = false;
    last_save_path = path;
    return;
  }
  size_t w = f.write(it.buf, it.len);
  f.close();

  last_save_ok = (w == it.len);
  last_save_path = path;

  if (last_save_ok) Serial.printf("RX saved %s (%lu bytes)\n", path, (unsigned long)it.len);
  else              Serial.printf("RX SD short write: %s\n", path);
}

static void taskSD(void* pv) {
  for (;;) {
    SaveItem it;
    if (xQueueReceive(qSave, &it, portMAX_DELAY) == pdTRUE) {
      sdWriteJpeg(it);
      free(it.buf);
    }
  }
}

// ================= Wi-Fi AP tuning =================
static void wifiStartAP_LongRange() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  delay(100);

  WiFi.softAPConfig(RX_IP, RX_GW, RX_MASK);

  // IMPORTANT: pass channel + not hidden + max clients
  bool ok = WiFi.softAP(RX_SSID, RX_PASS, AP_CHANNEL, 0, 8);
  Serial.printf("RX AP started ok=%d SSID=%s CH=%d IP=%s\n",
                ok, RX_SSID, AP_CHANNEL, WiFi.softAPIP().toString().c_str());

  // range/robust settings (slower)
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  esp_wifi_set_max_tx_power(78); // ~19.5 dBm

  // NOTE: Using LR on AP can break phone compatibility, so we DO NOT force LR here.
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G);
}

// ================= HTTP =================
static void handleRoot() {
  int stations = WiFi.softAPgetStationNum();
  String html =
    "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>RX_CAM</title></head><body style='font-family:sans-serif;'>"
    "<h2>RX_CAM Latest</h2>"
    "<img id='img' src='/latest.jpg' style='max-width:100%;height:auto;border:1px solid #ccc;'/>"
    "<script>setInterval(()=>{document.getElementById('img').src='/latest.jpg?t='+Date.now();},400);</script>"
    "<hr/>"
    "<p><b>Latest ID:</b> " + String(g_latest_id) + "</p>"
    "<p><b>RX Mbps:</b> " + String(last_mbps, 2) + "</p>"
    "<p><b>SD last save:</b> " + String(last_save_ok ? "OK" : "FAIL") + "</p>"
    "<p><b>Last file:</b> " + last_save_path + "</p>"
    "<p><b>Stations:</b> " + String(stations) + "</p>"
    "</body></html>";
  server.send(200, "text/html", html);
}

static void handleLatest() {
  xSemaphoreTake(g_latest_mutex, portMAX_DELAY);
  uint8_t* buf = g_latest;
  size_t len = g_latest_len;
  xSemaphoreGive(g_latest_mutex);

  if (!buf || len == 0) {
    server.send(404, "text/plain", "No image yet");
    return;
  }
  server.setContentLength(len);
  server.send(200, "image/jpeg", "");
  server.client().write(buf, len);
}

// ================= Frame control =================
static void dropCurrentFrame(const char* why) {
  if (cur_buf) free(cur_buf);
  cur_buf = nullptr;
  cur_id = 0;
  cur_total = 0;
  cur_got = 0;
  cur_last_ms = 0;
  frame_start_ms = 0;
  if (why) Serial.printf("Drop frame: %s\n", why);
}

// ================= UDP receive (ROBUST: read whole datagram once) =================
static void handleUdpOnce() {
  if (cur_buf && (millis() - cur_last_ms > FRAME_TIMEOUT_MS)) {
    dropCurrentFrame("timeout");
  }

  int plen = udp.parsePacket();
  if (plen <= 0) return;

  static uint8_t pkt[1600];
  if (plen > (int)sizeof(pkt)) {
    while (udp.available()) udp.read();
    return;
  }

  int n = udp.read(pkt, plen);
  if (n != plen) return;

  uint8_t type = pkt[0];

  if (type == PKT_START) {
    if (plen < (int)sizeof(PktStart)) return;

    PktStart s;
    memcpy(&s, pkt, sizeof(s));

    if (cur_buf) dropCurrentFrame("new START arrived");

    if (s.total_len == 0 || s.total_len > MAX_JPEG_BYTES) {
      Serial.printf("Bad START total_len=%lu\n", (unsigned long)s.total_len);
      return;
    }

    cur_buf = (uint8_t*)ps_malloc(s.total_len);
    if (!cur_buf) { dropCurrentFrame("PSRAM alloc failed"); return; }

    cur_id = s.img_id;
    cur_total = s.total_len;
    cur_got = 0;
    cur_last_ms = millis();
    frame_start_ms = millis();
    return;
  }

  if (type == PKT_DATA) {
    if (!cur_buf) return;
    if (plen < (int)sizeof(PktDataHdr)) return;

    PktDataHdr h;
    memcpy(&h, pkt, sizeof(h));

    if (h.img_id != cur_id) return;
    if (h.len > UDP_PAYLOAD_MAX) { dropCurrentFrame("DATA len too big"); return; }
    if ((int)sizeof(PktDataHdr) + (int)h.len != plen) { dropCurrentFrame("payload mismatch"); return; }
    if (cur_got + h.len > cur_total) { dropCurrentFrame("overflow"); return; }

    memcpy(cur_buf + cur_got, pkt + sizeof(PktDataHdr), h.len);
    cur_got += h.len;
    cur_last_ms = millis();
    return;
  }

  if (type == PKT_END) {
    if (!cur_buf) return;
    if (plen < (int)sizeof(PktEnd)) return;

    PktEnd e;
    memcpy(&e, pkt, sizeof(e));

    if (e.img_id != cur_id || e.total_len != cur_total) { dropCurrentFrame("END mismatch"); return; }
    if (cur_got != cur_total) { dropCurrentFrame("incomplete"); return; }

    // compute Mbps
    uint32_t dt = millis() - frame_start_ms;
    if (dt == 0) dt = 1;
    last_mbps = (cur_total * 8.0f) / (dt * 1000.0f);

    // publish latest (take ownership of cur_buf)
    xSemaphoreTake(g_latest_mutex, portMAX_DELAY);
    if (g_latest) free(g_latest);
    g_latest = cur_buf;
    g_latest_len = cur_total;
    g_latest_id = cur_id;
    xSemaphoreGive(g_latest_mutex);

    // queue SD COPY so latest stays valid
    if (qSave) {
      uint8_t* copy = (uint8_t*)ps_malloc(cur_total);
      if (copy) {
        memcpy(copy, g_latest, cur_total);
        SaveItem si{g_latest_id, (uint32_t)cur_total, copy};
        if (xQueueSend(qSave, &si, 0) != pdTRUE) {
          free(copy);
          Serial.println("qSave full: dropped SD save");
        }
      } else {
        Serial.println("No PSRAM for SD copy (still serving latest)");
      }
    }

    Serial.printf("RX frame complete img=%lu bytes=%lu rx=%.2f Mbps\n",
                  (unsigned long)g_latest_id,
                  (unsigned long)g_latest_len,
                  last_mbps);

    // reset reassembly state (DO NOT free cur_buf; it is now g_latest)
    cur_buf = nullptr;
    cur_id = 0;
    cur_total = 0;
    cur_got = 0;
    cur_last_ms = 0;
    frame_start_ms = 0;
    return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  setCpuFrequencyMhz(240);
  Serial.printf("psramFound(): %s | size=%u\n",
                psramFound() ? "YES" : "NO", (unsigned)ESP.getPsramSize());

  g_latest_mutex = xSemaphoreCreateMutex();

  if (initSD()) {
    makeNewRunFolder();
    qSave = xQueueCreate(4, sizeof(SaveItem));
    xTaskCreatePinnedToCore(taskSD, "sd", 8192, nullptr, 1, nullptr, 1);
  } else {
    Serial.println("SD init failed (continuing without SD saving)");
  }

  wifiStartAP_LongRange();

  udp.begin(UDP_PORT);
  Serial.printf("UDP listening on %u\n", UDP_PORT);

  server.on("/", handleRoot);
  server.on("/latest.jpg", handleLatest);
  server.begin();
  Serial.println("HTTP server started (http://192.168.4.1/)");
}

void loop() {
  server.handleClient();
  handleUdpOnce();
}
