#include <esp32_can.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════
   LED
   ═══════════════════════════════════════════════════════════════ */
#define LED_PIN          16
#define LED_BLINK_COUNT  5
#define LED_BLINK_ON_MS  120
#define LED_BLINK_OFF_MS 120

/* ═══════════════════════════════════════════════════════════════
   CAN
   ═══════════════════════════════════════════════════════════════ */
#define CAN_TX_PIN           GPIO_NUM_5
#define CAN_RX_PIN           GPIO_NUM_4
#define CAN_SPEED            500000UL
#define CAN_OBD2_REQ_ID      0x7DF
#define CAN_OBD2_RESP_ID     0x7E8
#define CAN_READ_TIMEOUT_MS  200   /* ms to wait for a single CAN frame     */
#define CAN_INTER_PID_MS     25    /* ms gap between consecutive PID queries */
#define CAN_MAX_RETRIES      3     /* retry attempts per PID before "error"  */

/* ISO-TP frame type nibbles */
#define ISOTP_SINGLE  0x00
#define ISOTP_FIRST   0x10
#define ISOTP_CONSEC  0x20

/* OBD-II service modes */
#define OBD2_SVC_CURRENT  0x01
#define OBD2_SVC_DTC_READ 0x03
#define OBD2_SVC_DTC_CLR  0x04
#define OBD2_SVC_INFO     0x09

/* OBD-II PIDs — Mode 01 */
#define PID_ENGINE_LOAD   0x04
#define PID_COOLANT_TEMP  0x05
#define PID_STFT_B1       0x06
#define PID_LTFT_B1       0x07
#define PID_FUEL_PRESSURE 0x0A
#define PID_MAP           0x0B
#define PID_RPM           0x0C
#define PID_SPEED         0x0D
#define PID_IGN_ADVANCE   0x0E
#define PID_IAT           0x0F
#define PID_THROTTLE      0x11
#define PID_O2_VOLTAGE    0x14
#define PID_RUNTIME       0x1F
#define PID_BAT_VOLTAGE   0x42
#define PID_AMBIENT_TEMP  0x46
#define PID_OIL_TEMP      0x5C

/* OBD-II PIDs — Mode 09 */
#define PID_VIN      0x02
#define PID_ECU_NAME 0x0A

/* ═══════════════════════════════════════════════════════════════
   Wi-Fi / SERVER
   ═══════════════════════════════════════════════════════════════ */
#define WIFI_AP_SSID     "OBD2-Bike"
#define WIFI_AP_PASS     "12345678"
#define WIFI_AP_IP       "192.168.4.1"
#define CAPTIVE_HOSTNAME "obd.local"
#define DNS_PORT         53
#define HTTP_PORT        80
#define DTC_JSON_PATH    "/dtc_codes.json"
#define RECORD_CSV_PATH  "/record_data.csv"

/* ═══════════════════════════════════════════════════════════════
   EEPROM
   ═══════════════════════════════════════════════════════════════ */
#define EEPROM_SIZE       8
#define EEPROM_ADDR_SUBS  0
#define EEPROM_MAGIC_ADDR 4
#define EEPROM_MAGIC_VAL  0xA5
#define DEFAULT_SUB_MASK  ( (1UL<<0)|(1UL<<4)|(1UL<<6)|(1UL<<10)|(1UL<<9) )

/* ═══════════════════════════════════════════════════════════════
   PARAMETER TABLE
   ═══════════════════════════════════════════════════════════════ */
typedef enum {
  PARAM_RPM         =  0,
  PARAM_SPEED       =  1,
  PARAM_COOLANT     =  2,
  PARAM_IAT         =  3,
  PARAM_THROTTLE    =  4,
  PARAM_RUNTIME     =  5,
  PARAM_BATT_VOLT   =  6,
  PARAM_AMBIENT     =  7,
  PARAM_ENG_LOAD    =  8,
  PARAM_MAP         =  9,
  PARAM_O2_VOLT     = 10,
  PARAM_FUEL_PRES   = 11,
  PARAM_STFT_B1     = 12,
  PARAM_LTFT_B1     = 13,
  PARAM_IGN_ADVANCE = 14,
  PARAM_OIL_TEMP    = 15,
  PARAM_COUNT       = 16
} param_index_t;

struct ParamMeta { const char *key, *label, *unit, *fmt; };

static const ParamMeta PARAM_META[PARAM_COUNT] = {
  {"rpm",      "Engine RPM",        "RPM",             "%.0f"},
  {"speed",    "Vehicle Speed",     "km/h",            "%.0f"},
  {"coolant",  "Coolant Temp",      "\xc2\xb0""C",     "%.1f"},
  {"iat",      "Intake Air Temp",   "\xc2\xb0""C",     "%.1f"},
  {"throttle", "Throttle Position", "%",               "%.1f"},
  {"runtime",  "Engine Runtime",    "s",               "%.0f"},
  {"batt",     "Battery Voltage",   "V",               "%.2f"},
  {"ambient",  "Ambient Temp",      "\xc2\xb0""C",     "%.1f"},
  {"engload",  "Engine Load",       "%",               "%.1f"},
  {"map",      "MAP Pressure",      "kPa",             "%.0f"},
  {"o2volt",   "O2 Sensor Voltage", "V",               "%.3f"},
  {"fuelpres", "Fuel Pressure",     "kPa",             "%.0f"},
  {"stft",     "Short-term FT B1",  "%",               "%.1f"},
  {"ltft",     "Long-term FT B1",   "%",               "%.1f"},
  {"ignadv",   "Ignition Advance",  "\xc2\xb0""BTDC",  "%.1f"},
  {"oiltemp",  "Oil Temperature",   "\xc2\xb0""C",     "%.1f"},
};

/* ═══════════════════════════════════════════════════════════════
   TYPES
   ═══════════════════════════════════════════════════════════════ */
typedef enum { STATUS_ERROR=-1, STATUS_OK=0, STATUS_TIMEOUT=2 } obd2_status_t;

typedef struct {
  float   value;
  uint8_t raw[20];
  uint8_t raw_len;
} obd2_result_t;

#define OBD2_MAX_DTCS 10
typedef struct {
  char    codes[OBD2_MAX_DTCS][6];
  uint8_t count;
} obd2_dtc_result_t;

/* ═══════════════════════════════════════════════════════════════
   MUTEXES
   ───────────────────────────────────────────────────────────────
   g_mutex     — protects val_cache, sub_mask, recording state.
                 Never held at the same time as g_can_mutex.

   g_can_mutex — serialises the CAN bus.  Every send+receive
                 transaction holds this mutex for its full
                 duration (send → recv → unlock).  The stale-
                 frame drain also runs under this mutex.
   ═══════════════════════════════════════════════════════════════ */
static SemaphoreHandle_t g_mutex     = nullptr;
static SemaphoreHandle_t g_can_mutex = nullptr;

#define LOCK()      xSemaphoreTake(g_mutex,     portMAX_DELAY)
#define UNLOCK()    xSemaphoreGive(g_mutex)
#define CAN_LOCK()  xSemaphoreTake(g_can_mutex, portMAX_DELAY)
#define CAN_UNLOCK() xSemaphoreGive(g_can_mutex)

/* ═══════════════════════════════════════════════════════════════
   GLOBALS
   ═══════════════════════════════════════════════════════════════ */
WebServer g_http(HTTP_PORT);
DNSServer g_dns;

volatile uint32_t g_sub_mask        = DEFAULT_SUB_MASK;
bool              g_can_ok          = false;

bool              g_record_enabled  = false;
bool              g_recording       = false;
bool              g_record_paused   = false;
uint32_t          g_record_mask     = 0;
unsigned long     g_record_start_ms = 0;
unsigned long     g_record_rows     = 0;
volatile size_t   g_csv_size        = 0;
volatile size_t   g_spiffs_free     = 0;
size_t            g_dtc_size        = 0;

static char g_val_cache[PARAM_COUNT][16];

static TaskHandle_t g_obd_task_handle  = nullptr;
static TaskHandle_t g_http_task_handle = nullptr;

/* ═══════════════════════════════════════════════════════════════
   FORWARD DECLARATIONS
   ═══════════════════════════════════════════════════════════════ */
void can_init(void);
static bool           can_send_raw(uint32_t id, const uint8_t *data, uint8_t len);
static obd2_status_t  can_recv_raw(uint32_t exp_id, CAN_FRAME *out, uint32_t timeout_ms);
static void           can_drain_rx(void);   /* drain stale frames — call under CAN_LOCK */

void eeprom_init(void);
void eeprom_save(uint32_t mask);
void flash_fs_init(void);
bool dtc_lookup_meaning(const char *code, char *meaning, size_t meaning_len);

void wifi_ap_init(void);
void dns_init(void);
void http_init(void);

void http_handle_root(void);
void http_handle_data(void);
void http_handle_settings_get(void);
void http_handle_settings_post(void);
void http_handle_dtc_read(void);
void http_handle_dtc_clear(void);
void http_handle_vin(void);
void http_handle_ecu_name(void);
void http_handle_raw_cmd(void);
void http_handle_record_status(void);
void http_handle_record_enable(void);
void http_handle_record_start(void);
void http_handle_record_pause(void);
void http_handle_record_stop(void);
void http_handle_record_download(void);
void http_handle_not_found(void);

/* OBD getters — each call acquires g_can_mutex internally via pid_request */
obd2_status_t obd2_get_rpm(obd2_result_t *out);
obd2_status_t obd2_get_speed(obd2_result_t *out);
obd2_status_t obd2_get_coolant(obd2_result_t *out);
obd2_status_t obd2_get_iat(obd2_result_t *out);
obd2_status_t obd2_get_throttle(obd2_result_t *out);
obd2_status_t obd2_get_runtime(obd2_result_t *out);
obd2_status_t obd2_get_batt_voltage(obd2_result_t *out);
obd2_status_t obd2_get_ambient(obd2_result_t *out);
obd2_status_t obd2_get_engine_load(obd2_result_t *out);
obd2_status_t obd2_get_map(obd2_result_t *out);
obd2_status_t obd2_get_o2_voltage(obd2_result_t *out);
obd2_status_t obd2_get_fuel_pressure(obd2_result_t *out);
obd2_status_t obd2_get_stft_b1(obd2_result_t *out);
obd2_status_t obd2_get_ltft_b1(obd2_result_t *out);
obd2_status_t obd2_get_ign_advance(obd2_result_t *out);
obd2_status_t obd2_get_oil_temp(obd2_result_t *out);
obd2_status_t obd2_read_dtcs(obd2_dtc_result_t *out);
obd2_status_t obd2_clear_dtcs(void);
obd2_status_t obd2_get_vin(char *buf, uint8_t buf_len);
obd2_status_t obd2_get_ecu_name(char *buf, uint8_t buf_len);
obd2_status_t obd2_raw_command(const char *hex_cmd,
                               char *raw_resp, uint8_t raw_len,
                               char *decoded,  uint8_t dec_len);

static obd2_status_t pid_request(uint8_t mode, uint8_t pid, obd2_result_t *out);
static obd2_status_t multiframe_request(uint8_t mode, uint8_t pid,
                                        uint8_t *out_buf, uint8_t buf_len,
                                        uint8_t *out_len);
static const char   *dtc_prefix(uint8_t top2);
static void          poll_active_params(void);
static void          record_start(void);
static void          record_stop(void);
static void          record_write_header(void);
static void          record_append_row(void);
static void          led_startup_blink(void);

/* ═══════════════════════════════════════════════════════════════
   LED
   ═══════════════════════════════════════════════════════════════ */
static void led_startup_blink(void) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  for (int i = 0; i < LED_BLINK_COUNT; i++) {
    digitalWrite(LED_PIN, HIGH); delay(LED_BLINK_ON_MS);
    digitalWrite(LED_PIN, LOW);  delay(LED_BLINK_OFF_MS);
  }
  Serial.println("[LED] Startup blink complete — LED off");
}

/* ═══════════════════════════════════════════════════════════════
   FREERTOS TASKS
   ═══════════════════════════════════════════════════════════════ */
static void obd_task(void *) {
  for (;;) {
    poll_active_params();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
static void http_task(void *) {
  for (;;) {
    g_dns.processNextRequest();
    g_http.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ═══════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════ */
void setup() {
  Serial.begin(115200);
  delay(100);
  led_startup_blink();

  Serial.println("\n============================================");
  Serial.println("  ESP32  OBD-II  Web Server  (v2 fixed)");
  Serial.println("============================================");

  g_mutex     = xSemaphoreCreateMutex();
  g_can_mutex = xSemaphoreCreateMutex();

  for (int i = 0; i < PARAM_COUNT; i++)
    strncpy(g_val_cache[i], "--", sizeof(g_val_cache[i]));

  eeprom_init();
  can_init();
  flash_fs_init();

  if (SPIFFS.exists(RECORD_CSV_PATH)) {
    SPIFFS.remove(RECORD_CSV_PATH);
    Serial.println("[BOOT] Previous record_data.csv deleted");
  }
  g_record_enabled = g_recording = g_record_paused = false;
  g_record_rows = g_csv_size = 0;

  wifi_ap_init();
  dns_init();
  http_init();

  g_spiffs_free = SPIFFS.totalBytes() - SPIFFS.usedBytes();

  xTaskCreatePinnedToCore(obd_task,  "obd_task",  4096, nullptr, 2, &g_obd_task_handle,  1);
  xTaskCreatePinnedToCore(http_task, "http_task", 6144, nullptr, 1, &g_http_task_handle, 0);

  Serial.println("System ready — connect to Wi-Fi: " WIFI_AP_SSID);
  Serial.println("Browse to: http://" CAPTIVE_HOSTNAME " or http://" WIFI_AP_IP);
  Serial.printf("[MEM] Free heap: %u bytes\n", (unsigned)esp_get_free_heap_size());
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }

/* ═══════════════════════════════════════════════════════════════
   POLL ACTIVE PARAMS  (OBD task, Core 1)

   Key changes vs previous version:
   • CAN_INTER_PID_MS delay between each PID so ECU can recover.
   • Each getter already retries internally (pid_request).
   • g_mutex and g_can_mutex are never held at the same time.
   ═══════════════════════════════════════════════════════════════ */
static void poll_active_params(void) {
  LOCK();
  uint32_t active = g_sub_mask;
  UNLOCK();
  if (active == 0) return;

  obd2_result_t res;

  /*
   * POLL macro:
   *   - Calls getter (internally acquires g_can_mutex for CAN I/O,
   *     releases before returning).
   *   - Acquires g_mutex only to write result to cache.
   *   - Waits CAN_INTER_PID_MS before the next PID.
   */
#define POLL(PARAM, FMT, GETTER)                              \
  if (active & (1UL << (PARAM))) {                            \
    obd2_status_t _st = (GETTER)(&res);                       \
    LOCK();                                                   \
    if (_st == STATUS_OK)                                     \
      snprintf(g_val_cache[(PARAM)], 16, (FMT), res.value);  \
    else                                                      \
      strncpy(g_val_cache[(PARAM)], "error", 16);             \
    UNLOCK();                                                 \
    Serial.printf("[OBD] %-18s : %s %s\n",                   \
                  PARAM_META[(PARAM)].label,                  \
                  g_val_cache[(PARAM)],                       \
                  PARAM_META[(PARAM)].unit);                  \
    vTaskDelay(pdMS_TO_TICKS(CAN_INTER_PID_MS));              \
  }

  POLL(PARAM_RPM,         "%.0f",  obd2_get_rpm)
  POLL(PARAM_SPEED,       "%.0f",  obd2_get_speed)
  POLL(PARAM_COOLANT,     "%.1f",  obd2_get_coolant)
  POLL(PARAM_IAT,         "%.1f",  obd2_get_iat)
  POLL(PARAM_THROTTLE,    "%.1f",  obd2_get_throttle)
  POLL(PARAM_RUNTIME,     "%.0f",  obd2_get_runtime)
  POLL(PARAM_BATT_VOLT,   "%.2f",  obd2_get_batt_voltage)
  POLL(PARAM_AMBIENT,     "%.1f",  obd2_get_ambient)
  POLL(PARAM_ENG_LOAD,    "%.1f",  obd2_get_engine_load)
  POLL(PARAM_MAP,         "%.0f",  obd2_get_map)
  POLL(PARAM_O2_VOLT,     "%.3f",  obd2_get_o2_voltage)
  POLL(PARAM_FUEL_PRES,   "%.0f",  obd2_get_fuel_pressure)
  POLL(PARAM_STFT_B1,     "%.1f",  obd2_get_stft_b1)
  POLL(PARAM_LTFT_B1,     "%.1f",  obd2_get_ltft_b1)
  POLL(PARAM_IGN_ADVANCE, "%.1f",  obd2_get_ign_advance)
  POLL(PARAM_OIL_TEMP,    "%.1f",  obd2_get_oil_temp)

#undef POLL

  size_t free_bytes = SPIFFS.totalBytes() - SPIFFS.usedBytes();
  LOCK();
  g_spiffs_free = free_bytes;
  bool do_rec   = g_recording && !g_record_paused;
  if (do_rec) g_record_mask = active;
  UNLOCK();

  if (do_rec) record_append_row();

  Serial.printf("[OBD] Poll done. Free SPIFFS: %.2f MB\n",
                (float)free_bytes / 1048576.0f);
}

/* ═══════════════════════════════════════════════════════════════
   CAN LAYER
   ═══════════════════════════════════════════════════════════════ */
void can_init(void) {
  CAN0.setCANPins(CAN_RX_PIN, CAN_TX_PIN);
  if (CAN0.begin(CAN_SPEED)) {
    CAN0.watchFor(CAN_OBD2_RESP_ID);
    g_can_ok = true;
    Serial.println("[CAN] Initialised at 500 kbps");
  } else {
    g_can_ok = false;
    Serial.println("[CAN] ERROR — failed to initialise");
  }
}

/* Internal — caller MUST hold g_can_mutex */
static bool can_send_raw(uint32_t id, const uint8_t *data, uint8_t len) {
  if (!g_can_ok) return false;
  CAN_FRAME tx;
  tx.id       = id;
  tx.extended = false;
  tx.rtr      = 0;
  tx.length   = (len > 8) ? 8 : len;
  for (uint8_t i = 0; i < tx.length; i++) tx.data.byte[i] = data[i];
  return CAN0.sendFrame(tx);
}

/* Internal — caller MUST hold g_can_mutex.
   Polls every 1 ms using vTaskDelay so the scheduler keeps
   running while waiting.  The mutex is NOT released during the
   wait — that is intentional: no other task may touch the bus
   while we are expecting a response frame. */
static obd2_status_t can_recv_raw(uint32_t exp_id, CAN_FRAME *out,
                                   uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    if (CAN0.read(*out))
      if ((exp_id == 0) || (out->id == exp_id))
        return STATUS_OK;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return STATUS_TIMEOUT;
}

/*
 * can_drain_rx — read and discard all frames currently in the RX
 * FIFO.  Called at the start of every new PID request (while
 * holding g_can_mutex) so a late-arriving frame from the previous
 * request cannot be mistaken for the new response.
 *
 * Uses a short fixed window (5 ms) rather than looping on
 * availability, to avoid hanging if the bus is busy.
 */
static void can_drain_rx(void) {
  CAN_FRAME dummy;
  uint32_t drain_until = millis() + 5;
  while (millis() < drain_until && CAN0.read(dummy)) { /* discard */ }
}

/* ═══════════════════════════════════════════════════════════════
   EEPROM
   ═══════════════════════════════════════════════════════════════ */
void eeprom_init(void) {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) {
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
    EEPROM.put(EEPROM_ADDR_SUBS, (uint32_t)DEFAULT_SUB_MASK);
    EEPROM.commit();
    g_sub_mask = DEFAULT_SUB_MASK;
    Serial.printf("[EEPROM] First boot — default mask: 0x%08X\n", (unsigned)DEFAULT_SUB_MASK);
  } else {
    uint32_t saved = 0;
    EEPROM.get(EEPROM_ADDR_SUBS, saved);
    g_sub_mask = saved;
    Serial.printf("[EEPROM] Restored mask: 0x%08X\n", (unsigned)saved);
  }
}

void eeprom_save(uint32_t mask) {
  uint32_t current = 0;
  EEPROM.get(EEPROM_ADDR_SUBS, current);
  if (current != mask) {
    EEPROM.put(EEPROM_ADDR_SUBS, mask);
    EEPROM.commit();
    Serial.printf("[EEPROM] Saved mask: 0x%08X\n", (unsigned)mask);
  }
}

/* ═══════════════════════════════════════════════════════════════
   SPIFFS / DTC DATABASE
   ═══════════════════════════════════════════════════════════════ */
void flash_fs_init(void) {
  if (!SPIFFS.begin(true)) {
    Serial.println("[SPIFFS] ERROR - failed to mount"); return;
  }
  if (SPIFFS.exists(DTC_JSON_PATH)) {
    File f = SPIFFS.open(DTC_JSON_PATH, "r");
    if (!f) {
      Serial.println("[SPIFFS] ERROR - failed to open DTC JSON"); return;
    }
    else {
      g_dtc_size = f.size();
      Serial.printf("[SPIFFS] DTC database: %u bytes\n", (unsigned)g_dtc_size);
    }
    f.close();
  } else {
    Serial.printf("[SPIFFS] WARNING - missing %s\n", DTC_JSON_PATH);
  }
  Serial.printf("[SPIFFS] Total: %.2f MB  Free: %.2f MB\n",
                (float)SPIFFS.totalBytes()                        / 1048576.0f,
                (float)(SPIFFS.totalBytes()-SPIFFS.usedBytes())   / 1048576.0f);
}

/* Streaming DTC lookup — never loads full JSON into RAM */
bool dtc_lookup_meaning(const char *code, char *meaning, size_t meaning_len) {
  if (!code || !meaning || meaning_len == 0) return false;
  meaning[0] = '\0';
  File f = SPIFFS.open(DTC_JSON_PATH, "r");
  if (!f) return false;

  char key_pattern[10];
  snprintf(key_pattern, sizeof(key_pattern), "\"%s\"", code);
  const size_t key_len = strlen(key_pattern);
  size_t matched = 0; bool found_key = false;

  while (f.available()) {
    char c = (char)f.read();
    if (c == key_pattern[matched]) {
      if (++matched == key_len) {
        while (f.available()) {
          c = (char)f.read();
          if (c==' '||c=='\t'||c=='\r'||c=='\n') continue;
          if (c==':') found_key = true;
          break;
        }
        if (found_key) break;
        matched = 0;
      }
    } else { matched = (c==key_pattern[0]) ? 1 : 0; }
  }
  if (!found_key) { f.close(); return false; }

  bool found_quote = false;
  while (f.available()) {
    char c = (char)f.read();
    if (c==' '||c=='\t'||c=='\r'||c=='\n') continue;
    if (c=='"') found_quote = true;
    break;
  }
  if (!found_quote) { f.close(); return false; }

  size_t out_idx=0; bool escape=false;
  while (f.available()) {
    char c = (char)f.read();
    if (escape) {
      char d=c;
      switch(c){case '"':d='"';break;case '\\':d='\\';break;
                case 'n':d='\n';break;case 't':d='\t';break;default:break;}
      if (out_idx<meaning_len-1) meaning[out_idx++]=d;
      escape=false; continue;
    }
    if (c=='\\'){escape=true;continue;}
    if (c=='"') break;
    if (out_idx<meaning_len-1) meaning[out_idx++]=c;
  }
  meaning[out_idx]='\0';
  f.close();
  return (out_idx>0);
}

/* ═══════════════════════════════════════════════════════════════
   CSV RECORDING
   ═══════════════════════════════════════════════════════════════ */
static void record_write_header(void) {
  File f = SPIFFS.open(RECORD_CSV_PATH, "w");
  if (!f) { Serial.println("[REC] ERROR - could not create CSV"); return; }
  f.print("time_ms,elapsed_ms,subscribed_mask");
  for (int i=0;i<PARAM_COUNT;i++){f.print(',');f.print(PARAM_META[i].key);}
  f.println();
  size_t sz=f.size(); f.close();
  LOCK(); g_csv_size=sz; UNLOCK();
}

static void record_start(void) {
  if (SPIFFS.exists(RECORD_CSV_PATH)) {
    SPIFFS.remove(RECORD_CSV_PATH);
    Serial.printf("[REC] Deleted previous file\n");
  }
  LOCK();
  g_record_start_ms=millis(); g_record_mask=g_sub_mask;
  g_record_rows=0; g_record_paused=false; g_csv_size=0;
  UNLOCK();
  record_write_header();
  if (SPIFFS.exists(RECORD_CSV_PATH)) {
    LOCK(); g_recording=true; g_record_enabled=true; UNLOCK();
    Serial.println("[REC] Recording started");
  } else {
    Serial.println("[REC] ERROR - CSV not created");
  }
}

static void record_stop(void) {
  LOCK(); g_recording=false; g_record_paused=false; UNLOCK();
  Serial.printf("[REC] Recording stopped — %lu rows\n", g_record_rows);
}

static void record_append_row(void) {
  LOCK();
  bool still_rec      = g_recording && !g_record_paused;
  unsigned long now   = millis();
  unsigned long elaps = now - g_record_start_ms;
  uint32_t mask       = g_sub_mask;
  char snap[PARAM_COUNT][16];
  for (int i=0;i<PARAM_COUNT;i++) strncpy(snap[i],g_val_cache[i],16);
  UNLOCK();
  if (!still_rec) return;

  char row[512]; int pos=0;
  pos += snprintf(row+pos,sizeof(row)-pos,"%lu,%lu,%lu",now,elaps,(unsigned long)mask);
  for (int i=0;i<PARAM_COUNT&&pos<(int)sizeof(row)-20;i++) {
    row[pos++]=',';
    if ((mask&(1UL<<i)) && strcmp(snap[i],"--")!=0) {
      const char *s=snap[i]; bool q=false;
      for (const char *p=s;*p;p++) if(*p==','||*p=='"'||*p=='\n'){q=true;break;}
      if (q) {
        row[pos++]='"';
        for (const char *p=s;*p&&pos<(int)sizeof(row)-5;p++) {
          if(*p=='"') row[pos++]='"'; row[pos++]=*p;
        }
        row[pos++]='"';
      } else {
        int len=strlen(s);
        if(pos+len<(int)sizeof(row)-4){memcpy(row+pos,s,len);pos+=len;}
      }
    }
  }
  if(pos<(int)sizeof(row)-2){row[pos++]='\r';row[pos++]='\n';}
  row[pos]='\0';

  File f=SPIFFS.open(RECORD_CSV_PATH,"a");
  if(!f){Serial.println("[REC] ERROR - append failed");LOCK();g_recording=false;UNLOCK();return;}
  f.write((const uint8_t*)row,pos);
  size_t new_sz=f.size(); f.close();
  LOCK(); g_record_rows++; g_csv_size=new_sz; UNLOCK();
}

/* ═══════════════════════════════════════════════════════════════
   Wi-Fi
   ═══════════════════════════════════════════════════════════════ */
void wifi_ap_init(void) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  delay(200);
  IPAddress apIP; apIP.fromString(WIFI_AP_IP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
  Serial.printf("[WiFi] AP \"%s\" — IP: %s\n", WIFI_AP_SSID,
                WiFi.softAPIP().toString().c_str());

  WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t i) {
    Serial.printf("[WiFi] Connected    MAC: %02X:%02X:%02X:%02X:%02X:%02X  clients:%d\n",
      i.wifi_ap_staconnected.mac[0],i.wifi_ap_staconnected.mac[1],
      i.wifi_ap_staconnected.mac[2],i.wifi_ap_staconnected.mac[3],
      i.wifi_ap_staconnected.mac[4],i.wifi_ap_staconnected.mac[5],
      WiFi.softAPgetStationNum());
  }, ARDUINO_EVENT_WIFI_AP_STACONNECTED);

  WiFi.onEvent([](WiFiEvent_t e, WiFiEventInfo_t i) {
    Serial.printf("[WiFi] Disconnected MAC: %02X:%02X:%02X:%02X:%02X:%02X  clients:%d\n",
      i.wifi_ap_stadisconnected.mac[0],i.wifi_ap_stadisconnected.mac[1],
      i.wifi_ap_stadisconnected.mac[2],i.wifi_ap_stadisconnected.mac[3],
      i.wifi_ap_stadisconnected.mac[4],i.wifi_ap_stadisconnected.mac[5],
      WiFi.softAPgetStationNum());
  }, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
}

/* ═══════════════════════════════════════════════════════════════
   DNS
   ═══════════════════════════════════════════════════════════════ */
void dns_init(void) {
  IPAddress apIP; apIP.fromString(WIFI_AP_IP);
  g_dns.start(DNS_PORT, "*", apIP);
  Serial.println("[DNS] Captive portal started");
}

/* ═══════════════════════════════════════════════════════════════
   HTTP ROUTES
   ═══════════════════════════════════════════════════════════════ */
void http_init(void) {
  g_http.on("/",                    HTTP_GET,  http_handle_root);
  g_http.on("/index.html",          HTTP_GET,  http_handle_root);
  g_http.on("/generate_204",        HTTP_GET,  http_handle_root);
  g_http.on("/connecttest.txt",     HTTP_GET,  http_handle_root);
  g_http.on("/hotspot-detect.html", HTTP_GET,  http_handle_root);
  g_http.on("/ncsi.txt",            HTTP_GET,  http_handle_root);
  g_http.on("/fwlink",              HTTP_GET,  http_handle_root);
  g_http.on("/api/data",            HTTP_GET,  http_handle_data);
  g_http.on("/api/settings",        HTTP_GET,  http_handle_settings_get);
  g_http.on("/api/settings",        HTTP_POST, http_handle_settings_post);
  g_http.on("/api/dtc/read",        HTTP_GET,  http_handle_dtc_read);
  g_http.on("/api/dtc/clear",       HTTP_GET,  http_handle_dtc_clear);
  g_http.on("/api/vin",             HTTP_GET,  http_handle_vin);
  g_http.on("/api/ecu",             HTTP_GET,  http_handle_ecu_name);
  g_http.on("/api/raw",             HTTP_POST, http_handle_raw_cmd);
  g_http.on("/api/record/status",   HTTP_GET,  http_handle_record_status);
  g_http.on("/api/record/enable",   HTTP_POST, http_handle_record_enable);
  g_http.on("/api/record/start",    HTTP_GET,  http_handle_record_start);
  g_http.on("/api/record/pause",    HTTP_GET,  http_handle_record_pause);
  g_http.on("/api/record/stop",     HTTP_GET,  http_handle_record_stop);
  g_http.on("/api/record/download", HTTP_GET,  http_handle_record_download);
  g_http.onNotFound(http_handle_not_found);
  g_http.begin();
  Serial.println("[HTTP] Server started on port 80");
}

static void begin_response(int, const char *) {
  g_http.sendHeader("Access-Control-Allow-Origin","*");
  g_http.sendHeader("Cache-Control","no-cache");
}
static void redirect_home(void) {
  g_http.sendHeader("Location","http://" WIFI_AP_IP);
  g_http.send(302,"text/plain","");
}

/* ═══════════════════════════════════════════════════════════════
   HTML PAGE (PROGMEM)
   ═══════════════════════════════════════════════════════════════ */
static const char HTML_PAGE[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"/>
<title>ESP32 OBD2 Scanner</title>
<style>
:root{--bg:#0a0c0f;--surface:#111418;--card:#181d24;--border:#252c38;--accent:#00e5ff;--accent2:#ff6b35;--text:#e8edf5;--muted:#7d879a;--danger:#ff3b3b;--ok:#00e5a0;--warn:#ffd166;--font:"Courier New","Lucida Console",monospace}*{box-sizing:border-box;margin:0;padding:0}html,body{background:var(--bg);color:var(--text);font-family:var(--font);min-height:100vh}body{max-width:920px;margin:0 auto;border-left:1px solid #141922;border-right:1px solid #141922}nav{position:sticky;top:0;z-index:100;display:flex;align-items:center;justify-content:space-between;padding:0 16px;height:56px;background:rgba(10,12,15,.94);border-bottom:1px solid var(--border);backdrop-filter:blur(8px)}.nav-brand{display:flex;align-items:center;gap:10px;font-size:1rem;font-weight:700;letter-spacing:.08em;color:var(--accent)}.nav-status{display:flex;align-items:center;gap:8px;font-size:.72rem;color:var(--muted)}.status-dot{width:8px;height:8px;border-radius:50%;background:var(--muted)}.status-dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok)}.status-dot.err{background:var(--danger);box-shadow:0 0 8px var(--danger)}.tabs{display:flex;border-bottom:1px solid var(--border);background:var(--surface);overflow-x:auto}.tab{flex:1;min-width:92px;text-align:center;padding:12px 6px;font-size:.72rem;letter-spacing:.06em;color:var(--muted);cursor:pointer;border-bottom:2px solid transparent;user-select:none;white-space:nowrap}.tab.active{color:var(--accent);border-bottom-color:var(--accent)}.panel{display:none;padding:14px 12px 80px}.panel.active{display:block}.gauge-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(145px,1fr));gap:10px}@media(max-width:420px){.gauge-grid{grid-template-columns:1fr 1fr}}.gauge-card,.tool-card,.record-card,.setting-item{background:var(--card);border:1px solid var(--border);border-radius:10px}.gauge-card{padding:14px 12px 12px;display:flex;flex-direction:column;gap:6px;position:relative;overflow:hidden}.gauge-card:before{content:"";position:absolute;inset:0 0 auto 0;height:2px;background:linear-gradient(90deg,var(--accent),transparent);opacity:.55}.gauge-label{font-size:.62rem;letter-spacing:.08em;color:var(--muted);text-transform:uppercase}.gauge-value{font-size:1.65rem;font-weight:700;color:var(--accent);line-height:1}.gauge-value.error{font-size:.95rem;color:var(--danger)}.gauge-unit{font-size:.65rem;color:var(--muted)}.tool-section{margin-bottom:18px}.tool-card,.record-card{padding:14px 12px;margin-bottom:12px}.tool-title{font-size:.68rem;letter-spacing:.1em;color:var(--muted);text-transform:uppercase;margin-bottom:10px}.tool-btn-row{display:flex;flex-wrap:wrap;gap:8px}button,.download-link{font-family:var(--font);border-radius:8px;cursor:pointer;transition:opacity .2s,border-color .2s,color .2s,transform .08s}button:active{transform:translateY(1px)}button:disabled{opacity:.45;cursor:not-allowed;transform:none}.tool-btn{flex:1;min-width:125px;padding:11px 10px;background:var(--card);border:1px solid var(--border);color:var(--text);font-size:.75rem;text-align:center}.tool-btn:hover:not(:disabled){border-color:var(--accent);color:var(--accent)}.tool-btn.danger:hover:not(:disabled){border-color:var(--danger);color:var(--danger)}.tool-btn.primary{background:var(--accent);color:#001014;border-color:var(--accent);font-weight:700}.tool-btn.primary:hover:not(:disabled){color:#fff}.result-box,textarea,input[type=text]{width:100%;padding:10px;background:#0d1017;border:1px solid var(--border);color:var(--ok);font-family:var(--font);font-size:.78rem;border-radius:8px;white-space:pre-wrap;word-break:break-word}textarea{min-height:74px;resize:vertical;margin-top:8px}input[type=text]{outline:none;color:var(--text);text-transform:uppercase}input[type=text]:focus{border-color:var(--accent)}.settings-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:8px}.setting-item{display:flex;align-items:center;gap:10px;padding:10px 12px;cursor:pointer}.setting-item.checked{border-color:var(--accent)}.setting-item input{display:none}.check-box{width:18px;height:18px;border:1.5px solid var(--border);border-radius:4px;flex-shrink:0;display:flex;align-items:center;justify-content:center}.setting-item.checked .check-box{background:var(--accent);border-color:var(--accent)}.check-box span{display:none;color:#001014;font-size:.7rem;font-weight:bold}.setting-item.checked .check-box span{display:block}.setting-text{font-size:.72rem;color:var(--text);line-height:1.3}.setting-text small{color:var(--muted);font-size:.6rem}.btn-save{margin-top:16px;width:100%;padding:12px;background:var(--accent);color:#001014;font-weight:700;font-size:.85rem;letter-spacing:.1em;border:none}.raw-row{display:flex;gap:8px;margin-bottom:8px}.raw-send{padding:10px 16px;border:none;background:var(--accent2);color:white;font-weight:700}.raw-labels{display:flex;gap:8px;margin-top:8px}.raw-label{font-size:.62rem;color:var(--muted);flex:1;margin-bottom:4px}.record-status{font-size:.72rem;color:var(--muted);line-height:1.8;margin:10px 0}.storage-bar-wrap{height:8px;background:#1a2030;border-radius:4px;margin:6px 0 2px;overflow:hidden}.storage-bar{height:100%;border-radius:4px;background:var(--ok);transition:width .5s,background .4s}.storage-bar.warn{background:var(--warn)}.storage-bar.danger{background:var(--danger)}.storage-label{font-size:.62rem;color:var(--muted)}.record-table-wrap{overflow:auto;max-height:300px;margin-top:12px;border:1px solid var(--border);border-radius:8px;background:#0d1017}table{width:100%;border-collapse:collapse;font-size:.68rem;min-width:700px}th,td{padding:8px 10px;border-bottom:1px solid #202633;text-align:left;white-space:nowrap}th{color:var(--accent);background:#0d1017;position:sticky;top:0;z-index:2}td{color:var(--text)}.download-link{display:none;margin-top:10px;padding:11px 12px;border:1px solid var(--accent);color:var(--accent);text-decoration:none;text-align:center;font-size:.75rem}.download-link.show{display:block}.toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%);background:var(--surface);border:1px solid var(--border);color:var(--text);padding:10px 20px;border-radius:20px;font-size:.78rem;opacity:0;pointer-events:none;transition:opacity .25s;z-index:999}.toast.show{opacity:1}.hint{color:var(--muted);font-size:.72rem;line-height:1.5;margin-top:10px}.hint code{color:var(--accent)}
</style>
</head>
<body>
<nav><div class="nav-brand">&#9881; OBD2 LIVE</div><div class="nav-status"><div class="status-dot" id="statusDot"></div><span id="statusText">connecting&hellip;</span></div></nav>
<div class="tabs"><div class="tab active" data-tab="dash">DASH</div><div class="tab" data-tab="tools">TOOLS</div><div class="tab" data-tab="raw">RAW CMD</div><div class="tab" data-tab="settings">SETTINGS</div><div class="tab" data-tab="record">RECORD DATA</div></div>
<div class="panel active" id="tab-dash"><div class="gauge-grid" id="gaugeGrid"></div><div class="hint">Live values update every second. ERR means that PID did not respond.</div></div>
<div class="panel" id="tab-tools"><div class="tool-section tool-card"><div class="tool-title">Vehicle Info</div><div class="tool-btn-row"><button class="tool-btn" onclick="toolAction('vin')">Read VIN</button><button class="tool-btn" onclick="toolAction('ecu')">Read ECU Name</button></div><textarea id="vehicleOut" readonly placeholder="Vehicle info result&hellip;"></textarea></div><div class="tool-section tool-card"><div class="tool-title">Diagnostic Trouble Codes</div><div class="tool-btn-row"><button class="tool-btn" onclick="toolAction('dtc/read')">Read DTCs</button><button class="tool-btn danger" onclick="toolAction('dtc/clear')">Clear DTCs</button></div><textarea id="dtcOut" readonly placeholder="DTC result&hellip;"></textarea></div></div>
<div class="panel" id="tab-raw"><div class="tool-card"><div class="tool-title">Custom OBD-II Hex Command</div><div class="raw-row"><input id="rawInput" type="text" maxlength="8" inputmode="text" autocomplete="off" spellcheck="false" placeholder="e.g. 010C"/><button class="raw-send" onclick="sendRaw()">SEND</button></div><div class="hint">Hex only 0-9 A-F, even number of characters. Example: <code>010C</code>, <code>010D</code>, <code>0902</code>.</div><div class="raw-labels"><div class="raw-label">RAW RESPONSE</div><div class="raw-label">DECODED VALUE</div></div><div style="display:flex;gap:8px;"><textarea id="rawOut" readonly placeholder="Raw hex bytes&hellip;"></textarea><textarea id="decOut" readonly placeholder="Decoded result&hellip;"></textarea></div></div></div>
<div class="panel" id="tab-settings"><div class="tool-card"><div class="tool-title">Select parameters to display and record</div><div class="settings-grid" id="settingsGrid"></div><button class="btn-save" onclick="saveSettings()">SAVE SETTINGS</button><div class="hint">Changing subscriptions while recording will not stop recording.</div></div></div>
<div class="panel" id="tab-record"><div class="record-card"><div class="tool-title">Record subscribed live data</div><div class="tool-btn-row"><button class="tool-btn primary" id="recordStartBtn" onclick="startRecording()">Start Recording</button><button class="tool-btn" id="recordPauseBtn" onclick="togglePauseRecording()">Pause</button><button class="tool-btn danger" id="recordStopBtn" onclick="stopRecording()">Stop Recording</button></div><div class="record-status" id="recordStatus">Press Start Recording to create a fresh CSV file.</div><div id="storageWrap" style="display:none"><div class="storage-bar-wrap"><div class="storage-bar" id="storageBar" style="width:0%"></div></div><div class="storage-label" id="storageLabel"></div></div><a class="download-link" id="recordDownload" href="/api/record/download" download="record_data.csv">&#11015; Download record_data.csv</a><div class="record-table-wrap"><table><thead id="recordHead"></thead><tbody id="recordBody"></tbody></table></div><div class="hint">Starting again replaces the previous <code>record_data.csv</code> on the ESP32.</div></div></div>
<div class="toast" id="toast"></div>
<script>
const PARAMS=[{id:0,key:'rpm',label:'Engine RPM',unit:'RPM'},{id:1,key:'speed',label:'Vehicle Speed',unit:'km/h'},{id:2,key:'coolant',label:'Coolant Temp',unit:'\u00b0C'},{id:3,key:'iat',label:'Intake Air Temp',unit:'\u00b0C'},{id:4,key:'throttle',label:'Throttle Position',unit:'%'},{id:5,key:'runtime',label:'Engine Runtime',unit:'s'},{id:6,key:'batt',label:'Battery Voltage',unit:'V'},{id:7,key:'ambient',label:'Ambient Temp',unit:'\u00b0C'},{id:8,key:'engload',label:'Engine Load',unit:'%'},{id:9,key:'map',label:'MAP Pressure',unit:'kPa'},{id:10,key:'o2volt',label:'O2 Sensor Voltage',unit:'V'},{id:11,key:'fuelpres',label:'Fuel Pressure',unit:'kPa'},{id:12,key:'stft',label:'Short-term FT B1',unit:'%'},{id:13,key:'ltft',label:'Long-term FT B1',unit:'%'},{id:14,key:'ignadv',label:'Ignition Advance',unit:'\u00b0BTDC'},{id:15,key:'oiltemp',label:'Oil Temperature',unit:'\u00b0C'}];
let currentMask=0,recordRows=[];
document.querySelectorAll('.tab').forEach(t=>t.addEventListener('click',()=>switchTab(t.dataset.tab)));
function switchTab(n){document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active',t.dataset.tab===n));document.querySelectorAll('.panel').forEach(p=>p.classList.toggle('active',p.id==='tab-'+n));if(n==='record')refreshRecordStatus();}
function buildGauges(mask){const g=document.getElementById('gaugeGrid');g.innerHTML='';PARAMS.forEach(p=>{if(!(mask&(1<<p.id)))return;const c=document.createElement('div');c.className='gauge-card';c.innerHTML=`<div class="gauge-label">${p.label}</div><div class="gauge-value" id="val-${p.key}">--</div><div class="gauge-unit">${p.unit}</div>`;g.appendChild(c);});}
function buildSettings(mask){const g=document.getElementById('settingsGrid');g.innerHTML='';PARAMS.forEach(p=>{const ck=!!(mask&(1<<p.id));const item=document.createElement('label');item.className='setting-item'+(ck?' checked':'');item.innerHTML=`<input type="checkbox" data-id="${p.id}" ${ck?'checked':''}><div class="check-box"><span>\u2713</span></div><div class="setting-text">${p.label}<br><small>${p.unit}</small></div>`;const cb=item.querySelector('input');cb.addEventListener('change',()=>item.classList.toggle('checked',cb.checked));g.appendChild(item);});}
async function loadSettings(){try{const r=await fetch('/api/settings');const d=await r.json();currentMask=d.mask;buildGauges(currentMask);buildSettings(currentMask);renderRecordTable();}catch(e){setStatus(false);}}
async function saveSettings(){let mask=0;document.querySelectorAll('#settingsGrid input[type="checkbox"]').forEach(cb=>{if(cb.checked)mask|=(1<<Number(cb.dataset.id));});try{const r=await fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({mask})});const d=await r.json();currentMask=d.mask;buildGauges(currentMask);renderRecordTable();showToast('Settings saved \u2713');}catch(e){showToast('Save failed');}}
async function pollData(){try{const r=await fetch('/api/data');if(!r.ok)throw new Error('HTTP '+r.status);const d=await r.json();if(d.mask!==currentMask){currentMask=d.mask;buildGauges(currentMask);buildSettings(currentMask);}updateGauges(d.values||{});const vals=Object.values(d.values||{});setStatus(vals.some(v=>v&&v!=='--'&&v!=='error'));}catch(e){setStatus(false);}}
function updateGauges(v){PARAMS.forEach(p=>{const el=document.getElementById('val-'+p.key);if(!el)return;const val=v[p.key];if(val===undefined||val==='--')return;if(val==='error'){el.textContent='ERR';el.className='gauge-value error';}else{el.textContent=val;el.className='gauge-value';}});}
function setStatus(ok){document.getElementById('statusDot').className='status-dot '+(ok?'ok':'err');document.getElementById('statusText').textContent=ok?'ECU connected':'No ECU response';}
async function toolAction(ep){const out=(ep==='vin'||ep==='ecu')?document.getElementById('vehicleOut'):document.getElementById('dtcOut');out.value='Loading\u2026';try{const r=await fetch('/api/'+ep);const d=await r.json();out.value=d.result||JSON.stringify(d);}catch(e){out.value='Error: '+e.message;}}
function sanitizeHexInput(el){el.value=el.value.toUpperCase().replace(/[^0-9A-F]/g,'');}
async function sendRaw(){const inp=document.getElementById('rawInput');sanitizeHexInput(inp);const cmd=inp.value.trim();if(!cmd){showToast('Enter an OBD hex command');return;}if(!/^[0-9A-F]+$/.test(cmd)||cmd.length%2!==0||cmd.length<4||cmd.length>8){showToast('Use hex only, even length, e.g. 010C');return;}document.getElementById('rawOut').value='Sending\u2026';document.getElementById('decOut').value='';try{const r=await fetch('/api/raw',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd})});const d=await r.json();document.getElementById('rawOut').value=d.raw||'no response';document.getElementById('decOut').value=d.decoded||'--';}catch(e){document.getElementById('rawOut').value='Error: '+e.message;}}
async function startRecording(){try{const r=await fetch('/api/record/start');const d=await r.json();recordRows=[];showToast(d.result||'Started');await refreshRecordStatus();}catch(e){showToast('Start failed');}}
async function togglePauseRecording(){try{const r=await fetch('/api/record/pause');const d=await r.json();showToast(d.result||'Toggled');await refreshRecordStatus();}catch(e){showToast('Pause failed');}}
async function stopRecording(){try{const r=await fetch('/api/record/stop');const d=await r.json();showToast(d.result||'Stopped');await refreshRecordStatus();}catch(e){showToast('Stop failed');}}
function updateStorageUI(d){const wrap=document.getElementById('storageWrap'),bar=document.getElementById('storageBar'),lbl=document.getElementById('storageLabel');if(!d.spiffs_total||!d.spiffs_total){wrap.style.display='none';return;}wrap.style.display='block';const pct=Math.min(100,Math.round((1-(d.spiffs_free/d.spiffs_total))*100));bar.style.width=pct+'%';bar.className='storage-bar'+(pct>90?' danger':pct>75?' warn':'');lbl.textContent=`Storage: ${(d.spiffs_free/1048576).toFixed(2)} MB free of ${(d.spiffs_total/1048576).toFixed(2)} MB`;}
async function refreshRecordStatus(){try{const r=await fetch('/api/record/status');const d=await r.json();const st=document.getElementById('recordStatus'),s=document.getElementById('recordStartBtn'),p=document.getElementById('recordPauseBtn'),x=document.getElementById('recordStopBtn'),lk=document.getElementById('recordDownload');s.disabled=!!d.recording;p.disabled=!d.recording;x.disabled=!d.recording&&!d.file_exists;p.textContent=d.paused?'Resume':'Pause';let msg='';if(d.recording&&d.paused)msg=`Paused \u23f8 \u2014 Rows:${d.rows||0} | ${(d.size/1024).toFixed(1)} KB`;else if(d.recording)msg=`Recording \u25cf \u2014 Rows:${d.rows||0} | ${(d.size/1024).toFixed(1)} KB`;else if(d.file_exists&&d.size>0)msg=`Stopped. ${d.rows||0} rows (${(d.size/1024).toFixed(1)} KB). Ready to download.`;else msg='Press Start Recording to create a fresh CSV file.';st.textContent=msg;updateStorageUI(d);lk.classList.toggle('show',!d.recording&&!!d.file_exists&&d.size>0);if(d.file_exists)loadRecordCsv();else{recordRows=[];renderRecordTable();}}catch(e){console.error(e);}}
async function loadRecordCsv(){try{const r=await fetch('/api/record/download?ts='+Date.now());if(!r.ok)return;const txt=await r.text();const lines=txt.trim().split(/\r?\n/);if(!lines.length)return;const cols=parseCsvLine(lines[0]);recordRows=lines.slice(1).filter(Boolean).map(l=>{const v=parseCsvLine(l);const row={};cols.forEach((c,i)=>row[c]=v[i]||'');return row;});renderRecordTable();}catch(e){}}
function parseCsvLine(line){const out=[];let cur='',q=false;for(let i=0;i<line.length;i++){const c=line[i];if(q){if(c==='"'&&line[i+1]==='"'){cur+='"';i++;}else if(c==='"')q=false;else cur+=c;}else if(c==='"')q=true;else if(c===','){out.push(cur);cur='';}else cur+=c;}out.push(cur);return out;}
function renderRecordTable(){const vc=['elapsed_ms',...PARAMS.filter(p=>currentMask&(1<<p.id)).map(p=>p.key)];document.getElementById('recordHead').innerHTML='<tr>'+vc.map(c=>`<th>${c}</th>`).join('')+'</tr>';document.getElementById('recordBody').innerHTML=recordRows.map(row=>'<tr>'+vc.map(c=>`<td>${row[c]??''}</td>`).join('')+'</tr>').join('');const w=document.querySelector('.record-table-wrap');if(w)w.scrollTop=w.scrollHeight;}
function showToast(msg){const t=document.getElementById('toast');t.textContent=msg;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2200);}
(function boot(){const raw=document.getElementById('rawInput');raw.addEventListener('input',()=>sanitizeHexInput(raw));loadSettings();pollData();refreshRecordStatus();setInterval(pollData,1000);setInterval(refreshRecordStatus,2500);})();
</script>
</body>
</html>
)HTMLEOF";

/* ═══════════════════════════════════════════════════════════════
   HTTP HANDLERS
   ═══════════════════════════════════════════════════════════════ */
void http_handle_root(void) {
  g_http.sendHeader("Cache-Control","no-cache");
  g_http.send_P(200,"text/html",HTML_PAGE);
}

void http_handle_data(void) {
  begin_response(200,"application/json");
  LOCK();
  uint32_t mask=g_sub_mask;
  char snap[PARAM_COUNT][16];
  for(int i=0;i<PARAM_COUNT;i++) strncpy(snap[i],g_val_cache[i],16);
  UNLOCK();
  StaticJsonDocument<512> doc;
  doc["mask"]=(unsigned long)mask;
  JsonObject vals=doc.createNestedObject("values");
  for(int i=0;i<PARAM_COUNT;i++) vals[PARAM_META[i].key]=snap[i];
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_settings_get(void) {
  begin_response(200,"application/json");
  LOCK(); uint32_t m=g_sub_mask; UNLOCK();
  StaticJsonDocument<64> doc; doc["mask"]=(unsigned long)m;
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_settings_post(void) {
  begin_response(200,"application/json");
  if(!g_http.hasArg("plain")){g_http.send(400,"application/json","{\"error\":\"no body\"}");return;}
  StaticJsonDocument<64> req;
  if(deserializeJson(req,g_http.arg("plain"))){g_http.send(400,"application/json","{\"error\":\"bad json\"}");return;}
  uint32_t nm=(uint32_t)(unsigned long)req["mask"];
  nm&=((1UL<<PARAM_COUNT)-1);
  LOCK(); g_sub_mask=nm;
  for(int i=0;i<PARAM_COUNT;i++) if(!(nm&(1UL<<i))) strncpy(g_val_cache[i],"--",16);
  UNLOCK();
  eeprom_save(nm);
  StaticJsonDocument<64> resp; resp["mask"]=(unsigned long)nm;
  String out; serializeJson(resp,out);
  g_http.send(200,"application/json",out);
}

void http_handle_dtc_read(void) {
  begin_response(200,"application/json");
  obd2_dtc_result_t dtc={};
  obd2_status_t st=obd2_read_dtcs(&dtc);
  StaticJsonDocument<6144> doc;
  if(st==STATUS_OK){
    if(dtc.count==0){doc["result"]="No fault codes found";}
    else{
      JsonArray arr=doc.createNestedArray("dtcs"); String rt;
      for(uint8_t i=0;i<dtc.count;i++){
        char m[180]={0}; bool f=dtc_lookup_meaning(dtc.codes[i],m,sizeof(m));
        JsonObject item=arr.createNestedObject();
        item["code"]=dtc.codes[i]; item["meaning"]=f?m:"Meaning not found";
        if(i) rt+="\n"; rt+=dtc.codes[i]; rt+=" - "; rt+=f?m:"Meaning not found";
      }
      doc["result"]=rt;
    }
  } else { doc["result"]="Error reading DTCs"; }
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_dtc_clear(void) {
  begin_response(200,"application/json");
  obd2_status_t st=obd2_clear_dtcs();
  StaticJsonDocument<64> doc;
  doc["result"]=(st==STATUS_OK)?"DTCs cleared successfully":"Error clearing DTCs";
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_vin(void) {
  begin_response(200,"application/json");
  char buf[64]={0}; obd2_status_t st=obd2_get_vin(buf,sizeof(buf));
  StaticJsonDocument<128> doc;
  doc["result"]=(st==STATUS_OK&&buf[0])?buf:"Error reading VIN";
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_ecu_name(void) {
  begin_response(200,"application/json");
  char buf[64]={0}; obd2_status_t st=obd2_get_ecu_name(buf,sizeof(buf));
  StaticJsonDocument<128> doc;
  doc["result"]=(st==STATUS_OK&&buf[0])?buf:"Error reading ECU name";
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_raw_cmd(void) {
  begin_response(200,"application/json");
  if(!g_http.hasArg("plain")){g_http.send(400,"application/json","{\"error\":\"no body\"}");return;}
  StaticJsonDocument<64> req;
  if(deserializeJson(req,g_http.arg("plain"))){g_http.send(400,"application/json","{\"error\":\"bad json\"}");return;}
  const char *cmd=req["cmd"]|"";
  char rb[64]={0},db[64]={0};
  obd2_raw_command(cmd,rb,sizeof(rb),db,sizeof(db));
  StaticJsonDocument<192> resp; resp["raw"]=rb; resp["decoded"]=db;
  String out; serializeJson(resp,out);
  g_http.send(200,"application/json",out);
}

void http_handle_record_status(void) {
  begin_response(200,"application/json");
  LOCK();
  bool rec=g_recording,paus=g_record_paused,en=g_record_enabled;
  unsigned long rows=g_record_rows; uint32_t mask=g_sub_mask;
  size_t sz=g_csv_size,fb=g_spiffs_free;
  UNLOCK();
  bool ex=SPIFFS.exists(RECORD_CSV_PATH);
  StaticJsonDocument<256> doc;
  doc["enabled"]=en; doc["recording"]=rec; doc["paused"]=paus;
  doc["rows"]=rows; doc["mask"]=(unsigned long)mask;
  doc["file_exists"]=ex; doc["path"]=RECORD_CSV_PATH;
  doc["size"]=(unsigned long)sz; doc["spiffs_free"]=(unsigned long)fb;
  doc["spiffs_total"]=(unsigned long)(SPIFFS.totalBytes()-g_dtc_size);
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_record_enable(void) {
  begin_response(200,"application/json");
  if(!g_http.hasArg("plain")){g_http.send(400,"application/json","{\"error\":\"no body\"}");return;}
  StaticJsonDocument<64> req;
  if(deserializeJson(req,g_http.arg("plain"))){g_http.send(400,"application/json","{\"error\":\"bad json\"}");return;}
  bool en=req["enabled"]|false;
  LOCK();
  g_record_enabled=en;
  if(!en&&g_recording){g_recording=false;g_record_paused=false;}
  if(!en) g_record_paused=false;
  bool rec=g_recording,paus=g_record_paused; unsigned long rows=g_record_rows; size_t sz=g_csv_size;
  UNLOCK();
  StaticJsonDocument<128> doc;
  doc["enabled"]=en; doc["recording"]=rec; doc["paused"]=paus;
  doc["rows"]=rows; doc["file_exists"]=SPIFFS.exists(RECORD_CSV_PATH); doc["size"]=(unsigned long)sz;
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_record_start(void) {
  begin_response(200,"application/json");
  record_start();
  LOCK(); bool rec=g_recording; unsigned long rows=g_record_rows; size_t sz=g_csv_size; UNLOCK();
  StaticJsonDocument<256> doc;
  doc["ok"]=rec; doc["result"]=rec?"Recording started":"Error starting recording";
  doc["enabled"]=g_record_enabled; doc["recording"]=rec; doc["paused"]=false;
  doc["rows"]=rows; doc["file_exists"]=SPIFFS.exists(RECORD_CSV_PATH); doc["size"]=(unsigned long)sz;
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_record_pause(void) {
  begin_response(200,"application/json");
  LOCK();
  if(g_recording) g_record_paused=!g_record_paused;
  bool rec=g_recording,paus=g_record_paused; unsigned long rows=g_record_rows; size_t sz=g_csv_size;
  UNLOCK();
  StaticJsonDocument<256> doc;
  doc["ok"]=rec; doc["result"]=rec?(paus?"Recording paused":"Recording resumed"):"Not recording";
  doc["enabled"]=g_record_enabled; doc["recording"]=rec; doc["paused"]=paus;
  doc["rows"]=rows; doc["file_exists"]=SPIFFS.exists(RECORD_CSV_PATH); doc["size"]=(unsigned long)sz;
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_record_stop(void) {
  begin_response(200,"application/json");
  record_stop();
  LOCK(); unsigned long rows=g_record_rows; size_t sz=g_csv_size; UNLOCK();
  bool ex=SPIFFS.exists(RECORD_CSV_PATH);
  StaticJsonDocument<256> doc;
  doc["ok"]=true; doc["result"]="Recording stopped";
  doc["enabled"]=g_record_enabled; doc["recording"]=false; doc["paused"]=false;
  doc["rows"]=rows; doc["file_exists"]=ex; doc["size"]=(unsigned long)sz;
  doc["download"]="/api/record/download";
  String out; serializeJson(doc,out);
  g_http.send(200,"application/json",out);
}

void http_handle_record_download(void) {
  if(!SPIFFS.exists(RECORD_CSV_PATH)){g_http.send(404,"text/plain","No recording file");return;}
  File f=SPIFFS.open(RECORD_CSV_PATH,"r");
  if(!f){g_http.send(500,"text/plain","Failed to open file");return;}
  g_http.sendHeader("Access-Control-Allow-Origin","*");
  g_http.sendHeader("Cache-Control","no-cache");
  g_http.sendHeader("Content-Disposition","attachment; filename=\"record_data.csv\"");
  g_http.streamFile(f,"text/csv");
  f.close();
}

void http_handle_not_found(void) { redirect_home(); }

/* ═══════════════════════════════════════════════════════════════
   OBD-II CORE — pid_request
   ───────────────────────────────────────────────────────────────
   This is the function that was the root cause of the flickering
   "error" readings.  Three fixes applied here:

   FIX A — g_can_mutex held for the ENTIRE send+recv pair.
            No other CAN transaction can interleave.

   FIX B — can_drain_rx() called before sending.
            Any stale frame sitting in the RX FIFO from a previous
            (possibly timed-out) request is discarded so it cannot
            be mistaken for this request's response.

   FIX C — Retry loop (CAN_MAX_RETRIES).
            If the response is missing or mismatched, wait
            CAN_INTER_PID_MS then try again.  This absorbs the
            rare cases where the ECU momentarily does not answer
            without writing "error" to the dashboard.
   ═══════════════════════════════════════════════════════════════ */
static obd2_status_t pid_request(uint8_t mode, uint8_t pid, obd2_result_t *out) {
  const uint8_t req[8] = {0x02, mode, pid, 0,0,0,0,0};

  for (uint8_t attempt = 0; attempt < CAN_MAX_RETRIES; attempt++) {

    CAN_LOCK();
    can_drain_rx();                                    /* FIX B: flush stale frames */

    if (!can_send_raw(CAN_OBD2_REQ_ID, req, 8)) {
      CAN_UNLOCK();
      vTaskDelay(pdMS_TO_TICKS(CAN_INTER_PID_MS));
      continue;                                        /* retry on send failure */
    }

    CAN_FRAME rx;
    obd2_status_t st = can_recv_raw(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
    CAN_UNLOCK();                                      /* FIX A: release after recv */

    if (st != STATUS_OK) {
      /* Timeout — give ECU a moment then retry */
      Serial.printf("[OBD] pid 0x%02X timeout (attempt %u)\n", pid, attempt+1);
      vTaskDelay(pdMS_TO_TICKS(CAN_INTER_PID_MS));
      continue;
    }

    if (rx.data.byte[1] != (uint8_t)(mode+0x40) || rx.data.byte[2] != pid) {
      /* Mismatched response — retry */
      Serial.printf("[OBD] pid 0x%02X mismatch resp=0x%02X (attempt %u)\n",
                    pid, rx.data.byte[1], attempt+1);
      vTaskDelay(pdMS_TO_TICKS(CAN_INTER_PID_MS));
      continue;
    }

    uint8_t dlc = rx.data.byte[0];
    if (dlc < 2) return STATUS_ERROR;

    out->raw_len = (dlc-2 < (uint8_t)sizeof(out->raw))
                 ? (dlc-2) : (uint8_t)sizeof(out->raw);
    for (uint8_t i = 0; i < out->raw_len; i++)
      out->raw[i] = rx.data.byte[3+i];
    return STATUS_OK;                                  /* success */
  }

  return STATUS_ERROR;                                 /* all retries exhausted */
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II CORE — multiframe_request
   Same FIX A + FIX B applied.  Retry is not added here because
   multi-frame sequences (VIN, ECU name) are only triggered
   manually from the TOOLS tab, not from the auto-poll loop.
   ═══════════════════════════════════════════════════════════════ */
static obd2_status_t multiframe_request(uint8_t mode, uint8_t pid,
                                         uint8_t *out_buf, uint8_t buf_len,
                                         uint8_t *out_len) {
  const uint8_t req[8] = {0x02, mode, pid, 0,0,0,0,0};

  CAN_LOCK();
  can_drain_rx();                                      /* FIX B */

  if (!can_send_raw(CAN_OBD2_REQ_ID, req, 8)) { CAN_UNLOCK(); return STATUS_ERROR; }

  CAN_FRAME rx;
  obd2_status_t st = can_recv_raw(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK) { CAN_UNLOCK(); return st; }

  uint8_t ft = rx.data.byte[0] & 0xF0;

  if (ft == ISOTP_SINGLE) {
    uint8_t dlc = rx.data.byte[0] & 0x0F;
    CAN_UNLOCK();
    if (dlc < 4) { out_buf[0]='\0'; *out_len=0; return STATUS_OK; }
    uint8_t dlen = dlc-3; if (dlen>buf_len-1) dlen=buf_len-1;
    memcpy(out_buf, &rx.data.byte[4], dlen);
    out_buf[dlen]='\0'; *out_len=dlen;
    return STATUS_OK;
  }

  if (ft != ISOTP_FIRST) { CAN_UNLOCK(); return STATUS_ERROR; }

  uint16_t total_payload = (uint16_t)(((rx.data.byte[0]&0x0F)<<8)|rx.data.byte[1]);
  uint16_t data_total    = (total_payload>3)?(total_payload-3):0;

  uint8_t idx=0;
  for (uint8_t i=5;i<=7&&idx<buf_len-1;i++) out_buf[idx++]=rx.data.byte[i];

  const uint8_t fc[8]={0x30,0,0,0,0,0,0,0};
  if (!can_send_raw(CAN_OBD2_REQ_ID,fc,8)) { CAN_UNLOCK(); return STATUS_ERROR; }

  uint8_t seq=1;
  while (idx<data_total && idx<buf_len-1) {
    st=can_recv_raw(CAN_OBD2_RESP_ID,&rx,CAN_READ_TIMEOUT_MS);
    if(st!=STATUS_OK){CAN_UNLOCK();return st;}
    if((rx.data.byte[0]&0xF0)!=ISOTP_CONSEC){CAN_UNLOCK();return STATUS_ERROR;}
    if((rx.data.byte[0]&0x0F)!=(seq&0x0F)){CAN_UNLOCK();return STATUS_ERROR;}
    for(uint8_t i=1;i<=7&&idx<buf_len-1;i++) out_buf[idx++]=rx.data.byte[i];
    seq++;
  }
  CAN_UNLOCK();
  out_buf[idx]='\0'; *out_len=idx;
  return STATUS_OK;
}

static const char *dtc_prefix(uint8_t top2) {
  switch(top2&0x03){case 0:return"P";case 1:return"C";case 2:return"B";case 3:return"U";}
  return "?";
}

/* ═══════════════════════════════════════════════════════════════
   RAW COMMAND — FIX A + FIX B applied
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_raw_command(const char *hex_cmd,
                                char *raw_resp, uint8_t raw_len,
                                char *decoded,  uint8_t dec_len) {
  strncpy(raw_resp,"no response",raw_len);
  strncpy(decoded,"--",dec_len);

  size_t hlen=strlen(hex_cmd);
  for(size_t i=0;i<hlen;i++){
    char c=hex_cmd[i];
    if(!((c>='0'&&c<='9')||(c>='A'&&c<='F')||(c>='a'&&c<='f'))){
      strncpy(raw_resp,"invalid cmd (hex only)",raw_len);return STATUS_ERROR;}
  }
  if(hlen<2||hlen>8||hlen%2!=0){
    strncpy(raw_resp,"invalid cmd (need 2-8 hex chars)",raw_len);return STATUS_ERROR;}

  uint8_t frame[8]={0}; uint8_t nbytes=(uint8_t)(hlen/2);
  for(uint8_t i=0;i<nbytes;i++){
    char tmp[3]={hex_cmd[i*2],hex_cmd[i*2+1],0};
    frame[i]=(uint8_t)strtol(tmp,nullptr,16);
  }
  uint8_t req[8]={0}; req[0]=nbytes; memcpy(&req[1],frame,nbytes);

  CAN_LOCK();
  can_drain_rx();                                      /* FIX B */
  if(!can_send_raw(CAN_OBD2_REQ_ID,req,8)){
    CAN_UNLOCK(); strncpy(raw_resp,"CAN send failed",raw_len); return STATUS_ERROR;
  }
  CAN_FRAME rx;
  obd2_status_t result=can_recv_raw(CAN_OBD2_RESP_ID,&rx,CAN_READ_TIMEOUT_MS);
  CAN_UNLOCK();                                        /* FIX A */

  if(result!=STATUS_OK){strncpy(raw_resp,"timeout",raw_len);return STATUS_TIMEOUT;}

  raw_resp[0]=0;
  for(uint8_t i=0;i<8;i++){
    char tmp[6]; snprintf(tmp,sizeof(tmp),"%02X ",rx.data.byte[i]);
    strncat(raw_resp,tmp,raw_len-strlen(raw_resp)-1);
  }

  uint8_t resp_mode=rx.data.byte[1],req_mode=frame[0];
  uint8_t req_pid=(nbytes>=2)?frame[1]:0xFF;

  if(resp_mode==(uint8_t)(req_mode+0x40)&&req_mode==OBD2_SVC_CURRENT){
    uint8_t A=rx.data.byte[3],B=rx.data.byte[4];
    float val=0;bool ok=true;const char *unit="";
    switch(req_pid){
      case PID_RPM:          val=((A*256.0f)+B)/4.0f;      unit="RPM";     break;
      case PID_SPEED:        val=A;                         unit="km/h";    break;
      case PID_COOLANT_TEMP: val=A-40;                      unit="degC";    break;
      case PID_IAT:          val=A-40;                      unit="degC";    break;
      case PID_THROTTLE:     val=A*100.0f/255.0f;           unit="%";       break;
      case PID_ENGINE_LOAD:  val=A*100.0f/255.0f;           unit="%";       break;
      case PID_MAP:          val=A;                         unit="kPa";     break;
      case PID_OIL_TEMP:     val=A-40;                      unit="degC";    break;
      case PID_AMBIENT_TEMP: val=A-40;                      unit="degC";    break;
      case PID_O2_VOLTAGE:   val=A*0.005f;                  unit="V";       break;
      case PID_FUEL_PRESSURE:val=A*3.0f;                    unit="kPa";     break;
      case PID_STFT_B1:
      case PID_LTFT_B1:      val=(A/128.0f-1.0f)*100.0f;   unit="%";       break;
      case PID_IGN_ADVANCE:  val=A/2.0f-64.0f;              unit="degBTDC"; break;
      case PID_BAT_VOLTAGE:  val=((A*256.0f)+B)/1000.0f;    unit="V";       break;
      case PID_RUNTIME:      val=(float)((A<<8)|B);         unit="s";       break;
      default: ok=false;
    }
    if(ok) snprintf(decoded,dec_len,"%.3f %s",val,unit);
    else   snprintf(decoded,dec_len,"A=0x%02X B=0x%02X C=0x%02X D=0x%02X",
                    A,B,rx.data.byte[5],rx.data.byte[6]);
  } else {
    snprintf(decoded,dec_len,"raw: %02X %02X %02X %02X",
             rx.data.byte[0],rx.data.byte[1],rx.data.byte[2],rx.data.byte[3]);
  }
  return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════
   MODE 01 GETTERS — all delegate to pid_request
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_get_rpm(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_RPM,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<2)return STATUS_ERROR;
  out->value=((out->raw[0]*256.0f)+out->raw[1])/4.0f;return STATUS_OK;}
obd2_status_t obd2_get_speed(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_SPEED,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=(float)out->raw[0];return STATUS_OK;}
obd2_status_t obd2_get_coolant(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_COOLANT_TEMP,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=(float)out->raw[0]-40.0f;return STATUS_OK;}
obd2_status_t obd2_get_iat(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_IAT,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=(float)out->raw[0]-40.0f;return STATUS_OK;}
obd2_status_t obd2_get_throttle(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_THROTTLE,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=out->raw[0]*100.0f/255.0f;return STATUS_OK;}
obd2_status_t obd2_get_runtime(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_RUNTIME,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<2)return STATUS_ERROR;
  out->value=(float)((out->raw[0]<<8)|out->raw[1]);return STATUS_OK;}
obd2_status_t obd2_get_batt_voltage(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_BAT_VOLTAGE,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<2)return STATUS_ERROR;
  out->value=(float)((out->raw[0]<<8)|out->raw[1])/1000.0f;return STATUS_OK;}
obd2_status_t obd2_get_ambient(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_AMBIENT_TEMP,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=(float)out->raw[0]-40.0f;return STATUS_OK;}
obd2_status_t obd2_get_engine_load(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_ENGINE_LOAD,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=out->raw[0]*100.0f/255.0f;return STATUS_OK;}
obd2_status_t obd2_get_map(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_MAP,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=(float)out->raw[0];return STATUS_OK;}
obd2_status_t obd2_get_o2_voltage(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_O2_VOLTAGE,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=out->raw[0]*0.005f;return STATUS_OK;}
obd2_status_t obd2_get_fuel_pressure(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_FUEL_PRESSURE,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=(float)out->raw[0]*3.0f;return STATUS_OK;}
obd2_status_t obd2_get_stft_b1(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_STFT_B1,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=((float)out->raw[0]/128.0f-1.0f)*100.0f;return STATUS_OK;}
obd2_status_t obd2_get_ltft_b1(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_LTFT_B1,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=((float)out->raw[0]/128.0f-1.0f)*100.0f;return STATUS_OK;}
obd2_status_t obd2_get_ign_advance(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_IGN_ADVANCE,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=(float)out->raw[0]/2.0f-64.0f;return STATUS_OK;}
obd2_status_t obd2_get_oil_temp(obd2_result_t *out){
  if(pid_request(OBD2_SVC_CURRENT,PID_OIL_TEMP,out)!=STATUS_OK)return STATUS_ERROR;
  if(out->raw_len<1)return STATUS_ERROR;
  out->value=(float)out->raw[0]-40.0f;return STATUS_OK;}

/* ═══════════════════════════════════════════════════════════════
   MODE 03 / 04 — DTCs
   FIX A + FIX B applied here too.
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_read_dtcs(obd2_dtc_result_t *out) {
  const uint8_t req[8]={0x01,OBD2_SVC_DTC_READ,0,0,0,0,0,0};
  CAN_LOCK(); can_drain_rx();
  if(!can_send_raw(CAN_OBD2_REQ_ID,req,8)){CAN_UNLOCK();return STATUS_ERROR;}
  CAN_FRAME rx;
  obd2_status_t st=can_recv_raw(CAN_OBD2_RESP_ID,&rx,CAN_READ_TIMEOUT_MS);
  CAN_UNLOCK();
  if(st!=STATUS_OK)return STATUS_TIMEOUT;
  if(rx.data.byte[1]!=(uint8_t)(OBD2_SVC_DTC_READ+0x40))return STATUS_ERROR;
  out->count=0;
  uint8_t num=rx.data.byte[2];
  for(uint8_t i=0;i<num&&out->count<OBD2_MAX_DTCS;i++){
    uint8_t off=3+(i*2); if(off+1>7)break;
    uint8_t hi=rx.data.byte[off],lo=rx.data.byte[off+1];
    if(!hi&&!lo)continue;
    snprintf(out->codes[out->count],6,"%s%X%02X",
             dtc_prefix(hi>>6),(hi>>4)&0x03,((hi&0x0F)<<4)|(lo>>4));
    char tmp[2]={"0123456789ABCDEF"[lo&0x0F],'\0'};
    strncat(out->codes[out->count],tmp,1);
    out->count++;
  }
  return STATUS_OK;
}

obd2_status_t obd2_clear_dtcs(void) {
  const uint8_t req[8]={0x01,OBD2_SVC_DTC_CLR,0,0,0,0,0,0};
  CAN_LOCK(); can_drain_rx();
  if(!can_send_raw(CAN_OBD2_REQ_ID,req,8)){CAN_UNLOCK();return STATUS_ERROR;}
  CAN_FRAME rx;
  obd2_status_t st=can_recv_raw(CAN_OBD2_RESP_ID,&rx,CAN_READ_TIMEOUT_MS);
  CAN_UNLOCK();
  if(st!=STATUS_OK)return STATUS_TIMEOUT;
  if(rx.data.byte[1]!=(uint8_t)(OBD2_SVC_DTC_CLR+0x40))return STATUS_ERROR;
  Serial.println("[OBD] DTCs cleared");
  return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════
   MODE 09 — VIN / ECU NAME
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_get_vin(char *buf, uint8_t buf_len) {
  uint8_t raw[50]={0};uint8_t rlen=0;
  if(multiframe_request(OBD2_SVC_INFO,PID_VIN,raw,sizeof(raw),&rlen)!=STATUS_OK)
    return STATUS_ERROR;
  uint8_t idx=0;
  for(uint8_t i=0;i<rlen&&idx<buf_len-1;i++)
    if(raw[i]>=0x20&&raw[i]<=0x7E)buf[idx++]=(char)raw[i];
  buf[idx]='\0';return STATUS_OK;
}

obd2_status_t obd2_get_ecu_name(char *buf, uint8_t buf_len) {
  uint8_t raw[50]={0};uint8_t rlen=0;
  if(multiframe_request(OBD2_SVC_INFO,PID_ECU_NAME,raw,sizeof(raw),&rlen)!=STATUS_OK)
    return STATUS_ERROR;
  uint8_t idx=0;
  for(uint8_t i=0;i<rlen&&idx<buf_len-1;i++)
    if(raw[i]>=0x20&&raw[i]<=0x7E)buf[idx++]=(char)raw[i];
  buf[idx]='\0';return STATUS_OK;
}
