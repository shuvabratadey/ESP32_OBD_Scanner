/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║         ESP32  OBD-II  Wi-Fi  Web  Server                                                                 ║
 * ║  • Wi-Fi AP  +  DNS captive-portal  +  HTTP dashboard                                                     ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <esp32_can.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════
   LED CONFIGURATION
   ═══════════════════════════════════════════════════════════════ */
#define LED_PIN             16      /* GPIO_16 — startup blink then off */
#define LED_BLINK_COUNT     5       /* how many times to blink on boot   */
#define LED_BLINK_ON_MS     120     /* LED on duration per blink         */
#define LED_BLINK_OFF_MS    120     /* LED off duration per blink        */

/* ═══════════════════════════════════════════════════════════════
   CAN CONFIGURATION
   ═══════════════════════════════════════════════════════════════ */
#define CAN_TX_PIN          GPIO_NUM_5
#define CAN_RX_PIN          GPIO_NUM_4
#define CAN_SPEED           500000UL
#define CAN_OBD2_REQ_ID     0x7DF
#define CAN_OBD2_RESP_ID    0x7E8
#define CAN_READ_TIMEOUT_MS 200

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
#define PID_VIN           0x02
#define PID_ECU_NAME      0x0A

/* ═══════════════════════════════════════════════════════════════
   Wi-Fi / SERVER CONFIGURATION
   ═══════════════════════════════════════════════════════════════ */
#define WIFI_AP_SSID      "OBD2-Bike"
#define WIFI_AP_PASS      "12345678"
#define WIFI_AP_IP        "192.168.4.1"
#define CAPTIVE_HOSTNAME  "obd.local"
#define DNS_PORT          53
#define HTTP_PORT         80

/* ═══════════════════════════════════════════════════════════════
   EEPROM LAYOUT
     addr 0-3  uint32_t  subscription bitmask (selected params)
     addr 4    uint8_t   first-boot sentinel
   ═══════════════════════════════════════════════════════════════ */
#define EEPROM_SIZE       8
#define EEPROM_ADDR_SUBS  0
#define EEPROM_MAGIC_ADDR 4
#define EEPROM_MAGIC_VAL  0xA5

/* Default params shown on first boot */
#define DEFAULT_SUB_MASK  ( (1UL<<0)|(1UL<<4)|(1UL<<6)|(1UL<<10)|(1UL<<9) )
/*                            RPM     THROTTLE  BAT_VOLT  O2_VOLT   MAP       */

/* ═══════════════════════════════════════════════════════════════
   PARAMETER INDEX ENUM  (bit position in g_sub_mask)
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

/* Human-readable metadata for each param */
struct ParamMeta {
  const char *key;    /* short key used in JSON */
  const char *label;  /* display label */
  const char *unit;   /* unit string   */
  const char *fmt;    /* printf format */
};

static const ParamMeta PARAM_META[PARAM_COUNT] = {
  /* 0  */ { "rpm",        "Engine RPM",        "RPM",  "%.0f"  },
  /* 1  */ { "speed",      "Vehicle Speed",     "km/h", "%.0f"  },
  /* 2  */ { "coolant",    "Coolant Temp",      "°C",   "%.1f"  },
  /* 3  */ { "iat",        "Intake Air Temp",   "°C",   "%.1f"  },
  /* 4  */ { "throttle",   "Throttle Position", "%",    "%.1f"  },
  /* 5  */ { "runtime",    "Engine Runtime",    "s",    "%.0f"  },
  /* 6  */ { "batt",       "Battery Voltage",   "V",    "%.2f"  },
  /* 7  */ { "ambient",    "Ambient Temp",      "°C",   "%.1f"  },
  /* 8  */ { "engload",    "Engine Load",       "%",    "%.1f"  },
  /* 9  */ { "map",        "MAP Pressure",      "kPa",  "%.0f"  },
  /* 10 */ { "o2volt",     "O2 Sensor Voltage", "V",    "%.3f"  },
  /* 11 */ { "fuelpres",   "Fuel Pressure",     "kPa",  "%.0f"  },
  /* 12 */ { "stft",       "Short-term FT B1",  "%",    "%.1f"  },
  /* 13 */ { "ltft",       "Long-term FT B1",   "%",    "%.1f"  },
  /* 14 */ { "ignadv",     "Ignition Advance",  "°BTDC","%.1f"  },
  /* 15 */ { "oiltemp",    "Oil Temperature",   "°C",   "%.1f"  },
};

/* ═══════════════════════════════════════════════════════════════
   STATUS / DATA TYPES
   ═══════════════════════════════════════════════════════════════ */
typedef enum {
  STATUS_ERROR   = -1,
  STATUS_OK      =  0,
  STATUS_TIMEOUT =  2
} obd2_status_t;

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
   GLOBALS
   ═══════════════════════════════════════════════════════════════ */
WebServer  g_http(HTTP_PORT);
DNSServer  g_dns;

volatile uint32_t g_sub_mask   = DEFAULT_SUB_MASK;
bool              g_can_ok     = false;   /* true if CAN init succeeded */

/* Latest polled values (string cache, updated every poll cycle) */
static char g_val_cache[PARAM_COUNT][16];  /* "error" or numeric string */

/* ═══════════════════════════════════════════════════════════════
   FORWARD DECLARATIONS
   ═══════════════════════════════════════════════════════════════ */
void can_init(void);
bool can_send(uint32_t id, const uint8_t *data, uint8_t len);
obd2_status_t can_recv(uint32_t exp_id, CAN_FRAME *out, uint32_t timeout_ms);

void eeprom_init(void);
void eeprom_save(uint32_t mask);

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
void http_handle_not_found(void);

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
static const char *dtc_prefix(uint8_t top2);
static void poll_active_params(void);
static void led_startup_blink(void);

/* ═══════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════ */
void setup() {
  Serial.begin(115200);
  delay(100);
  
  led_startup_blink();
  Serial.println("\n============================================");
  Serial.println("  ESP32  OBD-II  Web Server");
  Serial.println("============================================");

  /* Init value cache to "--" */
  for (int i = 0; i < PARAM_COUNT; i++)
    strncpy(g_val_cache[i], "--", sizeof(g_val_cache[i]));

  eeprom_init();
  can_init();
  wifi_ap_init();
  dns_init();
  http_init();

  Serial.println("System ready — connect to Wi-Fi: " WIFI_AP_SSID);
  Serial.println("Browse to: http://" CAPTIVE_HOSTNAME "  or  http://" WIFI_AP_IP);
}

/* ═══════════════════════════════════════════════════════════════
   LOOP
   ═══════════════════════════════════════════════════════════════ */
void loop() {
  g_dns.processNextRequest();
  g_http.handleClient();
  poll_active_params();   /* blocking ~1 s when params are active */
}

static void led_startup_blink(void) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  for (int i = 0; i < LED_BLINK_COUNT; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(LED_BLINK_ON_MS);
    digitalWrite(LED_PIN, LOW);
    delay(LED_BLINK_OFF_MS);
  }

  /* Ensure LED stays off permanently */
  digitalWrite(LED_PIN, LOW);
  Serial.println("[LED] Startup blink complete — LED off");
}

/* ═══════════════════════════════════════════════════════════════
   POLL ACTIVE PARAMS  (called from loop)
   Updates g_val_cache[] for every bit set in g_sub_mask.
   ═══════════════════════════════════════════════════════════════ */
static void poll_active_params(void) {
  static unsigned long last_poll = 0;
  if (millis() - last_poll < 1000) return;
  last_poll = millis();

  uint32_t active = g_sub_mask;
  if (active == 0) return;

  obd2_result_t res;

#define POLL(PARAM, FMT, GETTER)                                \
  if (active & (1UL << (PARAM))) {                              \
    if ((GETTER)(&res) == STATUS_OK)                            \
      snprintf(g_val_cache[(PARAM)], 16, (FMT), res.value);    \
    else                                                        \
      strncpy(g_val_cache[(PARAM)], "error", 16);              \
    Serial.printf("[OBD] %-18s : %s %s\n",                     \
      PARAM_META[(PARAM)].label,                                \
      g_val_cache[(PARAM)],                                     \
      PARAM_META[(PARAM)].unit);                                \
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
  Serial.println("[OBD] ---- poll cycle complete ----");
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

bool can_send(uint32_t id, const uint8_t *data, uint8_t len) {
  if (!g_can_ok) return false;
  CAN_FRAME tx;
  tx.id       = id;
  tx.extended = false;
  tx.rtr      = 0;
  tx.length   = (len > 8) ? 8 : len;
  for (uint8_t i = 0; i < tx.length; i++) tx.data.byte[i] = data[i];
  return CAN0.sendFrame(tx);
}

obd2_status_t can_recv(uint32_t exp_id, CAN_FRAME *out, uint32_t timeout_ms) {
  uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    if (CAN0.read(*out))
      if ((exp_id == 0) || (out->id == exp_id))
        return STATUS_OK;
    delay(1);
  }
  return STATUS_TIMEOUT;
}

/* ═══════════════════════════════════════════════════════════════
   EEPROM LAYER
   ═══════════════════════════════════════════════════════════════ */
void eeprom_init(void) {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) {
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
    EEPROM.put(EEPROM_ADDR_SUBS, (uint32_t)DEFAULT_SUB_MASK);
    EEPROM.commit();
    g_sub_mask = DEFAULT_SUB_MASK;
    Serial.printf("[EEPROM] First boot — default mask: 0x%08X\n",
                  (unsigned)DEFAULT_SUB_MASK);
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
   Wi-Fi ACCESS POINT
   ═══════════════════════════════════════════════════════════════ */
void wifi_ap_init(void) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  delay(200);
  IPAddress apIP;
  apIP.fromString(WIFI_AP_IP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255,255,255,0));
  Serial.printf("[WiFi] AP \"%s\" started — IP: %s\n",
                WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

  /* Station connect/disconnect events */
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("[WiFi] Device connected   — MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      info.wifi_ap_staconnected.mac[0], info.wifi_ap_staconnected.mac[1],
      info.wifi_ap_staconnected.mac[2], info.wifi_ap_staconnected.mac[3],
      info.wifi_ap_staconnected.mac[4], info.wifi_ap_staconnected.mac[5]);
    Serial.printf("[WiFi] Total clients: %d\n", WiFi.softAPgetStationNum());
  }, ARDUINO_EVENT_WIFI_AP_STACONNECTED);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("[WiFi] Device disconnected — MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      info.wifi_ap_stadisconnected.mac[0], info.wifi_ap_stadisconnected.mac[1],
      info.wifi_ap_stadisconnected.mac[2], info.wifi_ap_stadisconnected.mac[3],
      info.wifi_ap_stadisconnected.mac[4], info.wifi_ap_stadisconnected.mac[5]);
    Serial.printf("[WiFi] Remaining clients: %d\n", WiFi.softAPgetStationNum());
  }, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
}

/* ═══════════════════════════════════════════════════════════════
   DNS CAPTIVE PORTAL  (redirects all queries to AP IP)
   ═══════════════════════════════════════════════════════════════ */
void dns_init(void) {
  IPAddress apIP;
  apIP.fromString(WIFI_AP_IP);
  g_dns.start(DNS_PORT, "*", apIP);
  Serial.println("[DNS] Captive portal DNS started");
}

/* ═══════════════════════════════════════════════════════════════
   HTTP SERVER — ROUTE SETUP
   ═══════════════════════════════════════════════════════════════ */
void http_init(void) {
  g_http.on("/",                HTTP_GET,  http_handle_root);
  g_http.on("/index.html",      HTTP_GET,  http_handle_root);

  /* Captive-portal probe URLs (iOS, Android, Windows) */
  g_http.on("/generate_204",    HTTP_GET,  http_handle_root);
  g_http.on("/connecttest.txt", HTTP_GET,  http_handle_root);
  g_http.on("/hotspot-detect.html", HTTP_GET, http_handle_root);
  g_http.on("/ncsi.txt",        HTTP_GET,  http_handle_root);
  g_http.on("/fwlink",          HTTP_GET,  http_handle_root);

  /* JSON API endpoints */
  g_http.on("/api/data",        HTTP_GET,  http_handle_data);
  g_http.on("/api/settings",    HTTP_GET,  http_handle_settings_get);
  g_http.on("/api/settings",    HTTP_POST, http_handle_settings_post);
  g_http.on("/api/dtc/read",    HTTP_GET,  http_handle_dtc_read);
  g_http.on("/api/dtc/clear",   HTTP_GET,  http_handle_dtc_clear);
  g_http.on("/api/vin",         HTTP_GET,  http_handle_vin);
  g_http.on("/api/ecu",         HTTP_GET,  http_handle_ecu_name);
  g_http.on("/api/raw",         HTTP_POST, http_handle_raw_cmd);

  g_http.onNotFound(http_handle_not_found);
  g_http.begin();
  Serial.println("[HTTP] Server started on port 80");
}

/* ─── Helper: add CORS headers and log request ─────────────────── */
static void begin_response(int code, const char *ct) {
  g_http.sendHeader("Access-Control-Allow-Origin", "*");
  g_http.sendHeader("Cache-Control", "no-cache");
  Serial.printf("[HTTP] %s %s → %d\n",
    (g_http.method() == HTTP_GET ? "GET" : "POST"),
    g_http.uri().c_str(), code);
}

/* ─── Redirect helper (captive portal) ─────────────────────────── */
static void redirect_home(void) {
  g_http.sendHeader("Location", "http://" WIFI_AP_IP);
  g_http.send(302, "text/plain", "");
  Serial.printf("[HTTP] Redirect → %s\n", "http://" WIFI_AP_IP);
}

/* ═══════════════════════════════════════════════════════════════
   INLINE HTML PAGE
   Served from program memory to save RAM.
   Single-file: HTML + CSS + JS in one response.
   ═══════════════════════════════════════════════════════════════ */
static const char HTML_PAGE[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"/>
<title>OBD2 Live Dashboard</title>
<style>
  :root {
    --bg:       #0a0c0f;
    --surface:  #111418;
    --card:     #181d24;
    --border:   #252c38;
    --accent:   #00e5ff;
    --accent2:  #ff6b35;
    --text:     #e8edf5;
    --muted:    #5a6478;
    --danger:   #ff3b3b;
    --ok:       #00e5a0;
    --font:     'Courier New', 'Lucida Console', monospace;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  html, body { background: var(--bg); color: var(--text); font-family: var(--font); min-height: 100vh; }

  /* ── TOP NAV ── */
  nav {
    position: sticky; top: 0; z-index: 100;
    display: flex; align-items: center; justify-content: space-between;
    padding: 0 16px; height: 52px;
    background: rgba(10,12,15,0.92);
    border-bottom: 1px solid var(--border);
    backdrop-filter: blur(8px);
  }
  .nav-brand { display: flex; align-items: center; gap: 10px; font-size: 1rem; font-weight: 700; letter-spacing: 0.08em; color: var(--accent); }
  .nav-brand svg { width: 22px; height: 22px; }
  .nav-status { display: flex; align-items: center; gap: 8px; font-size: 0.72rem; color: var(--muted); }
  .status-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--muted); transition: background 0.4s; }
  .status-dot.ok  { background: var(--ok); box-shadow: 0 0 6px var(--ok); }
  .status-dot.err { background: var(--danger); box-shadow: 0 0 6px var(--danger); }
  .btn-icon { background: none; border: 1px solid var(--border); color: var(--text); border-radius: 6px; padding: 6px 10px; cursor: pointer; font-size: 1rem; transition: border-color 0.2s, color 0.2s; }
  .btn-icon:hover { border-color: var(--accent); color: var(--accent); }

  /* ── TABS ── */
  .tabs { display: flex; gap: 0; border-bottom: 1px solid var(--border); background: var(--surface); }
  .tab { flex: 1; text-align: center; padding: 11px 6px; font-size: 0.72rem; letter-spacing: 0.06em; color: var(--muted); cursor: pointer; border-bottom: 2px solid transparent; transition: color 0.2s, border-color 0.2s; user-select: none; }
  .tab.active { color: var(--accent); border-bottom-color: var(--accent); }

  /* ── PANELS ── */
  .panel { display: none; padding: 14px 12px; }
  .panel.active { display: block; }

  /* ── GAUGE GRID ── */
  .gauge-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(140px, 1fr)); gap: 10px; }
  @media (max-width: 380px) { .gauge-grid { grid-template-columns: 1fr 1fr; } }

  .gauge-card {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 14px 12px 12px;
    display: flex; flex-direction: column; gap: 6px;
    position: relative; overflow: hidden;
    transition: border-color 0.3s;
  }
  .gauge-card::before {
    content: ''; position: absolute; inset: 0 0 auto 0; height: 2px;
    background: linear-gradient(90deg, var(--accent) 0%, transparent 100%);
    opacity: 0.5;
  }
  .gauge-label { font-size: 0.62rem; letter-spacing: 0.08em; color: var(--muted); text-transform: uppercase; }
  .gauge-value { font-size: 1.7rem; font-weight: 700; color: var(--accent); line-height: 1; letter-spacing: -0.02em; }
  .gauge-value.error { font-size: 0.9rem; color: var(--danger); }
  .gauge-unit  { font-size: 0.65rem; color: var(--muted); }

  /* ── SPINNER ── */
  .spinner-wrap { display: none; flex-direction: column; align-items: center; justify-content: center; gap: 16px; padding: 60px 20px; }
  .spinner-wrap.show { display: flex; }
  .spinner { width: 48px; height: 48px; border: 3px solid var(--border); border-top-color: var(--accent); border-radius: 50%; animation: spin 0.9s linear infinite; }
  @keyframes spin { to { transform: rotate(360deg); } }
  .spinner-msg { color: var(--muted); font-size: 0.8rem; letter-spacing: 0.06em; }

  /* ── SETTINGS PANEL ── */
  .settings-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(160px, 1fr)); gap: 8px; }
  .setting-item {
    display: flex; align-items: center; gap: 10px;
    background: var(--card); border: 1px solid var(--border);
    border-radius: 8px; padding: 10px 12px; cursor: pointer;
    transition: border-color 0.2s;
  }
  .setting-item.checked { border-color: var(--accent); }
  .setting-item input { display: none; }
  .check-box {
    width: 18px; height: 18px; border: 1.5px solid var(--border);
    border-radius: 4px; flex-shrink: 0; display: flex; align-items: center; justify-content: center;
    transition: background 0.2s, border-color 0.2s;
  }
  .setting-item.checked .check-box { background: var(--accent); border-color: var(--accent); }
  .check-box svg { display: none; }
  .setting-item.checked .check-box svg { display: block; }
  .setting-text { font-size: 0.72rem; color: var(--text); line-height: 1.3; }

  .btn-save {
    margin-top: 16px; width: 100%; padding: 12px;
    background: var(--accent); color: #000; font-weight: 700; font-family: var(--font);
    font-size: 0.85rem; letter-spacing: 0.1em; border: none; border-radius: 8px;
    cursor: pointer; transition: opacity 0.2s;
  }
  .btn-save:hover { opacity: 0.85; }

  /* ── TOOLS PANEL ── */
  .tool-section { margin-bottom: 20px; }
  .tool-title { font-size: 0.65rem; letter-spacing: 0.1em; color: var(--muted); text-transform: uppercase; margin-bottom: 10px; }
  .tool-btn-row { display: flex; flex-wrap: wrap; gap: 8px; }
  .tool-btn {
    flex: 1; min-width: 120px; padding: 11px 10px;
    background: var(--card); border: 1px solid var(--border);
    color: var(--text); font-family: var(--font); font-size: 0.75rem;
    border-radius: 8px; cursor: pointer; text-align: center;
    transition: border-color 0.2s, color 0.2s;
  }
  .tool-btn:hover { border-color: var(--accent); color: var(--accent); }
  .tool-btn.danger:hover { border-color: var(--danger); color: var(--danger); }

  .result-box {
    width: 100%; min-height: 60px; padding: 10px;
    background: #0d1017; border: 1px solid var(--border);
    color: var(--ok); font-family: var(--font); font-size: 0.78rem;
    border-radius: 8px; resize: vertical; margin-top: 8px;
    white-space: pre-wrap; word-break: break-all;
  }

  /* ── RAW CMD ── */
  .raw-row { display: flex; gap: 8px; margin-bottom: 8px; }
  .raw-input {
    flex: 1; padding: 10px 12px;
    background: var(--card); border: 1px solid var(--border);
    color: var(--text); font-family: var(--font); font-size: 0.85rem;
    border-radius: 8px; outline: none; text-transform: uppercase;
    transition: border-color 0.2s;
  }
  .raw-input:focus { border-color: var(--accent); }
  .raw-send {
    padding: 10px 16px;
    background: var(--accent2); border: none; color: #fff;
    font-family: var(--font); font-size: 0.8rem; font-weight: 700;
    border-radius: 8px; cursor: pointer; transition: opacity 0.2s;
  }
  .raw-send:hover { opacity: 0.85; }
  .raw-labels { display: flex; gap: 8px; }
  .raw-label { font-size: 0.62rem; color: var(--muted); letter-spacing: 0.06em; flex: 1; margin-bottom: 4px; }

  /* ── TOAST ── */
  .toast {
    position: fixed; bottom: 24px; left: 50%; transform: translateX(-50%);
    background: var(--surface); border: 1px solid var(--border);
    color: var(--text); padding: 10px 20px; border-radius: 20px;
    font-size: 0.78rem; opacity: 0; pointer-events: none;
    transition: opacity 0.3s; z-index: 999;
  }
  .toast.show { opacity: 1; }

  /* ── LOADING OVERLAY ── */
  .btn-loading { opacity: 0.5; pointer-events: none; }
</style>
</head>
<body>

<nav>
  <div class="nav-brand">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
      <circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M4.22 4.22l2.12 2.12M17.66 17.66l2.12 2.12M2 12h3M19 12h3M4.22 19.78l2.12-2.12M17.66 6.34l2.12-2.12"/>
    </svg>
    OBD2 LIVE
  </div>
  <div class="nav-status">
    <div class="status-dot" id="statusDot"></div>
    <span id="statusText">connecting…</span>
  </div>
</nav>

<div class="tabs">
  <div class="tab active" data-tab="dash">DASH</div>
  <div class="tab" data-tab="tools">TOOLS</div>
  <div class="tab" data-tab="raw">RAW CMD</div>
  <div class="tab" data-tab="settings">SETTINGS</div>
</div>

<!-- ═══ DASHBOARD PANEL ═══ -->
<div class="panel active" id="tab-dash">
  <div class="spinner-wrap" id="dashSpinner">
    <div class="spinner"></div>
    <div class="spinner-msg">Waiting for ECU response…</div>
  </div>
  <div class="gauge-grid" id="gaugeGrid"></div>
</div>

<!-- ═══ TOOLS PANEL ═══ -->
<div class="panel" id="tab-tools">
  <div class="tool-section">
    <div class="tool-title">Vehicle Info</div>
    <div class="tool-btn-row">
      <button class="tool-btn" onclick="toolAction('vin','vinOut')">Read VIN</button>
      <button class="tool-btn" onclick="toolAction('ecu','ecuOut')">Read ECU Name</button>
    </div>
    <textarea class="result-box" id="vinOut" readonly placeholder="VIN result…"></textarea>
    <textarea class="result-box" id="ecuOut" readonly placeholder="ECU name result…"></textarea>
  </div>
  <div class="tool-section">
    <div class="tool-title">Diagnostic Trouble Codes</div>
    <div class="tool-btn-row">
      <button class="tool-btn" onclick="toolAction('dtc/read','dtcOut')">Read DTCs</button>
      <button class="tool-btn danger" onclick="toolAction('dtc/clear','dtcOut')">Clear DTCs</button>
    </div>
    <textarea class="result-box" id="dtcOut" readonly placeholder="DTC result…"></textarea>
  </div>
</div>

<!-- ═══ RAW CMD PANEL ═══ -->
<div class="panel" id="tab-raw">
  <div class="tool-section">
    <div class="tool-title">Custom OBD-II Hex Command</div>
    <div class="raw-row">
      <input class="raw-input" id="rawInput" maxlength="8" placeholder="e.g. 010C" />
      <button class="raw-send" onclick="sendRaw()">SEND</button>
    </div>
    <div class="raw-labels">
      <div class="raw-label">RAW RESPONSE</div>
      <div class="raw-label">DECODED VALUE</div>
    </div>
    <div style="display:flex;gap:8px;">
      <textarea class="result-box" id="rawOut"  readonly placeholder="Raw hex bytes…"></textarea>
      <textarea class="result-box" id="decOut"  readonly placeholder="Decoded result…"></textarea>
    </div>
  </div>
</div>

<!-- ═══ SETTINGS PANEL ═══ -->
<div class="panel" id="tab-settings">
  <div class="tool-title" style="padding:4px 0 12px;">Select parameters to display</div>
  <div class="settings-grid" id="settingsGrid"></div>
  <button class="btn-save" onclick="saveSettings()">SAVE SETTINGS</button>
</div>

<div class="toast" id="toast"></div>

<script>
/* ── Param metadata (mirrors firmware) ── */
const PARAMS = [
  {id:0,  key:'rpm',      label:'Engine RPM',        unit:'RPM'},
  {id:1,  key:'speed',    label:'Vehicle Speed',      unit:'km/h'},
  {id:2,  key:'coolant',  label:'Coolant Temp',       unit:'°C'},
  {id:3,  key:'iat',      label:'Intake Air Temp',    unit:'°C'},
  {id:4,  key:'throttle', label:'Throttle Position',  unit:'%'},
  {id:5,  key:'runtime',  label:'Engine Runtime',     unit:'s'},
  {id:6,  key:'batt',     label:'Battery Voltage',    unit:'V'},
  {id:7,  key:'ambient',  label:'Ambient Temp',       unit:'°C'},
  {id:8,  key:'engload',  label:'Engine Load',        unit:'%'},
  {id:9,  key:'map',      label:'MAP Pressure',       unit:'kPa'},
  {id:10, key:'o2volt',   label:'O2 Sensor Voltage',  unit:'V'},
  {id:11, key:'fuelpres', label:'Fuel Pressure',      unit:'kPa'},
  {id:12, key:'stft',     label:'Short-term FT B1',   unit:'%'},
  {id:13, key:'ltft',     label:'Long-term FT B1',    unit:'%'},
  {id:14, key:'ignadv',   label:'Ignition Advance',   unit:'°BTDC'},
  {id:15, key:'oiltemp',  label:'Oil Temperature',    unit:'°C'},
];

let currentMask = 0;
let allErrors   = 0;
let pollTimer   = null;

/* ── Tab switching ── */
document.querySelectorAll('.tab').forEach(tab => {
  tab.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById('tab-' + tab.dataset.tab).classList.add('active');
  });
});

/* ── Build gauge grid ── */
function buildGauges(mask) {
  const grid = document.getElementById('gaugeGrid');
  grid.innerHTML = '';
  PARAMS.forEach(p => {
    if (!(mask & (1 << p.id))) return;
    const card = document.createElement('div');
    card.className = 'gauge-card';
    card.id = 'card-' + p.key;
    card.innerHTML = `
      <div class="gauge-label">${p.label}</div>
      <div class="gauge-value" id="val-${p.key}">--</div>
      <div class="gauge-unit">${p.unit}</div>`;
    grid.appendChild(card);
  });
}

/* ── Build settings checkboxes ── */
function buildSettings(mask) {
  const grid = document.getElementById('settingsGrid');
  grid.innerHTML = '';
  PARAMS.forEach(p => {
    const checked = !!(mask & (1 << p.id));
    const item = document.createElement('label');
    item.className = 'setting-item' + (checked ? ' checked' : '');
    item.innerHTML = `
      <input type="checkbox" data-id="${p.id}" ${checked ? 'checked' : ''}>
      <div class="check-box">
        <svg width="11" height="8" viewBox="0 0 11 8" fill="none">
          <path d="M1 4L4 7L10 1" stroke="#000" stroke-width="2" stroke-linecap="round"/>
        </svg>
      </div>
      <div class="setting-text">${p.label}<br><span style="color:var(--muted);font-size:0.6rem">${p.unit}</span></div>`;
    item.querySelector('input').addEventListener('change', function() {
      item.classList.toggle('checked', this.checked);
    });
    grid.appendChild(item);
  });
}

/* ── Update gauge values ── */
function updateGauges(data) {
  allErrors = 0;
  PARAMS.forEach(p => {
    const el = document.getElementById('val-' + p.key);
    if (!el) return;
    const v = data[p.key];
    if (v === undefined || v === '--') return;
    if (v === 'error') { el.textContent = 'ERR'; el.className = 'gauge-value error'; allErrors++; }
    else               { el.textContent = v;     el.className = 'gauge-value'; }
  });
}

/* ── Status indicator ── */
function setStatus(ok) {
  const dot  = document.getElementById('statusDot');
  const text = document.getElementById('statusText');
  const spin = document.getElementById('dashSpinner');
  dot.className  = 'status-dot ' + (ok ? 'ok' : 'err');
  text.textContent = ok ? 'ECU connected' : 'No ECU response';
  spin.classList.toggle('show', !ok);
}

/* ── Poll live data ── */
async function pollData() {
  try {
    const r = await fetch('/api/data');
    if (!r.ok) throw new Error('HTTP ' + r.status);
    const d = await r.json();
    currentMask = d.mask;
    buildGauges(currentMask);
    updateGauges(d.values);
    const hasError = Object.values(d.values).some(v => v === 'error');
    setStatus(!hasError || Object.values(d.values).some(v => v !== 'error' && v !== '--'));
  } catch(e) {
    setStatus(false);
    console.error('Poll error:', e);
  }
}

/* ── Load settings from device ── */
async function loadSettings() {
  try {
    const r = await fetch('/api/settings');
    const d = await r.json();
    currentMask = d.mask;
    buildGauges(currentMask);
    buildSettings(currentMask);
  } catch(e) { console.error('Settings load error:', e); }
}

/* ── Save settings ── */
async function saveSettings() {
  let mask = 0;
  document.querySelectorAll('#settingsGrid input[type=checkbox]').forEach(cb => {
    if (cb.checked) mask |= (1 << parseInt(cb.dataset.id));
  });
  try {
    const r = await fetch('/api/settings', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({mask})
    });
    const d = await r.json();
    currentMask = d.mask;
    buildGauges(currentMask);
    showToast('Settings saved ✓');
  } catch(e) { showToast('Save failed'); }
}

/* ── Tool buttons ── */
async function toolAction(endpoint, outId) {
  const box = document.getElementById(outId);
  box.value = 'Loading…';
  try {
    const r = await fetch('/api/' + endpoint);
    const d = await r.json();
    box.value = d.result || JSON.stringify(d);
  } catch(e) { box.value = 'Error: ' + e.message; }
}

/* ── Raw command ── */
async function sendRaw() {
  const cmd = document.getElementById('rawInput').value.trim();
  if (!cmd) { showToast('Enter a hex command'); return; }
  document.getElementById('rawOut').value = 'Sending…';
  document.getElementById('decOut').value = '';
  try {
    const r = await fetch('/api/raw', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({cmd})
    });
    const d = await r.json();
    document.getElementById('rawOut').value = d.raw   || 'no response';
    document.getElementById('decOut').value = d.decoded || '--';
  } catch(e) {
    document.getElementById('rawOut').value = 'Error: ' + e.message;
  }
}

/* ── Toast notification ── */
function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2200);
}

/* ── Boot ── */
loadSettings();
setInterval(pollData, 1500);
</script>
</body>
</html>
)HTMLEOF";

/* ═══════════════════════════════════════════════════════════════
   HTTP HANDLERS
   ═══════════════════════════════════════════════════════════════ */

/* GET / — serve the dashboard HTML */
void http_handle_root(void) {
  Serial.printf("[HTTP] GET %s — serving dashboard\n", g_http.uri().c_str());
  g_http.sendHeader("Cache-Control", "no-cache");
  g_http.send_P(200, "text/html", HTML_PAGE);
}

/* GET /api/data — return current cached values + active mask */
void http_handle_data(void) {
  begin_response(200, "application/json");

  StaticJsonDocument<512> doc;
  doc["mask"] = (unsigned long)g_sub_mask;

  JsonObject vals = doc.createNestedObject("values");
  for (int i = 0; i < PARAM_COUNT; i++)
    vals[PARAM_META[i].key] = g_val_cache[i];

  String out;
  serializeJson(doc, out);
  g_http.send(200, "application/json", out);
}

/* GET /api/settings — return active subscription mask */
void http_handle_settings_get(void) {
  begin_response(200, "application/json");
  StaticJsonDocument<64> doc;
  doc["mask"] = (unsigned long)g_sub_mask;
  String out; serializeJson(doc, out);
  g_http.send(200, "application/json", out);
  Serial.printf("[HTTP] Settings read — mask: 0x%08X\n", (unsigned)g_sub_mask);
}

/* POST /api/settings  body: {"mask":12345} */
void http_handle_settings_post(void) {
  begin_response(200, "application/json");
  if (!g_http.hasArg("plain")) { g_http.send(400, "application/json", "{\"error\":\"no body\"}"); return; }

  StaticJsonDocument<64> req;
  DeserializationError err = deserializeJson(req, g_http.arg("plain"));
  if (err) { g_http.send(400, "application/json", "{\"error\":\"bad json\"}"); return; }

  uint32_t newMask = (uint32_t)(unsigned long)req["mask"];
  newMask &= ((1UL << PARAM_COUNT) - 1);   /* clamp to valid bits */
  g_sub_mask = newMask;
  eeprom_save(newMask);

  /* Reset cache for newly removed params */
  for (int i = 0; i < PARAM_COUNT; i++)
    if (!(newMask & (1UL << i)))
      strncpy(g_val_cache[i], "--", 16);

  StaticJsonDocument<64> resp;
  resp["mask"] = (unsigned long)newMask;
  String out; serializeJson(resp, out);
  g_http.send(200, "application/json", out);
  Serial.printf("[HTTP] Settings saved — new mask: 0x%08X\n", (unsigned)newMask);
}

/* GET /api/dtc/read */
void http_handle_dtc_read(void) {
  begin_response(200, "application/json");
  obd2_dtc_result_t dtc = {};
  obd2_status_t st = obd2_read_dtcs(&dtc);

  StaticJsonDocument<256> doc;
  if (st == STATUS_OK) {
    if (dtc.count == 0) {
      doc["result"] = "No fault codes found";
    } else {
      String s;
      for (uint8_t i = 0; i < dtc.count; i++) {
        if (i) s += ", ";
        s += dtc.codes[i];
      }
      doc["result"] = s;
    }
  } else {
    doc["result"] = "Error reading DTCs";
  }
  String out; serializeJson(doc, out);
  g_http.send(200, "application/json", out);
  Serial.printf("[HTTP] DTC read: %s\n", doc["result"].as<const char*>());
}

/* GET /api/dtc/clear */
void http_handle_dtc_clear(void) {
  begin_response(200, "application/json");
  obd2_status_t st = obd2_clear_dtcs();
  StaticJsonDocument<64> doc;
  doc["result"] = (st == STATUS_OK) ? "DTCs cleared successfully" : "Error clearing DTCs";
  String out; serializeJson(doc, out);
  g_http.send(200, "application/json", out);
  Serial.printf("[HTTP] DTC clear: %s\n", doc["result"].as<const char*>());
}

/* GET /api/vin */
void http_handle_vin(void) {
  begin_response(200, "application/json");
  char buf[64] = {0};
  obd2_status_t st = obd2_get_vin(buf, sizeof(buf));
  StaticJsonDocument<128> doc;
  doc["result"] = (st == STATUS_OK && buf[0]) ? buf : "Error reading VIN";
  String out; serializeJson(doc, out);
  g_http.send(200, "application/json", out);
  Serial.printf("[HTTP] VIN: %s\n", buf);
}

/* GET /api/ecu */
void http_handle_ecu_name(void) {
  begin_response(200, "application/json");
  char buf[64] = {0};
  obd2_status_t st = obd2_get_ecu_name(buf, sizeof(buf));
  StaticJsonDocument<128> doc;
  doc["result"] = (st == STATUS_OK && buf[0]) ? buf : "Error reading ECU name";
  String out; serializeJson(doc, out);
  g_http.send(200, "application/json", out);
  Serial.printf("[HTTP] ECU name: %s\n", buf);
}

/* POST /api/raw  body: {"cmd":"010C"} */
void http_handle_raw_cmd(void) {
  begin_response(200, "application/json");
  if (!g_http.hasArg("plain")) { g_http.send(400, "application/json", "{\"error\":\"no body\"}"); return; }

  StaticJsonDocument<64> req;
  if (deserializeJson(req, g_http.arg("plain"))) { g_http.send(400, "application/json", "{\"error\":\"bad json\"}"); return; }

  const char *cmd = req["cmd"] | "";
  char rawBuf[64]  = {0};
  char decBuf[64]  = {0};

  obd2_raw_command(cmd, rawBuf, sizeof(rawBuf), decBuf, sizeof(decBuf));

  StaticJsonDocument<192> resp;
  resp["raw"]     = rawBuf;
  resp["decoded"] = decBuf;
  String out; serializeJson(resp, out);
  g_http.send(200, "application/json", out);
  Serial.printf("[HTTP] Raw cmd '%s' → raw: %s  decoded: %s\n", cmd, rawBuf, decBuf);
}

/* 404 / captive portal catch-all */
void http_handle_not_found(void) {
  Serial.printf("[HTTP] 404 → redirect  URI: %s\n", g_http.uri().c_str());
  redirect_home();
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II INTERNAL HELPERS
   ═══════════════════════════════════════════════════════════════ */
static obd2_status_t pid_request(uint8_t mode, uint8_t pid, obd2_result_t *out) {
  const uint8_t req[8] = {0x02, mode, pid, 0,0,0,0,0};
  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) return STATUS_ERROR;

  CAN_FRAME rx;
  obd2_status_t st = can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK) return st;

  if (rx.data.byte[1] != (uint8_t)(mode+0x40) || rx.data.byte[2] != pid)
    return STATUS_ERROR;

  uint8_t dlc = rx.data.byte[0];
  if (dlc < 2) return STATUS_ERROR;

  out->raw_len = (dlc-2 < (uint8_t)sizeof(out->raw)) ? (dlc-2) : (uint8_t)sizeof(out->raw);
  for (uint8_t i = 0; i < out->raw_len; i++)
    out->raw[i] = rx.data.byte[3+i];
  return STATUS_OK;
}

static obd2_status_t multiframe_request(uint8_t mode, uint8_t pid,
                                         uint8_t *out_buf, uint8_t buf_len,
                                         uint8_t *out_len) {
  const uint8_t req[8] = {0x02, mode, pid, 0,0,0,0,0};
  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) return STATUS_ERROR;

  CAN_FRAME rx;
  obd2_status_t st = can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK) return st;

  uint8_t ft = rx.data.byte[0] & 0xF0;

  if (ft == ISOTP_SINGLE) {
    uint8_t dlc  = rx.data.byte[0] & 0x0F;
    if (dlc < 4) { out_buf[0]='\0'; *out_len=0; return STATUS_OK; }
    uint8_t dlen = dlc - 3;
    if (dlen > buf_len-1) dlen = buf_len-1;
    memcpy(out_buf, &rx.data.byte[4], dlen);
    out_buf[dlen] = '\0'; *out_len = dlen;
    return STATUS_OK;
  }

  if (ft != ISOTP_FIRST) return STATUS_ERROR;

  uint16_t total_payload = (uint16_t)(((rx.data.byte[0]&0x0F)<<8)|rx.data.byte[1]);
  uint16_t data_total    = (total_payload > 3) ? (total_payload-3) : 0;

  uint8_t idx = 0;
  for (uint8_t i = 5; i <= 7 && idx < buf_len-1; i++)
    out_buf[idx++] = rx.data.byte[i];

  const uint8_t fc[8] = {0x30,0,0,0,0,0,0,0};
  if (!can_send(CAN_OBD2_REQ_ID, fc, 8)) return STATUS_ERROR;

  uint8_t seq = 1;
  while (idx < data_total && idx < buf_len-1) {
    st = can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
    if (st != STATUS_OK) return st;
    if ((rx.data.byte[0]&0xF0) != ISOTP_CONSEC) return STATUS_ERROR;
    if ((rx.data.byte[0]&0x0F) != (seq&0x0F))   return STATUS_ERROR;
    for (uint8_t i = 1; i <= 7 && idx < buf_len-1; i++)
      out_buf[idx++] = rx.data.byte[i];
    seq++;
  }
  out_buf[idx] = '\0'; *out_len = idx;
  return STATUS_OK;
}

static const char *dtc_prefix(uint8_t top2) {
  switch (top2&0x03) {
    case 0: return "P"; case 1: return "C";
    case 2: return "B"; case 3: return "U";
  }
  return "?";
}

/* ═══════════════════════════════════════════════════════════════
   RAW COMMAND HANDLER
   Parses a 2-8 char hex string (e.g. "010C"), sends it as a CAN
   OBD frame, returns raw bytes as hex string and a basic decode.
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_raw_command(const char *hex_cmd,
                                char *raw_resp, uint8_t raw_len,
                                char *decoded,  uint8_t dec_len) {
  strncpy(raw_resp, "no response", raw_len);
  strncpy(decoded,  "--",          dec_len);

  /* Convert hex string to bytes */
  size_t hlen = strlen(hex_cmd);
  if (hlen < 2 || hlen > 8 || hlen%2 != 0) {
    strncpy(raw_resp, "invalid cmd (need 2-8 hex chars)", raw_len);
    return STATUS_ERROR;
  }
  uint8_t frame[8] = {0};
  uint8_t nbytes   = (uint8_t)(hlen/2);
  for (uint8_t i = 0; i < nbytes; i++) {
    char tmp[3] = {hex_cmd[i*2], hex_cmd[i*2+1], 0};
    frame[i] = (uint8_t)strtol(tmp, nullptr, 16);
  }

  /* Prepend length byte then payload */
  uint8_t req[8] = {0};
  req[0] = nbytes;
  memcpy(&req[1], frame, nbytes);

  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) {
    strncpy(raw_resp, "CAN send failed", raw_len);
    return STATUS_ERROR;
  }

  CAN_FRAME rx;
  if (can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS) != STATUS_OK) {
    strncpy(raw_resp, "timeout", raw_len);
    return STATUS_TIMEOUT;
  }

  /* Format raw bytes */
  raw_resp[0] = 0;
  for (uint8_t i = 0; i < 8; i++) {
    char tmp[6];
    snprintf(tmp, sizeof(tmp), "%02X ", rx.data.byte[i]);
    strncat(raw_resp, tmp, raw_len - strlen(raw_resp) - 1);
  }

  /* Basic decode: if response mode = req_mode+0x40, decode known PIDs */
  uint8_t resp_mode = rx.data.byte[1];
  uint8_t req_mode  = frame[0];
  uint8_t req_pid   = (nbytes >= 2) ? frame[1] : 0xFF;

  if (resp_mode == (uint8_t)(req_mode + 0x40) && req_mode == OBD2_SVC_CURRENT) {
    uint8_t A = rx.data.byte[3];
    uint8_t B = rx.data.byte[4];
    float val = 0;
    bool  ok  = true;
    const char *unit = "";

    switch (req_pid) {
      case PID_RPM:         val = ((A*256.0f)+B)/4.0f;             unit="RPM";    break;
      case PID_SPEED:       val = A;                               unit="km/h";   break;
      case PID_COOLANT_TEMP:val = A-40;                            unit="°C";     break;
      case PID_IAT:         val = A-40;                            unit="°C";     break;
      case PID_THROTTLE:    val = A*100.0f/255.0f;                 unit="%";      break;
      case PID_ENGINE_LOAD: val = A*100.0f/255.0f;                 unit="%";      break;
      case PID_MAP:         val = A;                               unit="kPa";    break;
      case PID_OIL_TEMP:    val = A-40;                            unit="°C";     break;
      case PID_AMBIENT_TEMP:val = A-40;                            unit="°C";     break;
      case PID_O2_VOLTAGE:  val = A*0.005f;                        unit="V";      break;
      case PID_FUEL_PRESSURE:val= A*3.0f;                          unit="kPa";    break;
      case PID_STFT_B1:
      case PID_LTFT_B1:     val=(A/128.0f-1.0f)*100.0f;           unit="%";      break;
      case PID_IGN_ADVANCE: val=A/2.0f-64.0f;                      unit="°BTDC";  break;
      case PID_BAT_VOLTAGE: val=((A*256.0f)+B)/1000.0f;            unit="V";      break;
      case PID_RUNTIME:     val=((float)((A<<8)|B));               unit="s";      break;
      default:              ok=false;
    }
    if (ok)
      snprintf(decoded, dec_len, "%.3f %s", val, unit);
    else
      snprintf(decoded, dec_len, "A=0x%02X B=0x%02X C=0x%02X D=0x%02X",
               A, B, rx.data.byte[5], rx.data.byte[6]);
  } else {
    snprintf(decoded, dec_len, "raw: %02X %02X %02X %02X",
             rx.data.byte[0], rx.data.byte[1], rx.data.byte[2], rx.data.byte[3]);
  }

  return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II SERVICE LAYER — LIVE PARAMETERS (Mode 01)
   (unchanged formulae from original firmware)
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_get_rpm(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_RPM, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 2) return STATUS_ERROR;
  out->value = ((out->raw[0]*256.0f)+out->raw[1])/4.0f;
  return STATUS_OK;
}
obd2_status_t obd2_get_speed(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_SPEED, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0]; return STATUS_OK;
}
obd2_status_t obd2_get_coolant(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_COOLANT_TEMP, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0]-40.0f; return STATUS_OK;
}
obd2_status_t obd2_get_iat(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_IAT, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0]-40.0f; return STATUS_OK;
}
obd2_status_t obd2_get_throttle(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_THROTTLE, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = out->raw[0]*100.0f/255.0f; return STATUS_OK;
}
obd2_status_t obd2_get_runtime(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_RUNTIME, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 2) return STATUS_ERROR;
  out->value = (float)((out->raw[0]<<8)|out->raw[1]); return STATUS_OK;
}
obd2_status_t obd2_get_batt_voltage(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_BAT_VOLTAGE, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 2) return STATUS_ERROR;
  out->value = (float)((out->raw[0]<<8)|out->raw[1])/1000.0f; return STATUS_OK;
}
obd2_status_t obd2_get_ambient(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_AMBIENT_TEMP, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0]-40.0f; return STATUS_OK;
}
obd2_status_t obd2_get_engine_load(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_ENGINE_LOAD, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = out->raw[0]*100.0f/255.0f; return STATUS_OK;
}
obd2_status_t obd2_get_map(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_MAP, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0]; return STATUS_OK;
}
obd2_status_t obd2_get_o2_voltage(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_O2_VOLTAGE, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = out->raw[0]*0.005f; return STATUS_OK;
}
obd2_status_t obd2_get_fuel_pressure(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_FUEL_PRESSURE, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0]*3.0f; return STATUS_OK;
}
obd2_status_t obd2_get_stft_b1(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_STFT_B1, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = ((float)out->raw[0]/128.0f-1.0f)*100.0f; return STATUS_OK;
}
obd2_status_t obd2_get_ltft_b1(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_LTFT_B1, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = ((float)out->raw[0]/128.0f-1.0f)*100.0f; return STATUS_OK;
}
obd2_status_t obd2_get_ign_advance(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_IGN_ADVANCE, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0]/2.0f-64.0f; return STATUS_OK;
}
obd2_status_t obd2_get_oil_temp(obd2_result_t *out) {
  if (pid_request(OBD2_SVC_CURRENT, PID_OIL_TEMP, out) != STATUS_OK) return STATUS_ERROR;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0]-40.0f; return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II SERVICE LAYER — DTCs (Mode 03 / 04)
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_read_dtcs(obd2_dtc_result_t *out) {
  const uint8_t req[8] = {0x01, OBD2_SVC_DTC_READ, 0,0,0,0,0,0};
  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) return STATUS_ERROR;

  CAN_FRAME rx;
  if (can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS) != STATUS_OK) return STATUS_TIMEOUT;
  if (rx.data.byte[1] != (uint8_t)(OBD2_SVC_DTC_READ+0x40)) return STATUS_ERROR;

  out->count = 0;
  uint8_t num = rx.data.byte[2];
  for (uint8_t i = 0; i < num && out->count < OBD2_MAX_DTCS; i++) {
    uint8_t off = 3+(i*2);
    if (off+1 > 7) break;
    uint8_t hi = rx.data.byte[off];
    uint8_t lo = rx.data.byte[off+1];
    if (!hi && !lo) continue;
    snprintf(out->codes[out->count], 6, "%s%X%02X",
             dtc_prefix(hi>>6), (hi>>4)&0x03, ((hi&0x0F)<<4)|(lo>>4));
    char tmp[2] = {"0123456789ABCDEF"[lo&0x0F], '\0'};
    strncat(out->codes[out->count], tmp, 1);
    out->count++;
  }
  return STATUS_OK;
}

obd2_status_t obd2_clear_dtcs(void) {
  const uint8_t req[8] = {0x01, OBD2_SVC_DTC_CLR, 0,0,0,0,0,0};
  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) return STATUS_ERROR;
  CAN_FRAME rx;
  if (can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS) != STATUS_OK) return STATUS_TIMEOUT;
  if (rx.data.byte[1] != (uint8_t)(OBD2_SVC_DTC_CLR+0x40)) return STATUS_ERROR;
  Serial.println("[OBD] DTCs cleared");
  return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II SERVICE LAYER — VEHICLE INFO (Mode 09)
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_get_vin(char *buf, uint8_t buf_len) {
  uint8_t raw[50]={0}; uint8_t rlen=0;
  if (multiframe_request(OBD2_SVC_INFO, PID_VIN, raw, sizeof(raw), &rlen) != STATUS_OK) return STATUS_ERROR;
  uint8_t idx=0;
  for (uint8_t i=0; i<rlen && idx<buf_len-1; i++)
    if (raw[i]>=0x20 && raw[i]<=0x7E) buf[idx++]=(char)raw[i];
  buf[idx]='\0';
  return STATUS_OK;
}

obd2_status_t obd2_get_ecu_name(char *buf, uint8_t buf_len) {
  uint8_t raw[50]={0}; uint8_t rlen=0;
  if (multiframe_request(OBD2_SVC_INFO, PID_ECU_NAME, raw, sizeof(raw), &rlen) != STATUS_OK) return STATUS_ERROR;
  uint8_t idx=0;
  for (uint8_t i=0; i<rlen && idx<buf_len-1; i++)
    if (raw[i]>=0x20 && raw[i]<=0x7E) buf[idx++]=(char)raw[i];
  buf[idx]='\0';
  return STATUS_OK;
}
