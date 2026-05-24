#include <esp32_can.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════
   CAN CONFIGURATION
   ═══════════════════════════════════════════════════════════════ */
#define CAN_TX_PIN          GPIO_NUM_5
#define CAN_RX_PIN          GPIO_NUM_4
#define CAN_SPEED           500000UL
#define CAN_OBD2_REQ_ID     0x7DF
#define CAN_OBD2_RESP_ID    0x7E8
#define CAN_READ_TIMEOUT_MS 100

/* ISO-TP frame type nibbles (upper nibble of byte[0]) */
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
#define PID_STFT_B1       0x06   /* Short-term fuel trim bank 1      */
#define PID_LTFT_B1       0x07   /* Long-term  fuel trim bank 1      */
#define PID_FUEL_PRESSURE 0x0A   /* Fuel rail pressure (gauge)       */
#define PID_MAP           0x0B   /* Manifold Absolute Pressure       */
#define PID_RPM           0x0C
#define PID_SPEED         0x0D
#define PID_IGN_ADVANCE   0x0E   /* Ignition timing advance          */
#define PID_IAT           0x0F   /* Intake air temperature           */
#define PID_THROTTLE      0x11
#define PID_O2_VOLTAGE    0x14   /* O2 sensor bank1 sensor1 voltage  */
#define PID_RUNTIME       0x1F
#define PID_BAT_VOLTAGE   0x42
#define PID_AMBIENT_TEMP  0x46
#define PID_OIL_TEMP      0x5C   /* Engine oil temperature           */

/* OBD-II PIDs — Mode 09 */
#define PID_VIN           0x02
#define PID_ECU_NAME      0x0A

/* ═══════════════════════════════════════════════════════════════
   BLE CONFIGURATION
   ═══════════════════════════════════════════════════════════════ */
#define BLE_DEVICE_NAME    "ESP32-OBD2-Bike"
#define BLE_SVC_UUID       "0000FFE0-0000-1000-8000-00805F9B34FB"

/* NOTIFY+READ UUIDs */
#define UUID_RPM           "0000FF01-0000-1000-8000-00805F9B34FB"
#define UUID_SPEED         "0000FF02-0000-1000-8000-00805F9B34FB"
#define UUID_COOLANT       "0000FF03-0000-1000-8000-00805F9B34FB"
#define UUID_IAT           "0000FF04-0000-1000-8000-00805F9B34FB"
#define UUID_THROTTLE      "0000FF05-0000-1000-8000-00805F9B34FB"
#define UUID_RUNTIME       "0000FF06-0000-1000-8000-00805F9B34FB"
#define UUID_BATT_VOLT     "0000FF07-0000-1000-8000-00805F9B34FB"
#define UUID_AMBIENT       "0000FF08-0000-1000-8000-00805F9B34FB"
#define UUID_ENG_LOAD      "0000FF09-0000-1000-8000-00805F9B34FB"
#define UUID_MAP           "0000FF0A-0000-1000-8000-00805F9B34FB"
#define UUID_O2_VOLT       "0000FF0B-0000-1000-8000-00805F9B34FB"
#define UUID_FUEL_PRES     "0000FF0C-0000-1000-8000-00805F9B34FB"
#define UUID_STFT_B1       "0000FF0D-0000-1000-8000-00805F9B34FB"
#define UUID_LTFT_B1       "0000FF0E-0000-1000-8000-00805F9B34FB"
#define UUID_IGN_ADVANCE   "0000FF0F-0000-1000-8000-00805F9B34FB"
#define UUID_OIL_TEMP      "0000FF10-0000-1000-8000-00805F9B34FB"

/* READ-ONLY UUIDs */
#define UUID_DTCS          "0000FF11-0000-1000-8000-00805F9B34FB"
#define UUID_VIN           "0000FF12-0000-1000-8000-00805F9B34FB"
#define UUID_ECU_NAME      "0000FF13-0000-1000-8000-00805F9B34FB"
#define UUID_CLR_DTCS      "0000FF14-0000-1000-8000-00805F9B34FB"

/* Number of NOTIFY+READ characteristics (= number of bits in sub mask) */
#define NOTIFY_PARAM_COUNT 16
#define BLE_NOTIFY_MAX_LEN 20   /* bytes — standard BLE notify payload limit */

/* ═══════════════════════════════════════════════════════════════
   EEPROM LAYOUT
     addr 0-3  uint32_t  subscription bitmask (NOTIFY params)
     addr 4    uint8_t   first-boot sentinel
   ═══════════════════════════════════════════════════════════════ */
#define EEPROM_SIZE        8
#define EEPROM_ADDR_SUBS   0
#define EEPROM_MAGIC_ADDR  4
#define EEPROM_MAGIC_VAL   0xA5

/* ═══════════════════════════════════════════════════════════════
   PARAMETER INDEX ENUM
   Each value = bit position in g_sub_mask.
   Must stay in sync with the MAKE_NR calls in ble_create_service().
   ═══════════════════════════════════════════════════════════════ */
typedef enum
{
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
  PARAM_OIL_TEMP    = 15
} param_index_t;

/* ═══════════════════════════════════════════════════════════════
   STATUS ENUM
   ═══════════════════════════════════════════════════════════════ */
typedef enum
{
  STATUS_ERROR   = -1,
  STATUS_OK      =  0,
  STATUS_TIMEOUT =  2
} obd2_status_t;

/* ═══════════════════════════════════════════════════════════════
   DATA TYPES
   ═══════════════════════════════════════════════════════════════ */
typedef struct
{
  float   value;
  uint8_t raw[20];
  uint8_t raw_len;
} obd2_result_t;

#define OBD2_MAX_DTCS 10
typedef struct
{
  char    codes[OBD2_MAX_DTCS][6];   /* e.g. "P0301\0" */
  uint8_t count;
} obd2_dtc_result_t;

/* ═══════════════════════════════════════════════════════════════
   GLOBAL BLE STATE
   ═══════════════════════════════════════════════════════════════ */
BLEServer  *g_server    = nullptr;
BLEService *g_service   = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

BLECharacteristic *g_char[NOTIFY_PARAM_COUNT]; /* NOTIFY+READ array */
BLECharacteristic *g_char_dtcs     = nullptr;  /* READ-ONLY: DTC list  */
BLECharacteristic *g_char_vin      = nullptr;  /* READ-ONLY: VIN       */
BLECharacteristic *g_char_ecu_name = nullptr;  /* READ-ONLY: ECU name  */
BLECharacteristic *g_char_clr_dtcs = nullptr;  /* READ-ONLY: clear DTCs*/

/* Subscription bitmask — bit N set ⟹ PARAM_N is notify-subscribed.
   Written from BLE stack task; read from loop() task.
   Protected by a spinlock for atomic 32-bit access on Xtensa.       */
volatile uint32_t   g_sub_mask = 0;
static portMUX_TYPE g_mask_mux = portMUX_INITIALIZER_UNLOCKED;

/* ═══════════════════════════════════════════════════════════════
   FORWARD DECLARATIONS
   ═══════════════════════════════════════════════════════════════ */
void          can_init(void);
bool          can_send(uint32_t id, const uint8_t *data, uint8_t len);
obd2_status_t can_recv(uint32_t exp_id, CAN_FRAME *out, uint32_t timeout_ms);

void ble_init(void);
void ble_create_service(void);
void ble_start_advertising(void);
void ble_set_value(param_index_t idx, const char *str);
void ble_notify(param_index_t idx);

void     eeprom_init(void);
void     eeprom_save(uint32_t mask);

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

static obd2_status_t pid_request(uint8_t mode, uint8_t pid, obd2_result_t *out);
static obd2_status_t multiframe_request(uint8_t mode, uint8_t pid,
                                        uint8_t *out_buf, uint8_t buf_len,
                                        uint8_t *out_len);
static const char *dtc_prefix(uint8_t top2);

/* ═══════════════════════════════════════════════════════════════
   BLE CALLBACK — server connect / disconnect
   ═══════════════════════════════════════════════════════════════ */
class ServerCB : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer) override
    {
      deviceConnected = true;

      // Optional but recommended
      BLEDevice::getAdvertising()->stop();
    }

    void onDisconnect(BLEServer *pServer) override
    {
      deviceConnected = false;
    }
};

/* ═══════════════════════════════════════════════════════════════
   BLE CALLBACK — CCCD descriptor write (enable / disable notify)

   IMPORTANT: this must be attached to the BLE2902 DESCRIPTOR, not
   to the characteristic. BLECharacteristicCallbacks::onWrite fires
   on characteristic-value writes, not on CCCD writes.
   ═══════════════════════════════════════════════════════════════ */
class CCCDCallback : public BLEDescriptorCallbacks
{
  public:
    explicit CCCDCallback(param_index_t idx) : m_idx(idx) {}

    void onWrite(BLEDescriptor *pDesc) override
    {
      uint8_t *data = pDesc->getValue();
      uint16_t cccd = 0;
      if (pDesc->getLength() >= 2)
        cccd = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));

      bool en = (cccd == 0x0001);

      portENTER_CRITICAL(&g_mask_mux);
      if (en) g_sub_mask |=  (1UL << m_idx);
      else    g_sub_mask &= ~(1UL << m_idx);
      uint32_t snap = g_sub_mask;
      portEXIT_CRITICAL(&g_mask_mux);

      Serial.printf("BLE: param %2d %s  mask=0x%08X\n",
                    (int)m_idx,
                    en ? "SUBSCRIBED  " : "UNSUBSCRIBED",
                    (unsigned)snap);
      eeprom_save(snap);
    }

  private:
    param_index_t m_idx;
};

/* ═══════════════════════════════════════════════════════════════
   BLE CALLBACK — READ-ONLY characteristic read

   Called by the BLE stack the moment the client issues a Read.
   We query the ECU here, set the value, and the stack then sends it.
   Tag selects which ECU query to perform:
     'D' = read DTCs
     'V' = read VIN
     'E' = read ECU name
     'C' = clear DTCs
   ═══════════════════════════════════════════════════════════════ */
class ReadOnlyCB : public BLECharacteristicCallbacks
{
  public:
    explicit ReadOnlyCB(char tag) : m_tag(tag) {}

    void onRead(BLECharacteristic *pChar) override
    {
      char buf[64] = {0};

      if (m_tag == 'D')
      {
        obd2_dtc_result_t dtc = {};
        if (obd2_read_dtcs(&dtc) == STATUS_OK)
        {
          if (dtc.count == 0)
          {
            strncpy(buf, "NONE", sizeof(buf));
          }
          else
          {
            for (uint8_t i = 0; i < dtc.count; i++)
            {
              if (i > 0) strncat(buf, ",", sizeof(buf) - strlen(buf) - 1);
              strncat(buf, dtc.codes[i], sizeof(buf) - strlen(buf) - 1);
            }
          }
        }
        else strncpy(buf, "error", sizeof(buf));

        Serial.printf("BLE read DTCs   : %s\n", buf);
      }
      else if (m_tag == 'V')
      {
        if (obd2_get_vin(buf, sizeof(buf)) != STATUS_OK)
          strncpy(buf, "error", sizeof(buf));
        Serial.printf("BLE read VIN    : %s\n", buf);
      }
      else if (m_tag == 'E')
      {
        if (obd2_get_ecu_name(buf, sizeof(buf)) != STATUS_OK)
          strncpy(buf, "error", sizeof(buf));
        Serial.printf("BLE read ECU    : %s\n", buf);
      }
      else if (m_tag == 'C')
      {
        strncpy(buf, (obd2_clear_dtcs() == STATUS_OK) ? "OK" : "error",
                sizeof(buf));
        Serial.printf("BLE clear DTCs  : %s\n", buf);
      }

      size_t len = strlen(buf);
      if (len > BLE_NOTIFY_MAX_LEN) len = BLE_NOTIFY_MAX_LEN;
      pChar->setValue((uint8_t *)buf, (uint8_t)len);
    }

  private:
    char m_tag;
};

/* ═══════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════ */
void setup()
{
  Serial.begin(115200);
  Serial.println("============================================");
  Serial.println("  ESP32 OBD-II BLE — Bike Edition");
  Serial.println("============================================");

  eeprom_init();
  can_init();
  ble_init();
  ble_create_service();
  ble_start_advertising();

  Serial.println("Ready — waiting for BLE client...");
}

/* ═══════════════════════════════════════════════════════════════
   LOOP — stream subscribed live parameters every 1 s
   All values are ASCII strings: numeric on success, "error" on fail.
   ═══════════════════════════════════════════════════════════════ */
void loop()
{
if (deviceConnected) 
{
  portENTER_CRITICAL(&g_mask_mux);
  uint32_t active = g_sub_mask;
  portEXIT_CRITICAL(&g_mask_mux);

  if (active == 0) {
    vTaskDelay(pdMS_TO_TICKS(200));
    return;
  }

  obd2_result_t res;
  char          str[16];

  /*  POLL macro:
        - queries ECU via GETTER
        - formats float as FMT string on success, or copies "error"
        - writes value into characteristic and sends notification     */
#define POLL(PARAM, FMT, GETTER)                              \
  if (active & (1UL << (PARAM))) {                            \
    if ((GETTER)(&res) == STATUS_OK)                          \
      snprintf(str, sizeof(str), (FMT), res.value);           \
    else                                                      \
      strncpy(str, "error", sizeof(str));                     \
    ble_set_value((PARAM), str);                              \
    ble_notify((PARAM));                                      \
    Serial.printf("%-18s: %s\n", #PARAM, str);                \
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

  Serial.println("--------------------------------------------");
  vTaskDelay(pdMS_TO_TICKS(500));
}
  if (!deviceConnected && oldDeviceConnected) {
    portENTER_CRITICAL(&g_mask_mux);
      g_sub_mask = 0;
      portEXIT_CRITICAL(&g_mask_mux);

      Serial.println("BLE: disconnected — restarting advertising");

      delay(200);  // give stack time to cleanup

      BLEAdvertising *adv = BLEDevice::getAdvertising();
      adv->stop();
      adv->addServiceUUID(BLE_SVC_UUID);
      adv->setScanResponse(true);
      adv->setMinPreferred(0x06);
      adv->setMaxPreferred(0x12);
      adv->start();
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

/* ═══════════════════════════════════════════════════════════════
   CAN LAYER
   ═══════════════════════════════════════════════════════════════ */
void can_init(void)
{
  CAN0.setCANPins(CAN_RX_PIN, CAN_TX_PIN);
  CAN0.begin(CAN_SPEED);
  CAN0.watchFor(CAN_OBD2_RESP_ID);
  Serial.println("CAN: initialised at 500 kbps");
}

bool can_send(uint32_t id, const uint8_t *data, uint8_t len)
{
  CAN_FRAME tx;
  tx.id       = id;
  tx.extended = false;
  tx.rtr      = 0;
  tx.length   = (len > 8) ? 8 : len;
  for (uint8_t i = 0; i < tx.length; i++) tx.data.byte[i] = data[i];
  return CAN0.sendFrame(tx);
}

obd2_status_t can_recv(uint32_t exp_id, CAN_FRAME *out, uint32_t timeout_ms)
{
  uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline)
  {
    if (CAN0.read(*out))
      if ((exp_id == 0) || (out->id == exp_id))
        return STATUS_OK;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return STATUS_TIMEOUT;
}

/* ═══════════════════════════════════════════════════════════════
   BLE LAYER
   ═══════════════════════════════════════════════════════════════ */
void ble_init(void)
{
  BLEDevice::init(BLE_DEVICE_NAME);

  // Optional stability improvement
  BLEDevice::setMTU(185);

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());

  Serial.printf("BLE: device \"%s\" created\n", BLE_DEVICE_NAME);
}

void ble_create_service(void)
{
  /*  Handle budget:
        1   service declaration
       16   NOTIFY+READ chars × 3 handles (decl + value + CCCD) = 48
        4   READ-ONLY chars   × 2 handles (decl + value)        =  8
      Total = 57 → allocate 64.                                        */
  g_service = g_server->createService(BLEUUID(BLE_SVC_UUID), 64);

  /* ── NOTIFY + READ characteristics ──────────────────────────────
     The BLE2902 CCCD descriptor carries the CCCDCallback which fires
     when the client enables/disables notifications.
     PROPERTY_READ lets the client do a one-off read of the last
     cached value without needing an active subscription.            */
#define MAKE_NR(UUID_STR, PARAM_IDX)                                           \
  do {                                                                          \
    BLECharacteristic *ch = g_service->createCharacteristic(                   \
                            BLEUUID(UUID_STR),                                                      \
                            BLECharacteristic::PROPERTY_NOTIFY |                                   \
                            BLECharacteristic::PROPERTY_READ);                                     \
    BLE2902 *cccd = new BLE2902();                                              \
    cccd->setNotifications(false);                                              \
    cccd->setCallbacks(new CCCDCallback((param_index_t)(PARAM_IDX)));          \
    ch->addDescriptor(cccd);                                                    \
    ch->setValue("--");     /* placeholder until first ECU poll */             \
    g_char[(PARAM_IDX)] = ch;                                                   \
  } while (0)

  MAKE_NR(UUID_RPM,         PARAM_RPM);
  MAKE_NR(UUID_SPEED,       PARAM_SPEED);
  MAKE_NR(UUID_COOLANT,     PARAM_COOLANT);
  MAKE_NR(UUID_IAT,         PARAM_IAT);
  MAKE_NR(UUID_THROTTLE,    PARAM_THROTTLE);
  MAKE_NR(UUID_RUNTIME,     PARAM_RUNTIME);
  MAKE_NR(UUID_BATT_VOLT,   PARAM_BATT_VOLT);
  MAKE_NR(UUID_AMBIENT,     PARAM_AMBIENT);
  MAKE_NR(UUID_ENG_LOAD,    PARAM_ENG_LOAD);
  MAKE_NR(UUID_MAP,         PARAM_MAP);
  MAKE_NR(UUID_O2_VOLT,     PARAM_O2_VOLT);
  MAKE_NR(UUID_FUEL_PRES,   PARAM_FUEL_PRES);
  MAKE_NR(UUID_STFT_B1,     PARAM_STFT_B1);
  MAKE_NR(UUID_LTFT_B1,     PARAM_LTFT_B1);
  MAKE_NR(UUID_IGN_ADVANCE, PARAM_IGN_ADVANCE);
  MAKE_NR(UUID_OIL_TEMP,    PARAM_OIL_TEMP);

#undef MAKE_NR

  /* ── READ-ONLY characteristics ───────────────────────────────────
     No NOTIFY / no CCCD. ReadOnlyCB::onRead() fires when the client
     reads the characteristic; the handler queries the ECU live.     */
#define MAKE_RO(UUID_STR, PTR, TAG)                                            \
  do {                                                                          \
    BLECharacteristic *ch = g_service->createCharacteristic(                   \
                            BLEUUID(UUID_STR),                                                      \
                            BLECharacteristic::PROPERTY_READ);                                     \
    ch->setCallbacks(new ReadOnlyCB(TAG));                                     \
    ch->setValue("--");                                                         \
    (PTR) = ch;                                                                 \
  } while (0)

  MAKE_RO(UUID_DTCS,     g_char_dtcs,     'D');
  MAKE_RO(UUID_VIN,      g_char_vin,      'V');
  MAKE_RO(UUID_ECU_NAME, g_char_ecu_name, 'E');
  MAKE_RO(UUID_CLR_DTCS, g_char_clr_dtcs, 'C');

#undef MAKE_RO

  g_service->start();
  Serial.println("BLE: service started (16 live + 4 read-only characteristics)");
}

void ble_start_advertising(void)
{
  BLEAdvertising *adv = BLEDevice::getAdvertising();

  adv->stop();  // ensure clean state

  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);

  adv->start();   // ✅ correct way

  Serial.println("BLE: advertising started");
}

/* Write an ASCII string value into a NOTIFY+READ characteristic */
void ble_set_value(param_index_t idx, const char *str)
{
  if (idx >= NOTIFY_PARAM_COUNT) return;
  size_t len = strlen(str);
  if (len > BLE_NOTIFY_MAX_LEN) len = BLE_NOTIFY_MAX_LEN;
  g_char[idx]->setValue((uint8_t *)str, (uint8_t)len);
}

/* Send a BLE notification for a NOTIFY+READ characteristic */
void ble_notify(param_index_t idx)
{
  if (!deviceConnected || idx >= NOTIFY_PARAM_COUNT) return;
  g_char[idx]->notify();
}

/* ═══════════════════════════════════════════════════════════════
   EEPROM LAYER
   ═══════════════════════════════════════════════════════════════ */
void eeprom_init(void)
{
  EEPROM.begin(EEPROM_SIZE);

  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL)
  {
    /* First boot: write sentinel and zero mask */
    EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
    EEPROM.put(EEPROM_ADDR_SUBS, (uint32_t)0);
    EEPROM.commit();
    g_sub_mask = 0;
    Serial.println("EEPROM: first boot, mask cleared");
  }
  else
  {
    uint32_t saved = 0;
    EEPROM.get(EEPROM_ADDR_SUBS, saved);
    portENTER_CRITICAL(&g_mask_mux);
    g_sub_mask = saved;
    portEXIT_CRITICAL(&g_mask_mux);
    Serial.printf("EEPROM: restored mask = 0x%08X\n", (unsigned)saved);
  }
}

void eeprom_save(uint32_t mask)
{
  uint32_t current = 0;
  EEPROM.get(EEPROM_ADDR_SUBS, current);
  if (current != mask)
  {
    EEPROM.put(EEPROM_ADDR_SUBS, mask);
    EEPROM.commit();
    Serial.printf("EEPROM: saved mask = 0x%08X\n", (unsigned)mask);
  }
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II INTERNAL HELPERS
   ═══════════════════════════════════════════════════════════════ */

/*  Single-frame PID request (Mode 01).
    Request frame:
      byte[0] = 0x02  (2 payload bytes follow)
      byte[1] = mode
      byte[2] = PID
      byte[3-7] = 0x00 padding

    Positive response frame (0x7E8):
      byte[0] = DLC
      byte[1] = mode + 0x40
      byte[2] = PID
      byte[3..] = data bytes                                          */
static obd2_status_t pid_request(uint8_t mode, uint8_t pid, obd2_result_t *out)
{
  const uint8_t req[8] = {0x02, mode, pid, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) return STATUS_ERROR;

  CAN_FRAME rx;
  obd2_status_t st = can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK) return st;

  if (rx.data.byte[1] != (uint8_t)(mode + 0x40) ||
      rx.data.byte[2] != pid)
    return STATUS_ERROR;

  uint8_t dlc = rx.data.byte[0];
  if (dlc < 2) return STATUS_ERROR;

  out->raw_len = (dlc - 2 < (uint8_t)sizeof(out->raw))
                 ? (dlc - 2) : (uint8_t)sizeof(out->raw);
  for (uint8_t i = 0; i < out->raw_len; i++)
    out->raw[i] = rx.data.byte[3 + i];

  return STATUS_OK;
}

/*  Multi-frame ISO-TP exchange (Mode 09 — VIN, ECU name).

    First Frame (FF) payload layout for a Mode-09 response:
      byte[0] = 0x1X  (FF marker; upper nibble of total length)
      byte[1] = lower byte of total payload length
      byte[2] = service byte (mode + 0x40 = 0x49)
      byte[3] = InfoType / PID
      byte[4] = count of data items (e.g. 1 for VIN)
      byte[5-7] = first 3 bytes of actual string data

    Consecutive Frame (CF) layout:
      byte[0] = 0x2N  (CF marker + 4-bit sequence number)
      byte[1-7] = 7 bytes of string data                             */
static obd2_status_t multiframe_request(uint8_t mode, uint8_t pid,
                                        uint8_t *out_buf, uint8_t buf_len,
                                        uint8_t *out_len)
{
  const uint8_t req[8] = {0x02, mode, pid, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) return STATUS_ERROR;

  CAN_FRAME rx;
  obd2_status_t st = can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK) return st;

  uint8_t ft = rx.data.byte[0] & 0xF0;

  /* ── Single frame ── */
  if (ft == ISOTP_SINGLE)
  {
    uint8_t dlc = rx.data.byte[0] & 0x0F;
    if (dlc < 4) {
      out_buf[0] = '\0';
      *out_len = 0;
      return STATUS_OK;
    }
    uint8_t dlen = dlc - 3;                  /* skip svc + pid + count */
    if (dlen > buf_len - 1) dlen = buf_len - 1;
    memcpy(out_buf, &rx.data.byte[4], dlen);
    out_buf[dlen] = '\0';
    *out_len = dlen;
    return STATUS_OK;
  }

  if (ft != ISOTP_FIRST) return STATUS_ERROR;

  /* ── First frame ── */
  uint16_t total_payload = (uint16_t)(((rx.data.byte[0] & 0x0F) << 8) |
                                      rx.data.byte[1]);
  /* Subtract the 3-byte header (svc + pid + count) to get pure data length */
  uint16_t data_total = (total_payload > 3) ? (total_payload - 3) : 0;

  /* Copy the first 3 data bytes from FF bytes 5-7 */
  uint8_t idx = 0;
  for (uint8_t i = 5; i <= 7 && idx < buf_len - 1; i++)
    out_buf[idx++] = rx.data.byte[i];

  /* Flow Control — tell ECU to send all remaining CFs immediately */
  const uint8_t fc[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (!can_send(CAN_OBD2_REQ_ID, fc, 8)) return STATUS_ERROR;

  /* Consecutive frames */
  uint8_t seq = 1;
  while (idx < data_total && idx < buf_len - 1)
  {
    st = can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
    if (st != STATUS_OK) return st;
    if ((rx.data.byte[0] & 0xF0) != ISOTP_CONSEC) return STATUS_ERROR;
    if ((rx.data.byte[0] & 0x0F) != (seq & 0x0F))  return STATUS_ERROR;
    for (uint8_t i = 1; i <= 7 && idx < buf_len - 1; i++)
      out_buf[idx++] = rx.data.byte[i];
    seq++;
  }

  out_buf[idx] = '\0';
  *out_len = idx;
  return STATUS_OK;
}

/* Top-2 bits of DTC high byte → letter prefix */
static const char *dtc_prefix(uint8_t top2)
{
  switch (top2 & 0x03)
  {
    case 0: return "P";
    case 1: return "C";
    case 2: return "B";
    case 3: return "U";
  }
  return "?";
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II SERVICE LAYER — LIVE PARAMETERS (Mode 01)
   ═══════════════════════════════════════════════════════════════ */

obd2_status_t obd2_get_rpm(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_RPM, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 2) return STATUS_ERROR;
  /* Formula: (256*A + B) / 4   → RPM */
  out->value = ((out->raw[0] * 256.0f) + out->raw[1]) / 4.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_speed(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_SPEED, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: A   → km/h */
  out->value = (float)out->raw[0];
  return STATUS_OK;
}

obd2_status_t obd2_get_coolant(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_COOLANT_TEMP, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: A - 40   → °C */
  out->value = (float)out->raw[0] - 40.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_iat(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_IAT, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0] - 40.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_throttle(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_THROTTLE, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: A * 100 / 255   → % */
  out->value = out->raw[0] * 100.0f / 255.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_runtime(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_RUNTIME, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 2) return STATUS_ERROR;
  /* Formula: 256*A + B   → seconds */
  out->value = (float)((out->raw[0] << 8) | out->raw[1]);
  return STATUS_OK;
}

obd2_status_t obd2_get_batt_voltage(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_BAT_VOLTAGE, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 2) return STATUS_ERROR;
  /* Formula: (256*A + B) / 1000   → V */
  out->value = (float)((out->raw[0] << 8) | out->raw[1]) / 1000.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_ambient(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_AMBIENT_TEMP, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = (float)out->raw[0] - 40.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_engine_load(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_ENGINE_LOAD, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = out->raw[0] * 100.0f / 255.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_map(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_MAP, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: A   → kPa (0-255) */
  out->value = (float)out->raw[0];
  return STATUS_OK;
}

obd2_status_t obd2_get_o2_voltage(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_O2_VOLTAGE, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: A * 0.005   → V (byte B = short fuel trim, ignored) */
  out->value = out->raw[0] * 0.005f;
  return STATUS_OK;
}

obd2_status_t obd2_get_fuel_pressure(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_FUEL_PRESSURE, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: A * 3   → kPa */
  out->value = (float)out->raw[0] * 3.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_stft_b1(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_STFT_B1, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: (A/128 - 1) * 100   → % (negative = lean trim) */
  out->value = ((float)out->raw[0] / 128.0f - 1.0f) * 100.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_ltft_b1(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_LTFT_B1, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  out->value = ((float)out->raw[0] / 128.0f - 1.0f) * 100.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_ign_advance(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_IGN_ADVANCE, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: A/2 - 64   → degrees BTDC */
  out->value = (float)out->raw[0] / 2.0f - 64.0f;
  return STATUS_OK;
}

obd2_status_t obd2_get_oil_temp(obd2_result_t *out)
{
  obd2_status_t st = pid_request(OBD2_SVC_CURRENT, PID_OIL_TEMP, out);
  if (st != STATUS_OK) return st;
  if (out->raw_len < 1) return STATUS_ERROR;
  /* Formula: A - 40   → °C */
  out->value = (float)out->raw[0] - 40.0f;
  return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II SERVICE LAYER — DTCs (Mode 03 / 04)

   Mode-03 positive response layout:
     byte[0] = DLC
     byte[1] = 0x43  (0x03 + 0x40)
     byte[2] = number of DTCs
     byte[3], byte[4] = 1st DTC high/low
     byte[5], byte[6] = 2nd DTC high/low   (max 2 per single CAN frame)
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_read_dtcs(obd2_dtc_result_t *out)
{
  const uint8_t req[8] = {0x01, OBD2_SVC_DTC_READ,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                         };
  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) return STATUS_ERROR;

  CAN_FRAME rx;
  obd2_status_t st = can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK) return st;
  if (rx.data.byte[1] != (uint8_t)(OBD2_SVC_DTC_READ + 0x40)) return STATUS_ERROR;

  out->count = 0;
  uint8_t num_dtcs = rx.data.byte[2];

  for (uint8_t i = 0; i < num_dtcs && out->count < OBD2_MAX_DTCS; i++)
  {
    uint8_t off = 3 + (i * 2);
    if (off + 1 > 7) break;               /* CAN frame boundary guard */

    uint8_t hi = rx.data.byte[off];
    uint8_t lo = rx.data.byte[off + 1];
    if (hi == 0 && lo == 0) continue;     /* null padding — skip       */

    snprintf(out->codes[out->count], 6, "%s%X%02X",
             dtc_prefix(hi >> 6),
             (hi >> 4) & 0x03,
             ((hi & 0x0F) << 4) | (lo >> 4));
    char tmp[2] = {"0123456789ABCDEF"[lo & 0x0F], '\0'};
    strncat(out->codes[out->count], tmp, 1);
    out->count++;
  }
  return STATUS_OK;
}

obd2_status_t obd2_clear_dtcs(void)
{
  const uint8_t req[8] = {0x01, OBD2_SVC_DTC_CLR,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                         };
  if (!can_send(CAN_OBD2_REQ_ID, req, 8)) return STATUS_ERROR;

  CAN_FRAME rx;
  obd2_status_t st = can_recv(CAN_OBD2_RESP_ID, &rx, CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK) return st;
  if (rx.data.byte[1] != (uint8_t)(OBD2_SVC_DTC_CLR + 0x40)) return STATUS_ERROR;

  Serial.println("OBD2: DTCs cleared");
  return STATUS_OK;
}

/* ═══════════════════════════════════════════════════════════════
   OBD-II SERVICE LAYER — VEHICLE INFO (Mode 09)
   ═══════════════════════════════════════════════════════════════ */
obd2_status_t obd2_get_vin(char *buf, uint8_t buf_len)
{
  uint8_t raw[50] = {0};
  uint8_t rlen    = 0;
  obd2_status_t st = multiframe_request(OBD2_SVC_INFO, PID_VIN,
                                        raw, sizeof(raw), &rlen);
  if (st != STATUS_OK) return st;

  uint8_t idx = 0;
  for (uint8_t i = 0; i < rlen && idx < buf_len - 1; i++)
    if (raw[i] >= 0x20 && raw[i] <= 0x7E)
      buf[idx++] = (char)raw[i];
  buf[idx] = '\0';
  return STATUS_OK;
}

obd2_status_t obd2_get_ecu_name(char *buf, uint8_t buf_len)
{
  uint8_t raw[50] = {0};
  uint8_t rlen    = 0;
  obd2_status_t st = multiframe_request(OBD2_SVC_INFO, PID_ECU_NAME,
                                        raw, sizeof(raw), &rlen);
  if (st != STATUS_OK) return st;

  uint8_t idx = 0;
  for (uint8_t i = 0; i < rlen && idx < buf_len - 1; i++)
    if (raw[i] >= 0x20 && raw[i] <= 0x7E)
      buf[idx++] = (char)raw[i];
  buf[idx] = '\0';
  return STATUS_OK;
}
