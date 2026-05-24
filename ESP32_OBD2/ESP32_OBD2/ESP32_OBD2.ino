/*
  ESP32        TJA1050
  -------      ---------
  GPIO5      →   TXD
  GPIO4      ←   RXD
  GND        →   GND
  3.3V       →   VCC
*/

/*
====== Engine OFF ======
RPM           : 1251
Speed (km/h)  : 0
Coolant (B0C)  : 31
IAT (B0C)      : 32
Throttle (%)  : 11.8
Runtime (s)   : 0
Battery (V)   : 10.94
Eng.Load (%)  : 30.2
O2 Volt (V)   : 0.415
DTCs Found    : 0
VIN           : ME3J3D5FDR1011604
ECU Name      : ECM-EngineControl
========================

====== Engine ON ======
RPM           : 1124
Speed (km/h)  : 0
Coolant (B0C)  : 32
IAT (B0C)      : 36
Throttle (%)  : 11.8
Runtime (s)   : 24
Battery (V)   : 12.24
Eng.Load (%)  : 30.6
O2 Volt (V)   : 0.210
DTCs Found    : 0
VIN           : ME3J3D5FDR1011604
ECU Name      : ECM-EngineControl
=======================
*/

#include <esp32_can.h>
#include <string.h>

/* ============================================================
    CONFIGURATION
   ============================================================ */
#define INTF_CAN_TX_PIN GPIO_NUM_5
#define INTF_CAN_RX_PIN GPIO_NUM_4
#define INTF_CAN_SPEED 500000UL

#define INTF_CAN_OBD2_REQ_ID 0x7DF    /* Broadcast functional request   */
#define INTF_CAN_OBD2_RESP_ID 0x7E8   /* ECU response ID                 */
#define INTF_CAN_OBD2_RESP_MASK 0x7E8 /* watch-filter mask               */

#define INTF_CAN_READ_TIMEOUT_MS 10 /* Max wait for a CAN frame (ms)   */

/* ISO-TP frame types (for multi-frame VIN / DTC reads) */
#define ISOTP_SINGLE_FRAME 0x00
#define ISOTP_FIRST_FRAME 0x10
#define ISOTP_CONSEC_FRAME 0x20
#define ISOTP_FLOW_CONTROL 0x30

/* OBD-II Service modes */
#define OBD2_MODE_CURRENT_DATA 0x01
#define OBD2_MODE_FREEZE_FRAME 0x02
#define OBD2_MODE_DTC_READ 0x03
#define OBD2_MODE_DTC_CLEAR 0x04
#define OBD2_MODE_VIN 0x09

/* OBD-II PIDs */
#define OBD2_PID_ENGINE_LOAD 0x04
#define OBD2_PID_COOLANT_TEMP 0x05
#define OBD2_PID_FUEL_PRESSURE 0x0A
#define OBD2_PID_THROTTLE 0x11
#define OBD2_PID_O2_VOLTAGE 0x14
#define OBD2_PID_RUNTIME 0x1F
#define OBD2_PID_MAF 0x10
#define OBD2_PID_RPM 0x0C
#define OBD2_PID_SPEED 0x0D
#define OBD2_PID_IAT 0x0F
#define OBD2_PID_FUEL_LEVEL 0x2F
#define OBD2_PID_AMBIENT_TEMP 0x46
#define OBD2_PID_BAT_VOLTAGE 0x42
#define OBD2_PID_VIN 0x02
#define OBD2_PID_ECU_NAME 0x0A

/* ============================================================
    SHARED STATUS ENUM
   ============================================================ */
typedef enum
{
  STATUS_ERROR = -1, /*!< Execution failed.            */
  STATUS_OK = 0,     /*!< Execution succeeded.         */
  STATUS_BUSY = 1,   /*!< Busy / not ready.            */
  STATUS_TIMEOUT = 2 /*!< No response within deadline. */
} obd2_status_t;

/* ============================================================
    DATA TYPES  (service layer outputs)
   ============================================================ */

/* Generic decoded result carrying up to 20 bytes of raw payload
   plus the final scaled floating-point value for numeric PIDs. */
typedef struct
{
  float value;     /* Scaled engineering value            */
  uint8_t raw[20]; /* Raw payload bytes (for strings/DTCs) */
  uint8_t raw_len; /* Number of valid bytes in raw[]       */
} obd2_result_t;

/* DTC storage — up to 10 fault codes per read */
#define OBD2_MAX_DTCS 10
typedef struct
{
  char codes[OBD2_MAX_DTCS][6]; /* e.g. "P0301\0"             */
  uint8_t count;                /* number of DTCs decoded      */
} obd2_dtc_result_t;

/* ============================================================
    INTERFACE LAYER — DECLARATIONS
   ============================================================ */
void intf_can_init(void);
bool intf_can_sendFrame(uint32_t id, const uint8_t *data, uint8_t length);
obd2_status_t intf_can_readFrame(uint32_t expected_id, CAN_FRAME *out_frame, uint32_t timeout_ms);

/* ============================================================
    SERVICE LAYER — DECLARATIONS
   ============================================================ */

/* ── Mode-01 numeric PIDs ── */
obd2_status_t service_obd2_getRPM(obd2_result_t *out);
obd2_status_t service_obd2_getSpeed(obd2_result_t *out);
obd2_status_t service_obd2_getCoolantTemp(obd2_result_t *out);
obd2_status_t service_obd2_getIAT(obd2_result_t *out);
obd2_status_t service_obd2_getThrottle(obd2_result_t *out);
obd2_status_t service_obd2_getRuntime(obd2_result_t *out);
obd2_status_t service_obd2_getFuelLevel(obd2_result_t *out);
obd2_status_t service_obd2_getBattVoltage(obd2_result_t *out);
obd2_status_t service_obd2_getAmbientTemp(obd2_result_t *out);
obd2_status_t service_obd2_getEngineLoad(obd2_result_t *out);
obd2_status_t service_obd2_getMAF(obd2_result_t *out);
obd2_status_t service_obd2_getO2Voltage(obd2_result_t *out);
obd2_status_t service_obd2_getFuelPressure(obd2_result_t *out);

/* ── Mode-03 / Mode-04  (DTCs) ── */
obd2_status_t service_obd2_readDTCs(obd2_dtc_result_t *out);
obd2_status_t service_obd2_clearDTCs(void);

/* ── Mode-09  (vehicle info) ── */
obd2_status_t service_obd2_getVIN(char *out_vin, uint8_t buf_len);
obd2_status_t service_obd2_getECUName(char *out_name, uint8_t buf_len);

/* ── Internal helper ── */
static obd2_status_t obd2_requestPID(
    uint8_t mode, uint8_t pid,
    obd2_result_t *out);

static obd2_status_t obd2_requestMultiframe(
    uint8_t mode, uint8_t pid,
    uint8_t *out_buf, uint8_t buf_len, uint8_t *out_len);

static const char *dtc_prefix(uint8_t high_nibble);

/* ============================================================
    SETUP
   ============================================================ */
void setup()
{
  Serial.begin(115200);
  Serial.println("===========================================");
  Serial.println("  ESP32 OBD-II Reader — Initialising …   ");
  Serial.println("===========================================");

  intf_can_init();

  Serial.println("CAN bus ready. Starting polling loop.");
}

/* ============================================================
    LOOP  — poll every parameter and print
   ============================================================ */
void loop()
{

  obd2_result_t res;
  obd2_dtc_result_t dtc_res;
  char str_buf[50];

  /* ── RPM ── */
  if (service_obd2_getRPM(&res) == STATUS_OK)
  {
    Serial.print("RPM           : ");
    Serial.println((uint16_t)res.value);
  }

  /* ── Speed ── */
  if (service_obd2_getSpeed(&res) == STATUS_OK)
  {
    Serial.print("Speed (km/h)  : ");
    Serial.println((uint8_t)res.value);
  }

  /* ── Coolant Temp ── */
  if (service_obd2_getCoolantTemp(&res) == STATUS_OK)
  {
    Serial.print("Coolant (°C)  : ");
    Serial.println((int16_t)res.value);
  }

  /* ── IAT ── */
  if (service_obd2_getIAT(&res) == STATUS_OK)
  {
    Serial.print("IAT (°C)      : ");
    Serial.println((int16_t)res.value);
  }

  /* ── Throttle ── */
  if (service_obd2_getThrottle(&res) == STATUS_OK)
  {
    Serial.print("Throttle (%)  : ");
    Serial.println(res.value, 1);
  }

  /* ── Runtime ── */
  if (service_obd2_getRuntime(&res) == STATUS_OK)
  {
    Serial.print("Runtime (s)   : ");
    Serial.println((uint16_t)res.value);
  }

  /* ── Fuel Level ── */
  if (service_obd2_getFuelLevel(&res) == STATUS_OK)
  {
    Serial.print("Fuel (%)      : ");
    Serial.println(res.value, 1);
  }

  /* ── Battery Voltage ── */
  if (service_obd2_getBattVoltage(&res) == STATUS_OK)
  {
    Serial.print("Battery (V)   : ");
    Serial.println(res.value, 2);
  }

  /* ── Ambient Temp ── */
  if (service_obd2_getAmbientTemp(&res) == STATUS_OK)
  {
    Serial.print("Ambient (°C)  : ");
    Serial.println((int16_t)res.value);
  }

  /* ── Engine Load ── */
  if (service_obd2_getEngineLoad(&res) == STATUS_OK)
  {
    Serial.print("Eng.Load (%)  : ");
    Serial.println(res.value, 1);
  }

  /* ── MAF ── */
  if (service_obd2_getMAF(&res) == STATUS_OK)
  {
    Serial.print("MAF (g/s)     : ");
    Serial.println(res.value, 2);
  }

  /* ── O2 Voltage ── */
  if (service_obd2_getO2Voltage(&res) == STATUS_OK)
  {
    Serial.print("O2 Volt (V)   : ");
    Serial.println(res.value, 3);
  }

  /* ── Fuel Pressure ── */
  if (service_obd2_getFuelPressure(&res) == STATUS_OK)
  {
    Serial.print("Fuel Pres(kPa): ");
    Serial.println((uint16_t)res.value);
  }

  /* ── DTCs ── */
  if (service_obd2_readDTCs(&dtc_res) == STATUS_OK)
  {
    Serial.print("DTCs Found    : ");
    Serial.println(dtc_res.count);
    for (uint8_t i = 0; i < dtc_res.count; i++)
    {
      Serial.print("  ");
      Serial.println(dtc_res.codes[i]);
    }
  }

  /* ── VIN ── */
  if (service_obd2_getVIN(str_buf, sizeof(str_buf)) == STATUS_OK)
  {
    Serial.print("VIN           : ");
    Serial.println(str_buf);
  }

  /* ── ECU Name ── */
  if (service_obd2_getECUName(str_buf, sizeof(str_buf)) == STATUS_OK)
  {
    Serial.print("ECU Name      : ");
    Serial.println(str_buf);
  }

  Serial.println("-------------------------------------------");

  vTaskDelay(pdMS_TO_TICKS(100));
}

/* ============================================================
    INTERFACE LAYER — IMPLEMENTATIONS
   ============================================================ */

/**
   @brief  Initialise CAN0 peripheral, set pins and baud rate,
           and install a hardware watch-filter for ECU response ID.
*/
void intf_can_init(void)
{
  CAN0.setCANPins(INTF_CAN_RX_PIN, INTF_CAN_TX_PIN);
  CAN0.begin(INTF_CAN_SPEED);
  CAN0.watchFor(INTF_CAN_OBD2_RESP_ID);
  Serial.println("intf_can_init : CAN0 initialised at 500 kbps");
}

/**
   @brief  Build and transmit a CAN frame.

   @param  id      11-bit CAN identifier.
   @param  data    Pointer to payload bytes.
   @param  length  Number of bytes to send (clamped to 8).
   @return true on success, false on driver error.
*/
bool intf_can_sendFrame(uint32_t id, const uint8_t *data, uint8_t length)
{
  CAN_FRAME tx;

  tx.id = id;
  tx.extended = false;
  tx.rtr = 0;
  tx.length = (length > 8) ? 8 : length;

  for (uint8_t i = 0; i < tx.length; i++)
  {
    tx.data.byte[i] = data[i];
  }

  return CAN0.sendFrame(tx);
}

/**
   @brief  Block-wait for a CAN frame matching expected_id.

   @param  expected_id  CAN ID to accept (0 = accept any).
   @param  out_frame    Pointer to frame struct to fill.
   @param  timeout_ms  Maximum wait in milliseconds.
   @return STATUS_OK on frame received, STATUS_TIMEOUT otherwise.
*/
obd2_status_t intf_can_readFrame(uint32_t expected_id,
                                 CAN_FRAME *out_frame,
                                 uint32_t timeout_ms)
{
  uint32_t deadline = millis() + timeout_ms;

  while (millis() < deadline)
  {
    if (CAN0.read(*out_frame))
    {
      if ((expected_id == 0) || (out_frame->id == expected_id))
      {
        return STATUS_OK;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  return STATUS_TIMEOUT;
}

/* ============================================================
    SERVICE LAYER — INTERNAL HELPERS
   ============================================================ */

/**
   @brief  Send a Mode-01 (or similar) single-frame PID request and
           decode the single-frame response into out->raw[].

   @param  mode  OBD-II service mode byte.
   @param  pid   Parameter ID byte.
   @param  out   Pointer to result struct; raw[] and raw_len are filled.
   @return STATUS_OK / STATUS_TIMEOUT / STATUS_ERROR.
*/
static obd2_status_t obd2_requestPID(uint8_t mode,
                                     uint8_t pid,
                                     obd2_result_t *out)
{
  /* Build 8-byte ISO-TP single-frame request:
     byte[0] = DLC (2), byte[1] = mode, byte[2] = PID, rest = 0x00  */
  const uint8_t req[8] = {0x02, mode, pid,
                          0x00, 0x00, 0x00, 0x00, 0x00};

  if (!intf_can_sendFrame(INTF_CAN_OBD2_REQ_ID, req, 8))
  {
    return STATUS_ERROR;
  }

  CAN_FRAME rx;
  obd2_status_t st = intf_can_readFrame(INTF_CAN_OBD2_RESP_ID,
                                        &rx,
                                        INTF_CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK)
    return st;

  /* Validate: response mode = request mode + 0x40, PID must match */
  if (rx.data.byte[1] != (mode + 0x40) ||
      rx.data.byte[2] != pid)
  {
    return STATUS_ERROR;
  }

  /* byte[0] = DLC; data payload starts at byte[3] */
  uint8_t dlc = rx.data.byte[0];
  if (dlc < 2)
    return STATUS_ERROR;

  out->raw_len = (dlc - 2 < sizeof(out->raw)) ? (dlc - 2) : sizeof(out->raw);
  for (uint8_t i = 0; i < out->raw_len; i++)
  {
    out->raw[i] = rx.data.byte[3 + i];
  }

  return STATUS_OK;
}

/**
   @brief  Handle a Mode-09 (vehicle info) multi-frame ISO-TP exchange.
           Sends the request, handles First-Frame → Flow-Control →
           Consecutive-Frames sequence, and assembles the payload.

   @param  mode     OBD-II service mode (typically 0x09).
   @param  pid      Info type (0x02 = VIN, 0x0A = ECU name).
   @param  out_buf  Caller-supplied buffer for assembled ASCII payload.
   @param  buf_len  Size of out_buf.
   @param  out_len  Number of bytes written to out_buf.
   @return STATUS_OK on success.
*/
static obd2_status_t obd2_requestMultiframe(uint8_t mode,
                                            uint8_t pid,
                                            uint8_t *out_buf,
                                            uint8_t buf_len,
                                            uint8_t *out_len)
{
  const uint8_t req[8] = {0x02, mode, pid,
                          0x00, 0x00, 0x00, 0x00, 0x00};

  if (!intf_can_sendFrame(INTF_CAN_OBD2_REQ_ID, req, 8))
  {
    return STATUS_ERROR;
  }

  CAN_FRAME rx;
  obd2_status_t st = intf_can_readFrame(INTF_CAN_OBD2_RESP_ID,
                                        &rx,
                                        INTF_CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK)
    return st;

  uint8_t frame_type = rx.data.byte[0] & 0xF0;

  /* ── Single frame response (short strings) ── */
  if (frame_type == ISOTP_SINGLE_FRAME)
  {
    uint8_t dlc = rx.data.byte[0] & 0x0F;
    /* skip mode+0x40, pid, data-count byte → data starts at byte[4] */
    uint8_t copy_len = (dlc - 3 < buf_len) ? (dlc - 3) : buf_len - 1;
    memcpy(out_buf, &rx.data.byte[4], copy_len);
    out_buf[copy_len] = '\0';
    *out_len = copy_len;
    return STATUS_OK;
  }

  /* ── First Frame (multi-frame) ── */
  if (frame_type != ISOTP_FIRST_FRAME)
    return STATUS_ERROR;

  uint16_t total_len = ((rx.data.byte[0] & 0x0F) << 8) | rx.data.byte[1];
  uint8_t idx = 0;

  /* First 6 data bytes are in the First Frame (bytes [2..7]).
     Skip mode+0x40 (byte[2]), pid (byte[3]), count (byte[4]) → data from byte[5]. */
  for (uint8_t i = 5; i <= 7 && idx < buf_len - 1; i++)
  {
    out_buf[idx++] = rx.data.byte[i];
  }

  /* Send Flow Control: ContinueToSend, block-size=0, ST-min=0 */
  const uint8_t fc[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (!intf_can_sendFrame(INTF_CAN_OBD2_REQ_ID, fc, 8))
    return STATUS_ERROR;

  /* Receive consecutive frames until total_len satisfied */
  uint8_t seq_num = 1;
  while (idx < total_len - 3 && idx < buf_len - 1)
  {
    st = intf_can_readFrame(INTF_CAN_OBD2_RESP_ID,
                            &rx, INTF_CAN_READ_TIMEOUT_MS);
    if (st != STATUS_OK)
      return st;

    if ((rx.data.byte[0] & 0xF0) != ISOTP_CONSEC_FRAME)
      return STATUS_ERROR;
    if ((rx.data.byte[0] & 0x0F) != (seq_num & 0x0F))
      return STATUS_ERROR;

    for (uint8_t i = 1; i <= 7 && idx < buf_len - 1; i++)
    {
      out_buf[idx++] = rx.data.byte[i];
    }
    seq_num++;
  }

  out_buf[idx] = '\0';
  *out_len = idx;
  return STATUS_OK;
}

/** Helper: return DTC letter prefix from the top-2 bits of high byte */
static const char *dtc_prefix(uint8_t high_nibble)
{
  switch (high_nibble & 0x03)
  {
  case 0:
    return "P"; /* Powertrain   */
  case 1:
    return "C"; /* Chassis      */
  case 2:
    return "B"; /* Body         */
  case 3:
    return "U"; /* Network/Comm */
  default:
    return "?";
  }
}

/* ============================================================
    SERVICE LAYER — MODE-01 NUMERIC PIDs
   ============================================================ */

/**
   @brief  Request and decode engine RPM.
           Formula: RPM = ((A*256)+B) / 4

   @param  out  Result; out->value = RPM (float).
*/
obd2_status_t service_obd2_getRPM(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_RPM, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 2)
    return STATUS_ERROR;

  out->value = ((out->raw[0] * 256.0f) + out->raw[1]) / 4.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode vehicle speed.
           Formula: Speed (km/h) = A

   @param  out  Result; out->value = speed in km/h.
*/
obd2_status_t service_obd2_getSpeed(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_SPEED, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = (float)out->raw[0];
  return STATUS_OK;
}

/**
   @brief  Request and decode engine coolant temperature.
           Formula: Temp (°C) = A - 40

   @param  out  Result; out->value = temperature in °C.
*/
obd2_status_t service_obd2_getCoolantTemp(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_COOLANT_TEMP, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = (float)out->raw[0] - 40.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode intake air temperature (IAT).
           Formula: IAT (°C) = A - 40

   @param  out  Result; out->value = temperature in °C.
*/
obd2_status_t service_obd2_getIAT(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_IAT, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = (float)out->raw[0] - 40.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode throttle position.
           Formula: Throttle (%) = A * 100 / 255

   @param  out  Result; out->value = throttle % (0–100).
*/
obd2_status_t service_obd2_getThrottle(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_THROTTLE, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = out->raw[0] * 100.0f / 255.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode engine run time since start.
           Formula: Runtime (s) = (A*256) + B

   @param  out  Result; out->value = seconds (float).
*/
obd2_status_t service_obd2_getRuntime(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_RUNTIME, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 2)
    return STATUS_ERROR;

  out->value = (float)((out->raw[0] << 8) | out->raw[1]);
  return STATUS_OK;
}

/**
   @brief  Request and decode fuel tank level.
           Formula: Fuel (%) = A * 100 / 255

   @param  out  Result; out->value = fuel % (0–100).
*/
obd2_status_t service_obd2_getFuelLevel(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_FUEL_LEVEL, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = out->raw[0] * 100.0f / 255.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode control module / battery voltage.
           Formula: Voltage (V) = ((A*256)+B) / 1000

   @param  out  Result; out->value = voltage in V.
*/
obd2_status_t service_obd2_getBattVoltage(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_BAT_VOLTAGE, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 2)
    return STATUS_ERROR;

  out->value = (float)((out->raw[0] << 8) | out->raw[1]) / 1000.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode ambient air temperature.
           Formula: Temp (°C) = A - 40

   @param  out  Result; out->value = temperature in °C.
*/
obd2_status_t service_obd2_getAmbientTemp(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_AMBIENT_TEMP, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = (float)out->raw[0] - 40.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode calculated engine load.
           Formula: Load (%) = A * 100 / 255

   @param  out  Result; out->value = engine load % (0–100).
*/
obd2_status_t service_obd2_getEngineLoad(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_ENGINE_LOAD, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = out->raw[0] * 100.0f / 255.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode Mass Air Flow (MAF) sensor rate.
           Formula: MAF (g/s) = ((A*256)+B) / 100

   @param  out  Result; out->value = mass air flow in g/s.
*/
obd2_status_t service_obd2_getMAF(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_MAF, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 2)
    return STATUS_ERROR;

  out->value = (float)((out->raw[0] << 8) | out->raw[1]) / 100.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode Bank-1 Sensor-1 O2 voltage.
           Formula: Voltage (V) = A / 200

   @param  out  Result; out->value = voltage in V (0–1.275).
*/
obd2_status_t service_obd2_getO2Voltage(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_O2_VOLTAGE, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = out->raw[0] / 200.0f;
  return STATUS_OK;
}

/**
   @brief  Request and decode fuel rail pressure (gauge).
           Formula: Pressure (kPa) = A * 3

   @param  out  Result; out->value = pressure in kPa.
*/
obd2_status_t service_obd2_getFuelPressure(obd2_result_t *out)
{
  obd2_status_t st = obd2_requestPID(OBD2_MODE_CURRENT_DATA,
                                     OBD2_PID_FUEL_PRESSURE, out);
  if (st != STATUS_OK)
    return st;
  if (out->raw_len < 1)
    return STATUS_ERROR;

  out->value = (float)out->raw[0] * 3.0f;
  return STATUS_OK;
}

/* ============================================================
    SERVICE LAYER — DTC READ / CLEAR  (Mode 03 / 04)
   ============================================================ */

/**
   @brief  Request stored Diagnostic Trouble Codes (Mode 03).
           Decodes up to OBD2_MAX_DTCS codes into out->codes[].

   @param  out  Caller-supplied DTC result struct.
   @return STATUS_OK on success (even when DTC count is 0).
*/
obd2_status_t service_obd2_readDTCs(obd2_dtc_result_t *out)
{
  /* Mode-03 request has no PID: just [0x01, 0x03, 0x00 …] */
  const uint8_t req[8] = {0x01, OBD2_MODE_DTC_READ,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  if (!intf_can_sendFrame(INTF_CAN_OBD2_REQ_ID, req, 8))
  {
    return STATUS_ERROR;
  }

  CAN_FRAME rx;
  obd2_status_t st = intf_can_readFrame(INTF_CAN_OBD2_RESP_ID,
                                        &rx,
                                        INTF_CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK)
    return st;

  /* byte[1] = 0x43 (mode 03 + 0x40), byte[2] = number of DTCs */
  if (rx.data.byte[1] != (OBD2_MODE_DTC_READ + 0x40))
    return STATUS_ERROR;

  out->count = 0;
  uint8_t num_dtcs = rx.data.byte[2];

  /* Each DTC occupies 2 bytes; first pair starts at byte[3] */
  for (uint8_t i = 0; i < num_dtcs && out->count < OBD2_MAX_DTCS; i++)
  {
    uint8_t high = rx.data.byte[3 + (i * 2)];
    uint8_t low = rx.data.byte[4 + (i * 2)];

    if (high == 0x00 && low == 0x00)
      continue; /* padding byte */

    /* Format: <prefix><high_nibble><low_nibble_of_high><high_nibble_of_low><low_nibble> */
    snprintf(out->codes[out->count], 6, "%s%X%02X",
             dtc_prefix(high >> 6),
             (high >> 4) & 0x03,
             ((high & 0x0F) << 4) | (low >> 4));

    /* append last nibble */
    char tmp[2] = {"0123456789ABCDEF"[low & 0x0F], '\0'};
    strncat(out->codes[out->count], tmp, 1);

    out->count++;
  }

  return STATUS_OK;
}

/**
   @brief  Send Mode-04 command to clear all stored DTCs and
           reset the MIL (Check Engine Light).

   @return STATUS_OK on positive acknowledgement.
*/
obd2_status_t service_obd2_clearDTCs(void)
{
  const uint8_t req[8] = {0x01, OBD2_MODE_DTC_CLEAR,
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  if (!intf_can_sendFrame(INTF_CAN_OBD2_REQ_ID, req, 8))
  {
    return STATUS_ERROR;
  }

  CAN_FRAME rx;
  obd2_status_t st = intf_can_readFrame(INTF_CAN_OBD2_RESP_ID,
                                        &rx,
                                        INTF_CAN_READ_TIMEOUT_MS);
  if (st != STATUS_OK)
    return st;

  /* Positive response: byte[1] = 0x44 */
  if (rx.data.byte[1] != (OBD2_MODE_DTC_CLEAR + 0x40))
    return STATUS_ERROR;

  Serial.println("service_obd2_clearDTCs : DTCs cleared successfully.");
  return STATUS_OK;
}

/* ============================================================
    SERVICE LAYER — VEHICLE INFORMATION  (Mode 09)
   ============================================================ */

/**
   @brief  Request 17-character Vehicle Identification Number (VIN).
           Uses ISO-TP multi-frame exchange internally.

   @param  out_vin  Caller buffer; must be at least 18 bytes.
   @param  buf_len  Size of out_vin buffer.
   @return STATUS_OK on success.
*/
obd2_status_t service_obd2_getVIN(char *out_vin, uint8_t buf_len)
{
  uint8_t raw[50] = {0};
  uint8_t raw_len = 0;

  obd2_status_t st = obd2_requestMultiframe(OBD2_MODE_VIN,
                                            OBD2_PID_VIN,
                                            raw, sizeof(raw),
                                            &raw_len);
  if (st != STATUS_OK)
    return st;

  /* Copy only printable ASCII characters */
  uint8_t out_idx = 0;
  for (uint8_t i = 0; i < raw_len && out_idx < buf_len - 1; i++)
  {
    if (raw[i] >= 0x20 && raw[i] <= 0x7E)
    {
      out_vin[out_idx++] = (char)raw[i];
    }
  }
  out_vin[out_idx] = '\0';
  return STATUS_OK;
}

/**
   @brief  Request ECU / component name string (Mode 09, Info 0x0A).
           Uses ISO-TP multi-frame exchange internally.

   @param  out_name  Caller buffer; 20+ bytes recommended.
   @param  buf_len   Size of out_name buffer.
   @return STATUS_OK on success.
*/
obd2_status_t service_obd2_getECUName(char *out_name, uint8_t buf_len)
{
  uint8_t raw[50] = {0};
  uint8_t raw_len = 0;

  obd2_status_t st = obd2_requestMultiframe(OBD2_MODE_VIN,
                                            OBD2_PID_ECU_NAME,
                                            raw, sizeof(raw),
                                            &raw_len);
  if (st != STATUS_OK)
    return st;

  uint8_t out_idx = 0;
  for (uint8_t i = 0; i < raw_len && out_idx < buf_len - 1; i++)
  {
    if (raw[i] >= 0x20 && raw[i] <= 0x7E)
    {
      out_name[out_idx++] = (char)raw[i];
    }
  }
  out_name[out_idx] = '\0';
  return STATUS_OK;
}
