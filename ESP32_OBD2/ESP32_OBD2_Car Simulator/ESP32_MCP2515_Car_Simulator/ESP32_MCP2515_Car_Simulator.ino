#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS 5

MCP_CAN CAN(CAN_CS);

// ===== CAN IDs =====
#define OBD_REQ_ID  0x7DF
#define ECU_RESP_ID 0x7E8

// ===== OBD SERVICE MODES =====
#define MODE_CURRENT_DATA 0x01
#define MODE_READ_DTC    0x03
#define MODE_CLEAR_DTC   0x04
#define MODE_INFO        0x09

// ===== PID DEFINITIONS =====
#define PIDS_SUPPORT_01_20     0x00
#define PIDS_SUPPORT_21_40     0x20
#define PIDS_SUPPORT_41_60     0x40

#define PID_ENGINE_LOAD        0x04
#define PID_COOLANT_TEMP       0x05
#define PID_STFT_B1            0x06
#define PID_LTFT_B1            0x07
#define PID_FUEL_PRESSURE      0x0A
#define PID_MAP                0x0B
#define PID_ENGINE_RPM         0x0C
#define PID_VEHICLE_SPEED      0x0D
#define PID_IGN_ADVANCE        0x0E
#define PID_IAT                0x0F
#define PID_THROTTLE           0x11
#define PID_O2_VOLTAGE         0x14
#define PID_RUNTIME            0x1F
#define PID_BATTERY_VOLTAGE    0x42
#define PID_AMBIENT_TEMP       0x46
#define PID_OIL_TEMP           0x5C

// ===== MODE 09 PIDS =====
#define PID_VIN                0x02
#define PID_ECU_NAME           0x0A

// ===== FAKE DATA =====
uint16_t fakeRPM = 1200;
uint8_t fakeSpeed = 40;
uint8_t fakeThrottle = 25;
uint8_t fakeLoad = 35;
uint16_t fakeRuntime = 100;
uint16_t fakeBatteryMv = 12600;

const char fakeVIN[] = "TESTVIN1234567890";
const char fakeECU[] = "FAKE-ESP32-ECU";

// Enable/disable fake DTCs here
bool dtcActive = true;

void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println("Booting fake car simulator with MCP2515...");

  // SPI: SCK=13, MISO=19, MOSI=23, CS=5
  SPI.begin(13, 19, 23, CAN_CS);

  // Change MCP_8MHZ to MCP_16MHZ if your MCP2515 board uses 16 MHz crystal
  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ))
  {
    Serial.println("CAN init fail, retrying...");
    delay(500);
  }

  Serial.println("CAN init OK!");
  CAN.setMode(MCP_NORMAL);
}

void loop()
{
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&rxId, &len, buf);

    Serial.print("REQUEST ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" DATA:");
    for (uint8_t i = 0; i < len; i++)
    {
      printhex(buf[i]);
    }
    Serial.println();

    if (rxId == OBD_REQ_ID || rxId == 0x7E0)
    {
      handleOBDRequest(buf, len);
    }

    updateFakeData();
  }

  delay(5);
}

// =======================================================
// HANDLE OBD REQUEST
// =======================================================
void handleOBDRequest(unsigned char *buf, unsigned char len)
{
  if (len < 2) return;

  uint8_t mode = buf[1];
  uint8_t pid  = buf[2];

  if (mode == MODE_CURRENT_DATA)
  {
    handleMode01(pid);
  }
  else if (mode == MODE_READ_DTC)
  {
    handleReadDTC();
  }
  else if (mode == MODE_CLEAR_DTC)
  {
    handleClearDTC();
  }
  else if (mode == MODE_INFO)
  {
    handleMode09(pid);
  }
  else
  {
    sendNegativeResponse(mode, 0x11); // service not supported
  }
}

// =======================================================
// MODE 01 - LIVE DATA
// =======================================================
void handleMode01(uint8_t pid)
{
  byte out[8] = {0};

  out[1] = 0x41;
  out[2] = pid;

  switch (pid)
  {
    case PIDS_SUPPORT_01_20:
      // Supports 01-20 range.
      // For simulator, return all supported.
      out[0] = 0x06;
      out[3] = 0xFF;
      out[4] = 0xFF;
      out[5] = 0xFF;
      out[6] = 0xFF;
      break;

    case PIDS_SUPPORT_21_40:
      out[0] = 0x06;
      out[3] = 0xFF;
      out[4] = 0xFF;
      out[5] = 0xFF;
      out[6] = 0xFF;
      break;

    case PIDS_SUPPORT_41_60:
      out[0] = 0x06;
      out[3] = 0xFF;
      out[4] = 0xFF;
      out[5] = 0xFF;
      out[6] = 0xFF;
      break;

    case PID_ENGINE_LOAD:
      // Formula: A * 100 / 255
      out[0] = 0x03;
      out[3] = map(fakeLoad, 0, 100, 0, 255);
      break;

    case PID_COOLANT_TEMP:
      // Formula: A - 40
      out[0] = 0x03;
      out[3] = 85 + 40;
      break;

    case PID_STFT_B1:
      // Formula: (A / 128 - 1) * 100
      // 128 = 0%
      out[0] = 0x03;
      out[3] = 128;
      break;

    case PID_LTFT_B1:
      out[0] = 0x03;
      out[3] = 130;
      break;

    case PID_FUEL_PRESSURE:
      // Formula: A * 3 kPa
      out[0] = 0x03;
      out[3] = 50; // 150 kPa
      break;

    case PID_MAP:
      // kPa directly
      out[0] = 0x03;
      out[3] = 45;
      break;

    case PID_ENGINE_RPM:
    {
      // Formula: ((A * 256) + B) / 4
      uint16_t rpmRaw = fakeRPM * 4;
      out[0] = 0x04;
      out[3] = highByte(rpmRaw);
      out[4] = lowByte(rpmRaw);
      break;
    }

    case PID_VEHICLE_SPEED:
      // km/h directly
      out[0] = 0x03;
      out[3] = fakeSpeed;
      break;

    case PID_IGN_ADVANCE:
      // Formula: A / 2 - 64
      // Example 12 degree BTDC => A = (12 + 64) * 2 = 152
      out[0] = 0x03;
      out[3] = 152;
      break;

    case PID_IAT:
      // Formula: A - 40
      out[0] = 0x03;
      out[3] = 32 + 40;
      break;

    case PID_THROTTLE:
      // Formula: A * 100 / 255
      out[0] = 0x03;
      out[3] = map(fakeThrottle, 0, 100, 0, 255);
      break;

    case PID_O2_VOLTAGE:
      // Formula used in your dashboard: A * 0.005
      // 0.700V / 0.005 = 140
      out[0] = 0x04;
      out[3] = 140;
      out[4] = 128;
      break;

    case PID_RUNTIME:
      // Formula: A * 256 + B seconds
      out[0] = 0x04;
      out[3] = highByte(fakeRuntime);
      out[4] = lowByte(fakeRuntime);
      break;

    case PID_BATTERY_VOLTAGE:
      // Your dashboard formula: ((A * 256) + B) / 1000.0
      // Example 12.600V => 12600
      out[0] = 0x04;
      out[3] = highByte(fakeBatteryMv);
      out[4] = lowByte(fakeBatteryMv);
      break;

    case PID_AMBIENT_TEMP:
      // Formula: A - 40
      out[0] = 0x03;
      out[3] = 30 + 40;
      break;

    case PID_OIL_TEMP:
      // Formula: A - 40
      out[0] = 0x03;
      out[3] = 90 + 40;
      break;

    default:
      sendNegativeResponse(MODE_CURRENT_DATA, 0x12); // sub-function/PID not supported
      return;
  }

  sendCAN(out);
}

// =======================================================
// MODE 03 - READ DTC
// =======================================================
void handleReadDTC()
{
  byte out[8] = {0};

  if (!dtcActive)
  {
    // No DTCs
    out[0] = 0x02;
    out[1] = 0x43;
    out[2] = 0x00;
    sendCAN(out);
    return;
  }

  /*
    Response format:
    out[0] = number of following data bytes
    out[1] = 0x43
    out[2] = DTC count
    then each DTC uses 2 bytes

    Example DTCs:
      P0300 -> bytes 0x03 0x00
      U0100 -> bytes 0xC1 0x00

    DTC encoding:
      First two bits:
        00 = P
        01 = C
        10 = B
        11 = U
  */

  out[0] = 0x06;
  out[1] = 0x43;
  out[2] = 0x02;  // two DTCs

  // P0300
  out[3] = 0x03;
  out[4] = 0x00;

  // U0100
  out[5] = 0xC1;
  out[6] = 0x00;

  sendCAN(out);
}

// =======================================================
// MODE 04 - CLEAR DTC
// =======================================================
void handleClearDTC()
{
  dtcActive = false;

  byte out[8] = {0};
  out[0] = 0x01;
  out[1] = 0x44;

  sendCAN(out);

  Serial.println("DTCs cleared by tester");
}

// =======================================================
// MODE 09 - VEHICLE INFO
// =======================================================
void handleMode09(uint8_t pid)
{
  if (pid == PID_VIN)
  {
    sendAsciiSingleFrame(0x49, PID_VIN, fakeVIN);
  }
  else if (pid == PID_ECU_NAME)
  {
    sendAsciiSingleFrame(0x49, PID_ECU_NAME, fakeECU);
  }
  else
  {
    sendNegativeResponse(MODE_INFO, 0x12);
  }
}

// =======================================================
// SEND ASCII RESPONSE - SIMPLE SINGLE FRAME
// Good enough for short fake values.
// =======================================================
void sendAsciiSingleFrame(uint8_t responseMode, uint8_t pid, const char *text)
{
  byte out[8] = {0};

  uint8_t textLen = strlen(text);

  /*
    ISO-TP single frame can carry max 7 data bytes.
    We need responseMode + pid + data, so only 5 ASCII chars fit.

    For simple testing, this sends first 5 chars only.
    For full VIN, you need multi-frame ISO-TP.
  */

  uint8_t copyLen = textLen;
  if (copyLen > 5) copyLen = 5;

  out[0] = 2 + copyLen;
  out[1] = responseMode;
  out[2] = pid;

  for (uint8_t i = 0; i < copyLen; i++)
  {
    out[3 + i] = text[i];
  }

  sendCAN(out);
}

// =======================================================
// SEND NEGATIVE RESPONSE
// =======================================================
void sendNegativeResponse(uint8_t requestedMode, uint8_t reason)
{
  byte out[8] = {0};

  out[0] = 0x03;
  out[1] = 0x7F;
  out[2] = requestedMode;
  out[3] = reason;

  sendCAN(out);
}

// =======================================================
// SEND CAN FRAME
// =======================================================
void sendCAN(byte out[8])
{
  if (CAN.sendMsgBuf(ECU_RESP_ID, 0, 8, out) == CAN_OK)
  {
    Serial.print("REPLY ID: 0x");
    Serial.print(ECU_RESP_ID, HEX);
    Serial.print(" DATA:");
    for (uint8_t i = 0; i < 8; i++)
    {
      printhex(out[i]);
    }
    Serial.println();
  }
  else
  {
    Serial.println("Send Failed");
  }
}

// =======================================================
// FAKE DATA UPDATE
// =======================================================
void updateFakeData()
{
  fakeRPM += 75;
  if (fakeRPM > 4500) fakeRPM = 900;

  fakeSpeed += 2;
  if (fakeSpeed > 140) fakeSpeed = 20;

  fakeThrottle += 3;
  if (fakeThrottle > 90) fakeThrottle = 10;

  fakeLoad += 2;
  if (fakeLoad > 85) fakeLoad = 20;

  fakeRuntime++;

  fakeBatteryMv += 2;
  if (fakeBatteryMv > 12850) fakeBatteryMv = 12400;
}

// =======================================================
// HEX PRINT
// =======================================================
void printhex(uint8_t val)
{
  Serial.print("\t");
  if (val < 0x10) Serial.print("0");
  Serial.print(val, HEX);
}
