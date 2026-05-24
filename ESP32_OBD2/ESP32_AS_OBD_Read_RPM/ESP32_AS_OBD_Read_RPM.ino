#include <esp32_can.h>

bool requestRPM();

void setup() {
  Serial.begin(115200);
  Serial.println("ESP is NOW Initialized");

  CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5);
  CAN0.begin(500000);

  CAN0.watchFor(0x7E8);
}

void loop() {
  if (!requestRPM()) {
    Serial.println("Failed get request");
  }
  else
  {
    CAN_FRAME rx_frame;

    if (CAN0.read(rx_frame)) {

      // Decode RPM response
      if (rx_frame.id == 0x7E8 &&
          rx_frame.data.byte[1] == 0x41 &&
          rx_frame.data.byte[2] == 0x0C) {

        uint16_t rpm = ((rx_frame.data.byte[3] << 8) | rx_frame.data.byte[4]) / 4;

        Serial.print("RPM: ");
        Serial.println(rpm);
      }
    }
  }
  // Small yield to avoid starving other tasks
  vTaskDelay(pdMS_TO_TICKS(10));
}

// ============================
// CAN REQUEST FUNCTION
// ============================
bool requestRPM() {
  CAN_FRAME tx_frame;

  tx_frame.id = 0x7DF;
  tx_frame.extended = false;
  tx_frame.length = 8;
  tx_frame.rtr = 0;

  tx_frame.data.byte[0] = 0x02;
  tx_frame.data.byte[1] = 0x01;
  tx_frame.data.byte[2] = 0x0C;
  tx_frame.data.byte[3] = 0x00;
  tx_frame.data.byte[4] = 0x00;
  tx_frame.data.byte[5] = 0x00;
  tx_frame.data.byte[6] = 0x00;
  tx_frame.data.byte[7] = 0x00;

  return CAN0.sendFrame(tx_frame);
}
