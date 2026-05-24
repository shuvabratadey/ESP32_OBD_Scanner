// requires https://github.com/collin80/esp32_can and https://github.com/collin80/can_common
/*
  ESP32        TJA1050
  -------      ---------
  GPIO5      →   TXD
  GPIO4      ←   RXD
  GND        →   GND
  3.3V       →   VCC
*/
#include <esp32_can.h>

void setup() {
  Serial.begin(115200);

  Serial.println("ESP is NOW Initialized");

  CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5); // see important note above
  CAN0.begin(500000); // 500Kbps
  //    CAN0.watchFor();
  //-----------------------
  //  For OBD-II Request and Responces Only
  //-----------------------
  CAN0.watchFor(0x7DF);
  CAN0.watchFor(0x7E8);
  /* For Example ==>

    ID: 0x7DF Data: 2 1 C 0 0 0 0 0
    ID: 0x7E8 Data: 4 41 C 0 0 0 0 0
  */
  //-----------------------
}

void loop() {
  CAN_FRAME can_message;
  if (CAN0.read(can_message)) {
    Serial.print("ID: 0x");
    Serial.print(can_message.id, HEX);
    Serial.print(" Data: ");

    for (int i = 0; i < can_message.length; i++) {
      Serial.print(can_message.data.byte[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}
