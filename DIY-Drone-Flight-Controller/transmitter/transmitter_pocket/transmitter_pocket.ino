#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8, 9);  // CE, CSN pins
const byte address[6] = "00001";

struct JoystickData {
  int16_t joy1x;
  int16_t joy1y;
  int16_t joy2x;
  int16_t joy2y;
  int16_t btn1;
  int16_t btn2;
  int16_t btn3;
  int16_t btn4;
  int16_t btn5;
};

JoystickData joyData;

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);//MIN<LOW<HIGH<MAX
  radio.stopListening();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // Now expecting 9 values
    int vals[9] = { 0 };
    int index = 0;
    int start = 0;

    for (int i = 0; i < line.length(); i++) {
      if (line.charAt(i) == ',' || i == line.length() - 1) {
        String part = line.substring(start, (i == line.length() - 1) ? i + 1 : i);
        vals[index++] = part.toInt();
        start = i + 1;
        if (index >= 9) break;
      }
    }

    // Fill struct
    joyData.joy1x = vals[0];
    joyData.joy1y = vals[1];
    joyData.joy2x = vals[2];
    joyData.joy2y = vals[3];

    joyData.btn1 = vals[4];
    joyData.btn2 = vals[5];
    joyData.btn3 = vals[6];
    joyData.btn4 = vals[7];
    joyData.btn5 = vals[8];

    // Send via nRF24
    if(joyData.btn3 == -1 || joyData.btn3 == 0){
    radio.write(&joyData, sizeof(joyData));
    }
  }
}
