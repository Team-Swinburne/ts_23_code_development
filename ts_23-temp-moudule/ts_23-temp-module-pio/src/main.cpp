#include <Arduino.h>
#include <SPI.h>
#include "ads7028.h"

// #define CS_PIN 21
ADS7028 tempBank2(20);
ADS7028 tempBank3(19);
ADS7028 tempBank4(18);
ADS7028 tempBank5(17);
ADS7028 tempBank6(16);

void setup() {
  Serial.begin(9600);
  Serial.println("Hello World");

  // initialize SPI:
  SPI.begin();
  tempBank2.init();
  tempBank3.init();
  tempBank4.init();
  tempBank5.init();
  tempBank6.init();
}

void loop() {
  // read three bytes from device A
  delay(100);
  Serial.println(tempBank6.readSingleRegister(SYSTEM_STATUS_ADDRESS));
  // for (int i = 0; i < 8; i++) {
  //   Serial.print("Bank 6 Channel ");
  //   Serial.print(i);
  //   Serial.print(tempBank6.readVoltage(i));
  // }

}