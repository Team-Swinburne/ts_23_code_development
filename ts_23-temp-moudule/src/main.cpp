// #include <Arduino.h>
// #include <SPI.h>
// #include "ads7028.h"

// #define CS_PIN 21

// SPISettings settingsA(2000000, MSBFIRST, SPI_MODE1); 

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   Serial.println("Hello World");

//   SPI.begin();//beginTransaction(settingsA);
//   initADS7028();
//   pinMode(CS_PIN, OUTPUT);
//   digitalWrite(CS_PIN, HIGH);
//   delay(1000);
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   uint8_t val = readSingleRegister(GENERAL_CFG_ADDRESS);
//   Serial.println(val, HEX);
//   delay(500);
// }

#include <SPI.h>  // include the SPI library:

const int slaveSelectPin = 20;

int digitalPotWrite(int, int);

void setup() {
  // set the slaveSelectPin as an output:
  pinMode (slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin(); 
}

void loop() {
  // go through the six channels of the digital pot:
  for (int channel = 0; channel < 6; channel++) { 
    // change the resistance on this channel from min to max:
    for (int level = 0; level < 255; level++) {
      digitalPotWrite(channel, level);
      delay(10);
    }
    // wait a second at the top:
    delay(100);
    // change the resistance on this channel from max to min:
    for (int level = 0; level < 255; level++) {
      digitalPotWrite(channel, 255 - level);
      delay(10);
    }
  }
}

int digitalPotWrite(int address, int value) {
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin,LOW);
  //  send in the address and value via SPI:
  SPI.transfer(address);
  SPI.transfer(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin,HIGH); 
}