#include <Arduino.h>
#include <ads7128.h>
#include <SPI.h>

#define DATA_WRITE          ((uint8_t) 0b00001000)
#define DATA_READ           ((uint8_t) 0b00010000)
#define SPI_MASTER_DUMMY    ((uint8_t) 0b00000000)

#define RESET               ((uint8_t) 0b10000001)

#define CSPIN               (PB6)
#define MOSI                (PB5)
#define MISO                (PB4)
#define SCLK                (PB3)

SPIClass SPI_1(MOSI, MISO, SCLK);


void setup(){
  
  SPI.begin();
  
  SPI_1.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

  pinMode(CSPIN, OUTPUT);
  //pinMode(MOSI, OUTPUT);
  //pinMode(MISO, INPUT);
  //pinMode(SCLK, OUTPUT);

  pinMode(PC13, OUTPUT);
  
  digitalWrite(CSPIN, HIGH);
  delay(1);

  Serial.begin(9600);
}

uint8_t readRegister(uint8_t address) {
  digitalWrite(CSPIN, LOW);
  delay(1);
  SPI_1.transfer(DATA_READ);
  SPI_1.transfer(address);
  SPI_1.transfer(SPI_MASTER_DUMMY);
  digitalWrite(CSPIN, HIGH);
  delay(1);
  digitalWrite(CSPIN, LOW);
  delay(1);
  uint8_t data = SPI_1.transfer(SPI_MASTER_DUMMY);
  SPI_1.transfer(0x00);
  SPI_1.transfer(0x00);
  digitalWrite(CSPIN, HIGH);
  delay(1);
  return data;
}


void writeRegister(uint8_t address, uint8_t value) {
  digitalWrite(CSPIN, LOW);
  delayMicroseconds(2);
  SPI_1.transfer(DATA_WRITE);
  SPI_1.transfer(address);
  SPI_1.transfer(value);
  delayMicroseconds(2);
  digitalWrite(CSPIN, HIGH);
}

//void init(){


  // SPI.begin();
  // SPI.setDataMode(SPI_MODE0);
  
  // pinMode(CSPIN, OUTPUT);

  // digitalWrite(CSPIN, HIGH);
  // delay(1);
  //SPI.beginTransaction(SPISettings(60000000, MSBFIRST, SPI_MODE0));
//}

void loop(){
p
  //writeRegister(0x0, 0b10010001);
  writeRegister(0x01, 0b01001000);
  uint8_t Values = readRegister(0x1);
  digitalWrite(PC13, HIGH);
  delay(1000);
  digitalWrite(PC13, LOW);
  Serial.println(Values);
}







// void setup() {
  
//   Serial.begin(9600);
//   pinMode(CS, OUTPUT);
//   pinMode(PC13, OUTPUT);

// //  SPI.begin();
//   SPI.beginTransaction(SPISettings(60000000, MSBFIRST, SPI_MODE0));

//   //SPI.setClockDivider(SPI_CLOCK_DIV16);
//   digitalWrite(CS, HIGH);

//   digitalWrite(PC13, HIGH);

//   // put your setup code here, to run once:
// }

// void loop() {

//   byte MasterSend, MasterRecieve;
  
//   MasterSend = SYSTEM_STATUS_ADDRESS;

//   digitalWrite(CS, LOW);

//   MasterRecieve =SPI.transfer(MasterSend);

//   Serial.println(MasterRecieve);

//   digitalWrite(CS, HIGH);

//   digitalWrite(PC13, HIGH);

//   delay(2000);
  
//   digitalWrite(PC13, LOW);

//   delay(2000);
//   // put your main code here, to run repeatedly:
// }