#include <Arduino.h>
//#include <ads7028.h>





//pinMode(PC13, OUTPUT)

// put function declarations here:
int myFunction(int, int);

void setup() {

  /* -------------------------------------------------------------------------- */
  /*                                  PIN INIT                                  */
  /* -------------------------------------------------------------------------- */
  pinMode(PA8, INPUT); //Low pressure
  pinMode(PA10, INPUT); //High Pressure
  pinMode(PA11, INPUT); //5kW Signal

  pinMode(PB12, INPUT); //BSPD DELAY
  pinMode(PB13, INPUT); //CLOCK
  pinMode(PB14, INPUT); //BSPD OK

  pinMode(PB0, OUTPUT); //CAN TX INDC
  pinMode(PB1, OUTPUT); //CAN RX INDC
  pinMode(PC13, OUTPUT); //PC13

  //CANBUS PINS

  //SPI PINS




}

void loop() {
  // put your main code here, to run repeatedly:

  
}