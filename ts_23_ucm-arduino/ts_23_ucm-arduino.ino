// TEAM SWINBURNE - UNIVERSAL CONTROL MODULE - HARDWARE REVISION 0
// NAM TRAN
// Revision 2

/***************************************************************************
    ts_23_ucm-arduino.ino

    INTRO

    Revision     Date          Comments
    --------   ----------     ------------
    0.0        No clue        Initial coding
    1.0        20/06/2021     Updated with ticker class and pcb_temp
    2.0        24/06/2021     Rewrite Ticker as sperated header TickerInterrupt

 /*
 * Upload settings (for USB): 
 * U(S)ART Support: Enabled (generic) serial
 * USB support: CDC (generic "Serial" supersedes U(S)ART
 * Upload method: HID Bootloader 2.2
 * 
 * 
 *
****************************************************************************/
/*---------------------------- LIBRARIES ----------------------------------*/
#include <Wire.h>
#include <eXoCAN.h>
#include <Adafruit_ADS1X15.h>

/*------------------------------- GPIOS -----------------------------------*/
#define PIN_DIGITAL_IN_1      PB10
#define PIN_DIGITAL_IN_2      PB11
#define PIN_DIGITAL_IN_3      PB12
#define PIN_DIGITAL_IN_4      PB13
#define PIN_PWM_1             PB15 // (24V)
#define PIN_PWM_2             PB1  // (24V)

/*------------------------------ Interfaces -------------------------------*/

eXoCAN can; // CANBus interface

/*------------------------------ Callbacks --------------------------------*/
void readInput() {
  uint8_t data[8] = {0};
  data[0] = digitalRead(PIN_DIGITAL_IN_1);
  data[1] = digitalRead(PIN_DIGITAL_IN_2);
  data[2] = digitalRead(PIN_DIGITAL_IN_3);
  data[3] = digitalRead(PIN_DIGITAL_IN_4);

  can.transmit(0x255, data[0], 4);
}
void setup() {
  // inputs
  pinMode(PIN_DIGITAL_IN_1,INPUT);
  pinMode(PIN_DIGITAL_IN_2,INPUT);
  pinMode(PIN_DIGITAL_IN_3,INPUT);
  pinMode(PIN_DIGITAL_IN_4,INPUT);

  // outputs
  pinMode(PIN_PWM_1,OUTPUT);
  pinMode(PIN_PWM_2,OUTPUT);

  // init CAN
  can.begin(STD_ID_LEN, BR500K, PORTB_8_9_XCVR);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(200);
  readInput();
}

