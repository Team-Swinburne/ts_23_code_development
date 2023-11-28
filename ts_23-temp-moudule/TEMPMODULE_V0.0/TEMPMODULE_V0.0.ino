#include "Arduino.h"
#include <FlexCAN_T4.h>
#include "Ticker.h"
#include "can_addresses.h"
#include <SPI.h>
#include "ads7028.h"


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

void updateHeartdata();
void canTX();
Ticker Heartbeat (updateHeartdata, 1000, 0, MILLIS);  // Display heartbeat on CAN
Ticker Can_tx (canTX, 1000, 0, MILLIS);               // FOR TESTING ONLY


// Try without attaching it to interrupts first, if works, try attaching read function to interrupt.
// Ticker ADC (readAdc, 1000, 0, MILLIS);                // read ADS7028, transmit on CAN, and store in global variable for other purposes in the code.

struct msgFrame {
  uint8_t len = 8;
  uint8_t bytes[8] = {0};
};

static msgFrame	heartFrame, //{.len = 6},
               	errorFrame; //{.len = 2}, 

float ADC_Voltages[8] = {0};

void setup(void) {
  Serial.begin(115200); delay(400);
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();

 digitalWrite(21, HIGH);  // CS1 ENABLE
  digitalWrite(20, HIGH);  // CS2 ENABLE
  digitalWrite(19, HIGH);  // CS3 ENABLE
  digitalWrite(18, HIGH);  // CS4 ENABLE
  digitalWrite(17, HIGH);  // CS5 ENABLE
  digitalWrite(16, HIGH);  // CS6 ENABLE

  Heartbeat.start();
  Can_tx.start();

  SPI.begin();          // initialise SPI communication

    //----------------------------------------------------------------------------//
   //          ***      SPI MODE CONFIGURATION INFORMATION          ***          //
  //----------------------------------------------------------------------------//
  // Available SPI Modes: SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3
  // SPI_MODE0: CPHA = 0, CPOL = 0
  // SPI_MODE1: CPHA = 1, CPOL = 0
  // SPI_MODE2: CPHA = 0, CPOL = 1
  // SPI_MODE3: CPHA = 1, CPOL = 1
  // NOTE: Clock Polarity (CPOL): It determines the idle state of the clock signal.
  //                              CPOL = 0: Clock is in the low state when idle.
  //                              CPOL = 1: Clock is in the high state when idle.
  // NOTE: Clock Phase (CPHA): It determines when the data is sampled or changed.
  //                           CPHA = 0: Data is sampled on the leading (first) edge of the clock.
  //                           CPHA = 1: Data is sampled on the trailing (second) edge of the clock.

  SPI.setDataMode(SPI_MODE0);           // SPI_MODE0 - Low state when idle; SPI is sampled on rising edge of the clock.

  SPI.setClockDivider(SPI_CLOCK_DIV128); // System clock of Teens 4.0 is 600MHz; ADS7028 maximum clock frequency is 60MHz
                                         // Therefore, a divider of 10 or more is required
                                         // SPI clock divider only offers divisions (closest to 10) of 8 and 16
                                         // 16 was chosen for highest possible performance of the ADS7028

  SPI.setBitOrder(MSBFIRST);            // Reading 'Most Significant Bit FIRST' instead of 'Least Significant Bit FIRST'

  // SET UP ADC CONFIGURATION //
  initADS7028();  // initialise ADCs 

  // Configure chip mode
  for (int i = 0; i<6; i++){
      setChannelAsAnalogInput(i);
  } //Configure all pins as analog in
  delay(50);
  writeSingleRegister(OPMODE_CFG_ADDRESS, OPMODE_CFG_CONV_MODE_MANUAL_MODE); //Select Manual Mode
  delay(50);
  writeSingleRegister(SEQUENCE_CFG_ADDRESS, SEQUENCE_CFG_SEQ_MODE_MANUAL); //Manual Channel Selection Mode
  delay(50);

  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  pinMode(21, OUTPUT);  // C1
  pinMode(20, OUTPUT);  // C2
  pinMode(19, OUTPUT);  // C3
  pinMode(18, OUTPUT);  // C4
  pinMode(17, OUTPUT);  // C5
  pinMode(16, OUTPUT);  // C6
}

// update the heart message data and transmit hearbeat, letting the other pals know you're alive

// NOTE: NEED TO DO HEARTBEAT STATE WHEN FINISHED WITH OTHER CODE
void updateHeartdata() 
{
  // NOTE: YOU WON'T BE ABLE TO WRITE HEARTBEAT FROM HEARTBEAT FUNCTION. THIS IS ONLY TEMPORARY.
  CAN_message_t msg;
  msg.id = CAN_TEMP_MODULE_GENERAL_BROADCAST;
  msg.flags.extended = 1;               // to show extended id on CAN

  // Note: if you want to add in the same address, change the position in the array and it must be done in this function.
  //       if you want to add in a different address, add to the address 'msg.id' to offset the address.
  msg.buf[0] = heartFrame.bytes[HEART_COUNTER]++;
  msg.buf[1] = 20;


  Can0.write(msg);
	// digitalToggle(PC13);
  Serial.println(heartFrame.bytes[HEART_COUNTER]);
  // Serial.println("Heartbeat called.");
}

// read data from ADCs and broadcast as a message on CAN

// note: function can read data from ADCs and print to terminal
// note: function cannot broadcast on can yet.
 void readAdc() {

  for (int i = 0; i < 8; i++){
    digitalWrite(16, LOW);
    startManualConversions(i, 100);
    int16_t testData = readData();
    stopConversions();

    Serial.print("Channel :");
    Serial.println(i, HEX);
    Serial.print(" Voltage: ");

    // ADC_Voltages[i] = abs((testData*1.0/4096.0)*5.0); 
    // ADC_Voltages[1] = abs((testData*1.0/4096.0)*5.0);  
    // Serial.println(ADC_Voltages[i]); 
    Serial.println(testData); 
    digitalWrite(16, LOW);
    }
  // current_convert(ADC_Voltages);    // prints the current of the resistors connected to the temp module.
  //check_for_overcurrents();
}

void canTX() 
{
  CAN_message_t msg;
  msg.id = CAN_TEMP_MODULE_BMS_BROADCAST;
  msg.flags.extended = 1;               // to show extended id on CAN

  msg.buf[0] = 10;
  Can0.write(msg);
}

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}

void loop() {
  Can0.events();

  // static uint32_t timeout = millis();
  // if ( millis() - timeout > 200 ) {
  //   CAN_message_t msg;
  //   msg.id = 0x100;
  //   for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = i + 1;
  //   Can0.write(msg);
  //   timeout = millis();
  // }
  Heartbeat.update();
  Can_tx.update();
  readAdc();
  
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);

  // Enable all analogue inputs
 
  delay (50);
}
