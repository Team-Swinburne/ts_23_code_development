/*  TEAM SWINBURNE - TS23
    BRAKE MODULE - HARDWARE REVISION 5
    ETHAN JONES

    INFORMATION: This code controlls the Brake Module (hardware revision 5).
    The brake module monitors the brakes and BSPD signal and sends messages
    through CAN to notify other CAN nodes about the inputs.

    Potentiometers are used to set Low pressure and High pressure. (current calibration as of 04/07/2021) is
*/

#include <eXoCAN.h>
#include "TickerInterrupt.h"
#include "can_addresses.h"
#include "BrakeModule_info.h" //This header stores information about module and calibrations
#include "ads7028.h"
#include <SPI.h>

//void Serial_Print(); //Used for debugging

/* -------------------------------------------------------------------------- */
/*                                  PIN INIT                                  */
/* -------------------------------------------------------------------------- */

//DigitalIn LowPressure(PA_8); //high pressure
//DigitalIn HighPressure(PA_10); //low pressure
//DigitalIn CurrentSensor(PA_11); //current sensor 5KW
//DigitalIn BSPD(PB_14); //BSPD_OK (no delay)
//DigitalIn BSPD_Delay(PB_12); //BSPD_OK (10 second delay)

#define HIGH_PRESSURE_PIN PA_10
#define LOW_PRESSURE_PIN PA_8
#define CURRENT_SENSOR_PIN PA_11
#define BSPD_PIN PB_14
#define BSPD_DELAY_PIN PB_12

//DigitalOut debugLedOut(PC_13); //Debug LED

// CANBus Interface
eXoCAN can;
//CAN can1(PB_8, PB_9); //CANBUS

// //Instantiation of SPI
//SPI spi(PB_5, PB_4, PB_3);// MOSI, MISO, SCLK
//DigitalOut ADC_CS(PB_6);

//DigitalOut *CS_PIN;

/* -------------------------------------------------------------------------- */
/*                 Other functions added in to help this works                */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                             OBJECTS AND STRUCTS                            */
/* -------------------------------------------------------------------------- */

//Objects and structs
HeartBeat_struct HeartBeat; //Struct contains the variables used for the HeartBeat
BrakeModule_struct BrakeModule; //Struct contains the variables used for the BrakeModule
brake_calibration_s brake_calibration; //Struct contains the calibration variables

struct msgFrame
{
  uint8_t len = 8;
  uint8_t bytes[8] = {0};
};

static msgFrame  heartFrame, //{.len = 6},
                errorFrame,// {.len = 2}, 
        digitalFrame1, //{.len = 1},
                analogFrame1,
                analogFrame2,
                analogFrame3;

uint8_t rxData[8];

/* -------------------------------------------------------------------------- */
/*                               HANDY FUNCTIONS                              */
/* -------------------------------------------------------------------------- */
uint8_t raw_to_percent(float brake_raw, float brake_max, float brake_min)
{
  float brake_percent = ((brake_raw - brake_min)/(brake_max - brake_min))*100.0;

  // Filter data to ensure it is within 0-100 percent
  brake_percent = max(brake_percent - DEADZONE,brake_percent);
  brake_percent = ((brake_percent - DEADZONE) / (100 - DEADZONE)) * 100.0;
  brake_percent = min(brake_percent, 100.0f);
  return (uint8_t)brake_percent;
}

float toFloat(int16_t data)
{
  float voltage = (data*1.0/4096.0)*5.0;
  if (voltage > 0)
  {
    return voltage;
  }
  else
  {
    return 5+voltage;
  }
}

void BrakeModuleUpdate()
{
  digitalWrite(PB_6, LOW);
  startManualConversions(4, 100);
  int16_t testData = readData();
  stopConversions();
  digitalWrite(PB_6, HIGH);
  delay(10);
  float Brake1_Voltage = toFloat(testData);

  digitalWrite(PB_6, LOW);
  startManualConversions(2, 100);
  testData = readData();
  stopConversions();
  digitalWrite(PB_6, HIGH);
  delay(10);
  float Brake2_Voltage = toFloat(testData);

  digitalWrite(PB_6, LOW);
  startManualConversions(0, 100);
  testData = readData();
  stopConversions();
  digitalWrite(PB_6, HIGH);
  delay(10);
  float LowRef_Voltage = toFloat(testData);

  digitalWrite(PB_6, LOW);
  startManualConversions(3, 100);
  testData = readData();
  stopConversions();
  digitalWrite(PB_6, HIGH);
  delay(10);
  float HighRef1_Voltage = toFloat(testData);

  digitalWrite(PB_6, LOW);
  startManualConversions(1, 100);
  testData = readData();
  stopConversions();
  digitalWrite(PB_6, HIGH);
  delay(10);
  float HighRef2_Voltage = toFloat(testData);

  BrakeModule.brake1_raw      = Brake1_Voltage;
  BrakeModule.brake2_raw      = Brake2_Voltage;
  BrakeModule.High_Pressure   = digitalRead(HIGH_PRESSURE_PIN);
  BrakeModule.Low_Pressure    = digitalRead(LOW_PRESSURE_PIN);
  BrakeModule.five_kW         = digitalRead(CURRENT_SENSOR_PIN);
  BrakeModule.BSPD_OK         = digitalRead(BSPD_DELAY_PIN);
  BrakeModule.BSPD_OK_delay   = digitalRead(BSPD_DELAY_PIN);

  BrakeModule.brake1_percent    = raw_to_percent(BrakeModule.brake1_raw, brake_calibration.brake1_max, brake_calibration.brake1_min);
  BrakeModule.brake2_percent    = raw_to_percent(BrakeModule.brake2_raw, brake_calibration.brake2_max, brake_calibration.brake2_min);
  BrakeModule.brake_avg_percent = (BrakeModule.brake1_percent + BrakeModule.brake2_percent)/2.0;

  BrakeModule.brake_low_ref   = LowRef_Voltage;
  BrakeModule.brake_high_ref1  = HighRef1_Voltage;
  BrakeModule.brake_high_ref2  = HighRef2_Voltage;
}


/* -------------------------------------------------------------------------- */
/*                                  CALLBACKS                                 */
/* -------------------------------------------------------------------------- */
void CAN_brakeModule_TX_Heartbeat()
{
  (HeartBeat.Counter >= 255) ? HeartBeat.Counter = 0 : HeartBeat.Counter++;

  heartFrame.bytes[CAN_HEARTBEAT_STATE] = HeartBeat.State;
  heartFrame.bytes[CAN_HEARTBEAT_COUNTER] = HeartBeat.Counter;
  heartFrame.bytes[CAN_HEARTBEAT_PCB_TEMP] = 0;
  heartFrame.bytes[CAN_HEARTBEAT_HARDWARE_REVISION] = 5;

  can.transmit(CAN_BRAKE_MODULE_BASE_ADDRESS+TS_HEARTBEAT_ID, heartFrame.bytes, heartFrame.len);

  digitalToggle(PC13);
}

void CAN_brakeModule_TX_Digital_1()
{
  //Reverse so that (1 = True, 0 = False)
  digitalFrame1.bytes[CAN_DIGITAL_1_BRAKE_HIGH_PRESSURE] = !BrakeModule.High_Pressure;
  digitalFrame1.bytes[CAN_DIGITAL_1_BRAKE_LOW_PRESSURE] = !BrakeModule.Low_Pressure;
  
  digitalFrame1.bytes[CAN_DIGITAL_1_BRAKE_5KW] = BrakeModule.five_kW;
  
  digitalFrame1.bytes[CAN_DIGITAL_1_BRAKE_BSPD_OK] = !BrakeModule.BSPD_OK;
  digitalFrame1.bytes[CAN_DIGITAL_1_BRAKE_BSPD_OK_DELAY] = !BrakeModule.BSPD_OK_delay;

  can.transmit(CAN_BRAKE_MODULE_BASE_ADDRESS+TS_DIGITAL_1_ID, digitalFrame1.bytes, digitalFrame1.len);
}

void CAN_brakeModule_TX_Analog_1()
{
  analogFrame1.bytes[CAN_ANALOG_1_BRAKE1_PERCENT] = BrakeModule.brake1_percent;
  analogFrame1.bytes[CAN_ANALOG_1_BRAKE2_PERCENT] = BrakeModule.brake2_percent;
  analogFrame1.bytes[CAN_ANALOG_1_BRAKE_AVG_PERCENT] = BrakeModule.brake_avg_percent;
  
  can.transmit(CAN_BRAKE_MODULE_BASE_ADDRESS+TS_ANALOGUE_1_ID, analogFrame1.bytes, analogFrame1.len);
}

void CAN_brakeModule_TX_Analog_2()
{
  //Convert to integers with 1mV resolution
  int brake1_Voltage = uint16_t(BrakeModule.brake1_raw*1000);
  int brake2_Voltage = uint16_t(BrakeModule.brake2_raw*1000);

  analogFrame2.bytes[0] = (brake1_Voltage >> 8) & 0xFF;
  analogFrame2.bytes[1] = (brake1_Voltage) & 0xFF;

  analogFrame2.bytes[2] = (brake2_Voltage >> 8) & 0xFF;
  analogFrame2.bytes[3] = (brake2_Voltage) & 0xFF;

  can.transmit(CAN_BRAKE_MODULE_BASE_ADDRESS+TS_ANALOGUE_2_ID, analogFrame2.bytes, analogFrame2.len);
}

void CAN_brakeModule_TX_Analog_3()
{
  //Convert to integers with 1mV resolution
  int highRef1_Voltage = uint16_t(BrakeModule.brake_high_ref1*1000);
  int highRef2_Voltage = uint16_t(BrakeModule.brake_high_ref2*1000);
  int lowRef_Voltage = uint16_t(BrakeModule.brake_low_ref*1000);

  analogFrame3.bytes[0] = (highRef1_Voltage >> 8) & 0xFF;
  analogFrame3.bytes[1] = (highRef1_Voltage) & 0xFF;

  analogFrame3.bytes[2] = (highRef2_Voltage >> 8) & 0xFF;
  analogFrame3.bytes[3] = (highRef2_Voltage) & 0xFF;

  analogFrame3.bytes[4] = (lowRef_Voltage >> 8) & 0xFF;
  analogFrame3.bytes[5] = (lowRef_Voltage) & 0xFF;

  can.transmit(CAN_BRAKE_MODULE_BASE_ADDRESS+TS_ANALOGUE_3_ID, analogFrame3.bytes, analogFrame3.len);
}

void CAN_brakeModule_RX()
{
  can.rxMsgLen = can.receive(can.id, can.fltIdx, can.rxData.bytes);

  if (can.rxMsgLen > -1) 
  {
    if(can.id == CAN_BRAKE_MODULE_BASE_ADDRESS+TS_SETPOINT_1_ID)
    {
      can.rxMsgLen = -1;
      brake_calibration.brake1_min = (float(can.rxData.bytes[0])/10.0);
      brake_calibration.brake1_max = (float(can.rxData.bytes[1])/10.0);
      brake_calibration.brake2_min = (float(can.rxData.bytes[2])/10.0);
      brake_calibration.brake2_max = (float(can.rxData.bytes[3])/10.0);
    }
  }
}
/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */
void setup()
{
  delay(3000);

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

  SPI.setDataMode(SPI_MODE0);            // SPI_MODE0 - Low state when idle; SPI is sampled on rising edge of the clock.

  SPI.setClockDivider(SPI_CLOCK_DIV128); // System clock of Teens 4.0 is 600MHz; ADS7028 maximum clock frequency is 60MHz
                                         // Therefore, a divider of 10 or more is required
                                         // SPI clock divider only offers divisions (closest to 10) of 8 and 16
                                         // 16 was chosen for highest possible performance of the ADS7028

  SPI.setBitOrder(MSBFIRST);             // Reading 'Most Significant Bit FIRST' instead of 'Least Significant Bit FIRST'

  // Set pin assignments for MOSI, MISO, and SCLCK
  SPI.setMOSI(PB_5);
  SPI.setMISO(PB_4);
  SPI.setSCLK(PB_3);

  // SET UP ADC CONFIGURATION //
  initADS7028();  // initialise ADCs 

  delay(1000);   // allow time to settle

  // Configure chip mode
  for (int i = 0; i<8; i++){
      setChannelAsAnalogInput(i);
  } //Configure all pins as analog in
  delay(50);
  writeSingleRegister(OPMODE_CFG_ADDRESS, OPMODE_CFG_CONV_MODE_MANUAL_MODE); //Select Manual Mode
  delay(50);
  writeSingleRegister(SEQUENCE_CFG_ADDRESS, SEQUENCE_CFG_SEQ_MODE_MANUAL); //Manual Channel Selection Mode
  delay(50);

  can.begin(STD_ID_LEN, BR500K, PORTB_8_9_XCVR);   //11 Bit Id, 500Kbps
  can.filterMask16Init(0, 0x158, 0x7ff);

  can.attachInterrupt(CAN_brakeModule_RX);

  TickerInterrupt Ticker(TIM2,1);
  Ticker.start();
  Ticker.attach(CAN_brakeModule_TX_Heartbeat, 1000);
  Ticker.attach(CAN_brakeModule_TX_Digital_1, 1000);
  Ticker.attach(CAN_brakeModule_TX_Analog_1, 1000);
  Ticker.attach(CAN_brakeModule_TX_Analog_2, 1000);
  Ticker.attach(CAN_brakeModule_TX_Analog_3, 1000);

  pinMode(HIGH_PRESSURE_PIN, INPUT);
  pinMode(LOW_PRESSURE_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(BSPD_PIN, INPUT);
  pinMode(BSPD_DELAY_PIN, INPUT);

  pinMode(PC13, OUTPUT);

  //SPI
  pinMode(PB_5, OUTPUT);          // MOSI
  pinMode(PB_4, INPUT_PULLDOWN);  // MISO
  pinMode(PB_3, OUTPUT);          // SCLCK
  pinMode(PB_6, OUTPUT);

  delay(3000);
}
void loop()
{
  BrakeModuleUpdate();
}
