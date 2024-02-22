/*  TEAM SWINBURNE - TS23
    BRAKE MODULE - HARDWARE REVISION 5
    ETHAN JONES

    INFORMATION: This code controlls the Brake Module (hardware revision 5).
    The brake module monitors the brakes and BSPD signal and sends messages
    through CAN to notify other CAN nodes about the inputs.

    Potentiometers are used to set Low pressure and High pressure. (current calibration as of 04/07/2021) is
*/

#include <mbed.h>
#include <CAN.h>
#include "can_addresses.h"
#include "BrakeModule_info.h" //This header stores information about module and calibrations

//void Serial_Print(); //Used for debugging

/* -------------------------------------------------------------------------- */
/*                                  PIN INIT                                  */
/* -------------------------------------------------------------------------- */

DigitalIn LowPressure(PA_8); //high pressure
DigitalIn HighPressure(PA_10); //low pressure
DigitalIn CurrentSensor(PA_11); //current sensor 5KW
DigitalIn BSPD(PB_14); //BSPD_OK (no delay)
DigitalIn BSPD_Delay(PB_12); //BSPD_OK (10 second delay)

DigitalOut debugLedOut(PC_13); //Debug LED

AnalogIn sensor1(PA_4); //sensor 1
AnalogIn sensor2(PA_5); //sensor 2
AnalogIn lowRef(PB_1);
AnalogIn highRef(PB_0);

CAN can1(PB_8, PB_9); //CANBUS

// //Instantiation of SPI
SPI spi(PB_5, PB_4, PB_3);// MOSI, MISO, SCLK
DigitalOut ADC_CS(PB_6);

DigitalOut *CS_PIN;


/* -------------------------------------------------------------------------- */
/*                             OBJECTS AND STRUCTS                            */
/* -------------------------------------------------------------------------- */

//Objects and structs
CANMessage can1_msg; //Object that formats the CAN message
HeartBeat_struct HeartBeat; //Struct contains the variables used for the HeartBeat
BrakeModule_struct BrakeModule; //Struct contains the variables used for the BrakeModule
brake_calibration_s brake_calibration; //Struct contains the calibration variables

//Creates tickers
Ticker ticker_CAN_HeartBeat;
Ticker ticker_CAN_Error;
Ticker ticker_CAN_Digital_1;
Ticker ticker_CAN_Analog_1;
Ticker ticker_CAN_Analog_2;
Ticker ticker_CAN_Analog_3;

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

void BrakeModuleUpdate()
{
  // // startManualConversions(4, 100);
  // // int16_t testData = readData();
  // // stopConversions();
  // // float Brake1_Voltage = toFloat(testData);

  // // startManualConversions(2, 100);
  // // testData = readData();
  // // stopConversions();
  // // float Brake2_Voltage = toFloat(testData);

  // // startManualConversions(0, 100);
  // // testData = readData();
  // // stopConversions();
  // // float LowRef_Voltage = toFloat(testData);

  // // startManualConversions(3, 100);
  // // testData = readData();
  // // stopConversions();
  // // float HighRef1_Voltage = toFloat(testData);

  // // startManualConversions(1, 100);
  // // testData = readData();
  // // stopConversions();
  // // float HighRef2_Voltage = toFloat(testData);

  // BrakeModule.brake1_raw      = Brake1_Voltage;
  // BrakeModule.brake2_raw      = Brake2_Voltage;
  // BrakeModule.High_Pressure   = HighPressure.read();
  // BrakeModule.Low_Pressure    = LowPressure.read();
  // BrakeModule.five_kW         = CurrentSensor.read();
  // BrakeModule.BSPD_OK         = BSPD_Delay.read();//BSPD.read();
  // BrakeModule.BSPD_OK_delay   = BSPD_Delay.read();

  // BrakeModule.brake1_percent    = raw_to_percent(BrakeModule.brake1_raw, brake_calibration.brake1_max, brake_calibration.brake1_min);
  // BrakeModule.brake2_percent    = raw_to_percent(BrakeModule.brake2_raw, brake_calibration.brake2_max, brake_calibration.brake2_min);
  // BrakeModule.brake_avg_percent = (BrakeModule.brake1_percent + BrakeModule.brake2_percent)/2.0;

  // BrakeModule.brake_low_ref   = LowRef_Voltage;
  // BrakeModule.brake_high_ref1  = HighRef1_Voltage;
  // BrakeModule.brake_high_ref2  = HighRef2_Voltage;
}


/* -------------------------------------------------------------------------- */
/*                                  CALLBACKS                                 */
/* -------------------------------------------------------------------------- */
void CAN_brakeModule_TX_Heartbeat()
{
  (HeartBeat.Counter >= 255) ? HeartBeat.Counter = 0 : HeartBeat.Counter++;

  char TX_data[4] = { 0 };

  TX_data[CAN_HEARTBEAT_STATE] = HeartBeat.State;
  TX_data[CAN_HEARTBEAT_COUNTER] = HeartBeat.Counter;
  TX_data[CAN_HEARTBEAT_PCB_TEMP] = 0;
  TX_data[CAN_HEARTBEAT_HARDWARE_REVISION] = 5;

  if (can1.write(CANMessage((CAN_BRAKE_MODULE_BASE_ADDRESS + TS_HEARTBEAT_ID), TX_data, 4))){
    debugLedOut = !debugLedOut;
  }
  else {
    can1.reset();
  }
}

void CAN_brakeModule_TX_Digital_1()
{
  char TX_data[5] = { 0 };

  //Reverse so that (1 = True, 0 = False)
  TX_data[CAN_DIGITAL_1_BRAKE_HIGH_PRESSURE] = !BrakeModule.High_Pressure;
  TX_data[CAN_DIGITAL_1_BRAKE_LOW_PRESSURE] = !BrakeModule.Low_Pressure;

  TX_data[CAN_DIGITAL_1_BRAKE_5KW] = BrakeModule.five_kW;

  //Reverse so that (1 = OK)
  TX_data[CAN_DIGITAL_1_BRAKE_BSPD_OK] = !BrakeModule.BSPD_OK;
  TX_data[CAN_DIGITAL_1_BRAKE_BSPD_OK_DELAY] = !BrakeModule.BSPD_OK_delay;

  if (can1.write(CANMessage((CAN_BRAKE_MODULE_BASE_ADDRESS + TS_DIGITAL_1_ID), TX_data, 5))){
    debugLedOut = !debugLedOut;
  }
}

void CAN_brakeModule_TX_Analog_1()
{
  char TX_data[3] = { 0 };

  TX_data[CAN_ANALOG_1_BRAKE1_PERCENT] = BrakeModule.brake1_percent;
  TX_data[CAN_ANALOG_1_BRAKE2_PERCENT] = BrakeModule.brake2_percent;
  TX_data[CAN_ANALOG_1_BRAKE_AVG_PERCENT]     = BrakeModule.brake_avg_percent;
  
  if (can1.write(CANMessage((CAN_BRAKE_MODULE_BASE_ADDRESS + TS_ANALOGUE_1_ID), TX_data, 3))){
    debugLedOut = !debugLedOut;
  }
}

void CAN_brakeModule_TX_Analog_2()
{
  char TX_data[8] = { 0 };

  //Convert to integers with 1mV resolution
  int brake1_Voltage = uint16_t(BrakeModule.brake1_raw*1000);
  int brake2_Voltage = uint16_t(BrakeModule.brake2_raw*1000);

  TX_data[0] = (brake1_Voltage >> 8) & 0xFF;
  TX_data[1] = (brake1_Voltage) & 0xFF;

  TX_data[2] = (brake2_Voltage >> 8) & 0xFF;
  TX_data[3] = (brake2_Voltage) & 0xFF;

  if (can1.write(CANMessage((CAN_BRAKE_MODULE_BASE_ADDRESS + TS_ANALOGUE_2_ID), TX_data, 8))){
    debugLedOut = !debugLedOut;
  }
}

void CAN_brakeModule_TX_Analog_3()
{
  char TX_data[8] = { 0 };

  //Convert to integers with 1mV resolution
  int highRef1_Voltage = uint16_t(BrakeModule.brake_high_ref1*1000);
  int highRef2_Voltage = uint16_t(BrakeModule.brake_high_ref2*1000);
  int lowRef_Voltage = uint16_t(BrakeModule.brake_low_ref*1000);

  TX_data[0] = (highRef1_Voltage >> 8) & 0xFF;
  TX_data[1] = (highRef1_Voltage) & 0xFF;

  TX_data[2] = (highRef2_Voltage >> 8) & 0xFF;
  TX_data[3] = (highRef2_Voltage) & 0xFF;

  TX_data[4] = (lowRef_Voltage >> 8) & 0xFF;
  TX_data[5] = (lowRef_Voltage) & 0xFF;
  
  if (can1.write(CANMessage((CAN_BRAKE_MODULE_BASE_ADDRESS + TS_ANALOGUE_3_ID), TX_data, 8))){
    debugLedOut = !debugLedOut;
  }
}

void CAN_brakeModule_RX()
{
  if (can1.read(can1_msg))
  {
    switch(can1_msg.id)
    {
      case (CAN_BRAKE_MODULE_BASE_ADDRESS+TS_SETPOINT_1_ID):
        brake_calibration.brake1_min = (float(can1_msg.data[0])/10.0);
        brake_calibration.brake1_max = (float(can1_msg.data[1])/10.0);
        brake_calibration.brake2_min = (float(can1_msg.data[2])/10.0);
        brake_calibration.brake2_max = (float(can1_msg.data[3])/10.0);
        break; 
    }
  }
}

/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */
int main() 
{

  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8,0);
  spi.frequency(100000);
  
  // Disable interrupts for smooth startup routine.
	wait_us(3000*1000);

  ADC_CS = 1;

  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8,0);
  spi.frequency(100000);
	
	__disable_irq();

  can1.frequency(CANBUS_FREQUENCY);
  can1.filter(CAN_BRAKE_MODULE_BASE_ADDRESS+TS_SETPOINT_1_ID, 0xFFF, CANStandard, 0); // set filter #0 to accept only standard messages with ID == RX_ID
	can1.attach(&CAN_brakeModule_RX);

  //Configure tickers
  ticker_CAN_HeartBeat.attach(&CAN_brakeModule_TX_Heartbeat, CAN_HEARTBEAT_PERIOD);
  ticker_CAN_Digital_1.attach(&CAN_brakeModule_TX_Digital_1, CAN_DIGITAL_1_PERIOD);
  ticker_CAN_Analog_1.attach(&CAN_brakeModule_TX_Analog_1, 0.2);
  ticker_CAN_Analog_2.attach(&CAN_brakeModule_TX_Analog_2, 0.2);
  ticker_CAN_Analog_3.attach(&CAN_brakeModule_TX_Analog_3, 0.2);

  // Re-enable interrupts again, now that interrupts are ready.
	__enable_irq();

	// Allow some time to settle!
	wait_us(3000*1000);

  while(1) 
  {
    BrakeModuleUpdate();
  }

  return 0;
}

