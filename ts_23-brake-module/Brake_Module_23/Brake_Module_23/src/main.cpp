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

//DigitalIn LowPressure(PA_6); //high pressure
//DigitalIn HighPressure(PA_7); //low pressure
//DigitalIn CurrentSensor(PA_8); //current sensor 5KW
//DigitalIn BSPD(PA_9); //BSPD_OK (no delay)
//DigitalIn BSPD_Delay(PB_12); //BSPD_OK (10 second delay)

DigitalOut debugLedOut(PA_7); //Debug LED

//AnalogIn sensor1(PA_4); //sensor 1
//AnalogIn sensor2(PA_5); //sensor 2
//AnalogIn lowRef(PB_1);
//AnalogIn highRef(PB_0);

CAN can1(PB_8, PB_9); //CANBUS

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



/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */
int main() 
{
  // Disable interrupts for smooth startup routine.
	wait_ms(1000);
	
	__disable_irq();

  can1.frequency(CANBUS_FREQUENCY);
  can1.filter(CAN_BRAKE_MODULE_BASE_ADDRESS+TS_SETPOINT_1_ID, 0xFFF, CANStandard, 0); // set filter #0 to accept only standard messages with ID == RX_ID
  //can1.attach(&CAN_brakeModule_RX);

  //Configure tickers
  ticker_CAN_HeartBeat.attach(&CAN_brakeModule_TX_Heartbeat, CAN_HEARTBEAT_PERIOD);
  //ticker_CAN_Digital_1.attach(&CAN_brakeModule_TX_Digital_1, CAN_DIGITAL_1_PERIOD);
  //ticker_CAN_Analog_1.attach(&CAN_brakeModule_TX_Analog_1, CAN_ANALOG_1_PERIOD);
  //ticker_CAN_Analog_2.attach(&CAN_brakeModule_TX_Analog_2, CAN_ANALOG_1_PERIOD);

  // Re-enable interrupts again, now that interrupts are ready.
	__enable_irq();

	// Allow some time to settle!
	wait_ms(1000);

  //while(1) 
  //{
    //BrakeModuleUpdate();
    //Serial_Print("ALIVE"); //Used for debugging.
  //}

  return 0;
}