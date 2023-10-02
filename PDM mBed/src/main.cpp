#include "mbed.h"
#include <CAN.h>
#include "can_addresses.h"
#include "PDM_info.h"


//Instantiation of CAN
CAN can1(PB_8, PB_9); //CANBUS

//Instantiation of Serial
Serial device(PA_9, PA_10);  // tx, rx

//Instantiation of SPI
SPI spi(PB_5, PB_4, PB_3);// MOSI, MISO, SCLK

/*----------------------------------------------------------------------------*/
/*                               PIN ASSIGNMENTS:                             */
/*----------------------------------------------------------------------------*/
//DebugLed
DigitalOut debugLedOut(PC_13); //Debug LED

//PWM Outputs
//DigitalOut PWM_1(PA_4); //PWM_1
//DigitalOut PWM_2(PA_3); //PWM_2
//DigitalOut PWM_3(PA_2); //PWM_3
//DigitalOut PWM_4(PA_1); //PWM_4

//SPI Chip Select
DigitalOut ADC_CS(PB_15); //CS_1 - ADC_CS
DigitalOut SMRT_DRV_CTRL_CS(PB_14); //CS_2 - SMRT_DRV_CTRL_CS
DigitalOut SMRT_DRV_DIG_CS(PB_12); //CS_3 - SMRT_DRV_DIG_CS
//DigitalOut CS_4(PB_13); //CS_4 - Unused
DigitalOut H_BRIDGE_A_CS(PB_11); //CS_5 - H_BRIDGE_A_CS
DigitalOut H_BRIDGE_B_CS(PB_10); //CS_6 - H_BRIDGE_B_CS
DigitalOut H_BRIDGE_C_CS(PA_6); //CS_7 - H_BRIDGE_C_CS
DigitalOut H_BRIDGE_D_CS(PA_5); //CS_8 - H_BRIDGE_D_CS

//CAN TX LEDs
//DigitalOut CAN_TX_LED(PB_0); //CAN_TX_LED for indicating transmission
//DigitalOut CAN_RX_LED(PB_1); //CAN_RX_LED for indicating receiving

/* -------------------------------------------------------------------------- */
/*                             OBJECTS AND STRUCTS                            */
/* -------------------------------------------------------------------------- */

//Objects and structs
CANMessage can1_msg; //Object that formats the CAN message
HeartBeat_struct HeartBeat; //Struct contains the variables used for the HeartBeat

//Creates tickers
Ticker ticker_CAN_HeartBeat; //Used to know the PCB is functioning and transmitting
//Ticker ticker_CAN_Error; //Used to troubleshoot errors
//Ticker ticker_CAN_Digital_1; //This will transmit Circuit Status (On - 255, Off - 0, and PWM value 0-255)
//Ticker ticker_CAN_Analog_1; //This will transmit current (or maybe just voltage from the ADCs)
Ticker ticker_SPI_Transmit;


/* -------------------------------------------------------------------------- */
/*                                  CALLBACKS                                 */
/* -------------------------------------------------------------------------- */

uint8_t message = 0x00;

void CAN_PDM_TX_Heartbeat()
{
  (HeartBeat.Counter >= 255) ? HeartBeat.Counter = 0 : HeartBeat.Counter++;

  char TX_data[4] = { 0 };

  TX_data[CAN_HEARTBEAT_STATE] = HeartBeat.State;
  TX_data[CAN_HEARTBEAT_COUNTER] = HeartBeat.Counter;
  TX_data[CAN_HEARTBEAT_PCB_TEMP] = 0;
  TX_data[CAN_HEARTBEAT_HARDWARE_REVISION] = message;

  if (can1.write(CANMessage((CAN_PDM_BASE_ADDRESS + TS_HEARTBEAT_ID), TX_data, 4))){
    debugLedOut = !debugLedOut;
  }
  else {
    can1.reset();
  }
}

void SPITest()
{
  SMRT_DRV_DIG_CS = 0;
  message = spi.write(0x01);
  SMRT_DRV_DIG_CS = 1;

}

void CAN_brakeModule_RX()
{
  if (can1.read(can1_msg))
  {
    switch(can1_msg.id)
    {
      case (CAN_BRAKE_MODULE_BASE_ADDRESS+TS_SETPOINT_1_ID):
            //device.printf("Receiving CAN");
        break; 
    }
  }
}

int main() 
{
    //All SPI Chips deselected
    ADC_CS = 1;
    SMRT_DRV_CTRL_CS = 1;
    SMRT_DRV_DIG_CS = 1;
    //CS_4 = 1;
    H_BRIDGE_A_CS = 1;
    H_BRIDGE_B_CS = 1;
    H_BRIDGE_C_CS = 1;
    H_BRIDGE_D_CS = 1;  

    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8,0);
    spi.frequency(100000);


  // Disable interrupts for smooth startup routine.
	wait_ms(1000);
	
	__disable_irq();

    device.baud(9600);
    device.printf("Hello World\n");

  can1.frequency(CANBUS_FREQUENCY);
  can1.filter(CAN_PDM_BASE_ADDRESS+TS_SETPOINT_1_ID, 0xFFF, CANStandard, 0); // set filter #0 to accept only standard messages with ID == RX_ID
	can1.attach(&CAN_brakeModule_RX);

  //Configure tickers
  ticker_CAN_HeartBeat.attach(&CAN_PDM_TX_Heartbeat, CAN_HEARTBEAT_PERIOD);
  //ticker_CAN_Digital_1.attach(&CAN_brakeModule_TX_Digital_1, CAN_DIGITAL_1_PERIOD);
  //ticker_CAN_Analog_1.attach(&CAN_brakeModule_TX_Analog_1, CAN_ANALOG_1_PERIOD);
  //ticker_CAN_Analog_2.attach(&CAN_brakeModule_TX_Analog_2, CAN_ANALOG_1_PERIOD);
  ticker_SPI_Transmit.attach(&SPITest, 0.001);
  // Re-enable interrupts again, now that interrupts are ready.
	__enable_irq();

	// Allow some time to settle!
	wait_ms(1000);

  while(1) 
  {    
    //Serial_Print(); //Used for debugging.
    //device.printf("Hello World\n");
    //SMRT_DRV_DIG_CS = 0;
    //message = spi.write(0x01);
    //SMRT_DRV_DIG_CS = 1;
    device.printf("WHOAMI register = 0x%X\n", message);
      SMRT_DRV_DIG_CS = 0;
      message = spi.write(0x00);
      SMRT_DRV_DIG_CS = 1;
  }

  return 0;
}

