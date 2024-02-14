/*  TEAM SWINBURNE - TS23 
    Aero pressure sensor board - HARDWARE REVISION 1
    Ben O'Mahony

    INFORMATION: This code sends the voltages produced by the air pressure sensors via the CAN.
    
    REVISION 0.0 (19/05/2023)
    Revision     Date          Comments
    --------   ----------     ------------
    0.0        19/05/2023       mbed code
*/

/*---------------------------------------------------------------------------------------------------*/
/*                                         Librarys                                                  */
/*---------------------------------------------------------------------------------------------------*/

#include <mbed.h>

/*---------------------------------------------------------------------------------------------------*/
/*                                          GPIOS                                                    */
/*---------------------------------------------------------------------------------------------------*/

DigitalOut Indicator_LED(LED1); // Heartbeat LED
BusOut TopMultiplexer(p5, p6, p7); // Multiplexer selction
AnalogIn ADC1(p15); // Top multiplexer Voltage
AnalogIn ADC2(p16); // Bottom multiplexer Voltage

CAN can1(p9, p10); //CANBUS

/*---------------------------------------------------------------------------------------------------*/
/*                                          Objects and Structs                                      */
/*---------------------------------------------------------------------------------------------------*/
/* Ticker WaitTime1 Trigers the heatbeat CAN transmition
   Tickers WaitTime2 through 5 all triger the reading of sensor values
   Ticker Ticker_HeartBeat Trigers the onboard LED light to flash     */

Ticker WaitTime1;
Ticker WaitTime2;
Ticker WaitTime3;
Ticker WaitTime4;
Ticker WaitTime5;
Ticker Ticker_HeartBeat;

/*---------------------------------------------------------------------------------------------------*/
/*                                         Variables and constants                                   */
/*---------------------------------------------------------------------------------------------------*/

#define canID 0x400 // base CAN ID messages will be sent on
int counter = 0; // counter for heartbeat
volatile bool SendCan1 = false; // become true on ticker
volatile bool SendCan2 = false; // become true on ticker
volatile bool SendCan3 = false; // become true on ticker
volatile bool SendCan4 = false; // become true on ticker
volatile bool heatbeat_cantransmit = false; // become true on ticker
int Top_PressureSensor_Voltage = 0; // voltage from top multiplexer
int  Bottom_PressureSensor_Voltage = 0; // voltage from bottom multiplexer
char TX_data1[8] = { 0 }; // 
char TX_data2[8] = { 0 };
char TX_data3[8] = { 0 };
char TX_data4[8] = { 0 };
int multiplexervalue = 0; // value sent to multiplexers
char a[8] = { 0 };


void CanDaemon()
{
    if(1)
    {

    }
    else 
    {

    }

    //can1.reset();
    //char counter = 0;

    //can1.write(CANMessage(0x100, &counter, 1));
}

void TransmitCan1()
{
    SendCan1 = true;
}

void TransmitCan2()
{
    SendCan2 = true;
}

void TransmitCan3()
{
    SendCan3 = true;
}

void TransmitCan4()
{
    SendCan4 = true;
}

void heartbeat_transmit()
{
  heatbeat_cantransmit = true;
}

//void Heartbeat()
//{
    //Indicator_LED = !Indicator_LED;
//}

int main() 
{

  can1.frequency(500000);

  // put your setup code here, to run once:

  WaitTime1.attach(&heartbeat_transmit, 1000ms);
  WaitTime2.attach(&TransmitCan1, 1000ms);
  WaitTime3.attach(&TransmitCan2, 1000ms);
  WaitTime4.attach(&TransmitCan3, 1000ms);
  WaitTime5.attach(&TransmitCan4, 1000ms);
  

  //Ticker_HeartBeat.attach(&Heartbeat, 100ms);

  while(1) 
  {
    // put your main code here, to run repeatedly:
    if(SendCan1)
    {
      int x = 0;
      multiplexervalue = 0;
        for(int i = 0; i < 4; i++)  {
          TopMultiplexer.write(multiplexervalue);
          //BottomMultiplexer.write(multiplexervalue);
          Top_PressureSensor_Voltage = 10000*3.3*ADC1.read();
          Bottom_PressureSensor_Voltage = 10000*3.3*ADC2.read();
          TX_data1[x] = (Top_PressureSensor_Voltage >> 8);
          x = x + 1;
          TX_data1[x] = (Top_PressureSensor_Voltage & 0xFF);
          x = x + 1;
          multiplexervalue = multiplexervalue + 1;
          
        }
        //TX_data4 = TX_data3;
        //can1.write(CANMessage(0x100, &counter, 1));
        can1.write(CANMessage(canID + 4, TX_data1, 8));
    
        SendCan1 = false;
    }

    if(SendCan2)
    {
      int x = 0;
      multiplexervalue = 0;
        for(int i = 0; i < 4; i++)  {
          TopMultiplexer.write(multiplexervalue);
          //BottomMultiplexer.write(multiplexervalue);
          Top_PressureSensor_Voltage = 10000*3.3*ADC1.read();
          Bottom_PressureSensor_Voltage = 10000*3.3*ADC2.read();
          TX_data2[x] = (Bottom_PressureSensor_Voltage >> 8);
          x = x + 1;
          TX_data2[x] = (Bottom_PressureSensor_Voltage & 0xFF);
          x = x + 1;
          multiplexervalue = multiplexervalue + 1;
          
        }
        //TX_data4 = TX_data3;
        //can1.write(CANMessage(0x100, &counter, 1));
        can1.write(CANMessage(canID + 5, TX_data2, 8)); 
    
        SendCan2 = false;
    }

    if(SendCan3)
    {
        int x = 0;
        multiplexervalue = 4;

        for(int i = 0; i < 4; i++)  {
          TopMultiplexer.write(multiplexervalue);
          //BottomMultiplexer.write(multiplexervalue);
          Top_PressureSensor_Voltage = 10000*3.3*ADC1.read();
          Bottom_PressureSensor_Voltage = 10000*3.3*ADC2.read();
          TX_data3[x] = (Top_PressureSensor_Voltage >> 8);
          x = x + 1;
          TX_data3[x] = (Top_PressureSensor_Voltage & 0xFF);
          x = x + 1;
          multiplexervalue = multiplexervalue + 1;
          
        }

      can1.write(CANMessage(canID + 6, TX_data3, 8));

      SendCan3 = false;
    }
    if(SendCan4)
    {
        int x = 0;
        multiplexervalue = 4;

        for(int i = 0; i < 4; i++)  {
          TopMultiplexer.write(multiplexervalue);
          //BottomMultiplexer.write(multiplexervalue);
          Top_PressureSensor_Voltage = 10000*3.3*ADC1.read();
          Bottom_PressureSensor_Voltage = 10000*3.3*ADC2.read();
          TX_data4[x] = (Bottom_PressureSensor_Voltage >> 8);
          x = x + 1;
          TX_data4[x] = (Bottom_PressureSensor_Voltage & 0xFF);
          x = x + 1;
          multiplexervalue = multiplexervalue + 1;
          
        }

      can1.write(CANMessage(canID + 7, TX_data4, 8));

      SendCan4 = false;
    }
    if(heatbeat_cantransmit)
    {
      char stattus[2] = { 0 };
      stattus[0] = 0;
      stattus[1] = counter;
      can1.write(CANMessage(canID, stattus, 2));
      counter++;
      Indicator_LED = !Indicator_LED;
      heatbeat_cantransmit = false;
    }
  }
}

//  Chn Identifier Fly   DLC  D0...1...2...3...4...5...6..D7
//  0          124         2 stat-count
//  0          128         8 --U23-- --U21-- --U19-- --U17--
//  0          129         8 --U10-- --U12-- --U14-- --U16--
//  0          130         8 --U09-- --U11-- --U13-- --U15--
//  0          131         8 --U24-- --U22-- --U20-- --U18--