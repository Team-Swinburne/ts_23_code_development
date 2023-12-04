// #include "mbed.h"
// #include "MCP2515JT.h"

// //Define SPI pins for your microcontroller
// SPI spi(PB_5, PB_4, PB_3);    // mosi, miso, sclk for mbed NXP1768 MCU

// DigitalOut cs(PA_2);      //Define MCP2515 chip-select pin


// //Set bit-rate timing values for your crystal or oscillator frequency
// //Use either Intrepid Control Systems: http://www.intrepidcs.com/support/mbtime.htm, or
// //Seeed Studio: http://www.seeedstudio.com/wiki/CAN-BUS_Shield_V1.2.
// //Value for 125 kbits/sec calculated for 10-MHz crystal
// //YOU MUST CALCULATE YOUR OWN VALUES!!!
 
// void baudConfig(int bitRate)//sets bitrate for MCP2515 node
// {
//     byte config0 = 0x00;
//     byte config1 = 0x00;
//     byte config2 = 0x00;
 
//     switch (bitRate)
//     {
// case 10:
//         config0 = 0x00;
//         config1 = 0x00;
//         config2 = 0x00;
//         break;
 
// case 20:
//         config0 = 0x00;
//         config1 = 0x00;
//         config2 = 0x00;
//         break;
 
// case 50:
//         config0 = 0x00;
//         config1 = 0x00;
//         config2 = 0x00;
//         break;
 
// case 100:
//         config0 = 0x00;
//         config1 = 0x00;
//         config2 = 0x00;
//         break;
 
// case 125:
//         config0 = 0x01;     //Trial for SEED Studios calc values
//         config1 = 0xBA;     //Titus 10-MHz crystal
//         config2 = 0x07;
//         break;

// case 250:
//         config0 = 0x00;
//         config1 = 0x00;
//         config2 = 0x00;
//         break;
 
// case 500:
//         config0 = 0x00;
//         config1 = 0x90;
//         config2 = 0x82;
//         break;
        
// case 1000:
//     //1 megabit mode added by Patrick Cruce(pcruce_at_igpp.ucla.edu)
//     //Faster communications enabled by shortening bit timing phases(3 Tq. PS1 & 3 Tq. PS2)
//     //Note that this may exacerbate errors due to synchronization or arbitration.
//     config0 = 0x00;
//     config1 = 0x00;
//     config2 = 0x00;
//     }
    
//     cs = 0;                 //Change MCP2515 chip-select to enable chip
//     wait_ms(10);            //10-millisec wait
//     spi.write(WRITE);       //Send SPI Write command
//     spi.write(CNF0);        //Identify CNF1 timing register
//     spi.write(config0);     //Send CNF1 value     
//     wait_ms(10);            //10-millisec wait
//     cs = 1;                 //Change MCP2515 chip-select to disable chip
//     wait_ms(10);            //10-millisec wait
 
//     cs = 0;
//     wait_ms(10);
//     spi.write(WRITE);
//     spi.write(CNF1);
//     spi.write(config1);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
 
//     cs = 0;
//     wait_ms(10);
//     spi.write(WRITE);
//     spi.write(CNF2);
//     spi.write(config2);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
// }
 
// //Set one of five configuration modes for MCP2515
// //Method added to enable testing in loopback mode.(pcruce_at_igpp.ucla.edu)
// void setMode(MCP2515Mode mode)
//      { 
//     byte writeVal = 0x00;
//     byte mask = 0x00;
 
//     switch(mode) {
//       case CONFIGURATION:
//             writeVal = 0x80;
//             break;
//       case NORMAL:
//             writeVal = 0x00;
//             break;
//       case SLEEP:
//             writeVal = 0x20;
//             break;
//     case LISTEN:
//             writeVal = 0x60;
//             break;
//       case LOOPBACK:
//             writeVal = 0x40;
//             break;
//    }
 
//     mask = 0xE0;        //Mask allows changes only to three REQOP bits in CANCTRL register
 
//     cs = 0;
//     spi.write(BIT_MODIFY);
//     spi.write(MCP2515CTRL);
//     spi.write(mask);
//     spi.write(writeVal);
//     cs = 1;
 
// }
 
// //Transmit Buffer 0
// void send_0()
// {
//      cs = 0;
//     spi.write(SEND_TX_BUF_0);
//     cs = 1;
// }

// //Transmit Buffer 1
// void send_1()
// {
//     cs = 0;
//     spi.write(SEND_TX_BUF_1);
//     cs = 1;
// }
 
// //Transmit Buffer 2
// void send_2()
// {
//     cs = 0;
//     spi.write(SEND_TX_BUF_2);
//     cs = 1;
// }
 
// //Read ID in Receiver Buffer 0
// char readID_0()
// {
    
//     char retVal;
//     cs = 0;
//     wait_ms(10);
//     spi.write(READ_RX_BUF_0_ID);
//     retVal = spi.write(0xFF);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
//     return retVal;
// }

// //Read ID in Receiver Buffer 1 
// char readID_1()
// {
//     char retVal;
//     cs = 0;
//     wait_ms(10);
//     spi.write(READ_RX_BUF_1_ID);
//     retVal = spi.write(0xFF);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
//     return retVal;
// }

// //Read Data in Receiver Buffer 0
// char readDATA_0()
// {
//     char retVal;
//     cs = 0;
//     wait_ms(10);
//     spi.write( READ_RX_BUF_0_DATA);
//     retVal = spi.write(0xFF);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
//     return retVal;
// }

// //Read Data in Receiver Buffer 1
// char readDATA_1()
// {
//     char retVal;
//     cs = 0;
//     wait_ms(10);
//     spi.write( READ_RX_BUF_1_DATA);
//     retVal = spi.write(0xFF);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
//     return retVal;
// }

//Extending MCP2515 data read for full frames(pcruce_at_igpp.ucla.edu)
//It is the responsibility of the user to allocate memory for output.
//If you don't know what length the bus frames will be, data_out array should be 8-bytes

//Full-Frame Data Read for Receiver Buffer 0
// void readDATA_ff_0(byte* length_out,byte *data_out,unsigned short *id_out){
 
//     byte len,i;
//     unsigned short id_h,id_l;
 
//     cs = 0;
//     spi.write(READ_RX_BUF_0_ID);
//     id_h = (unsigned short) spi.write(0xFF); //id high
//     id_l = (unsigned short) spi.write(0xFF); //id low
//     spi.write(0xFF); //extended id high(unused)
//     spi.write(0xFF); //extended id low(unused)
//     len = (spi.write(0xFF) & 0x0F); //data length code
//     for (i = 0;i<len;i++) {
//         data_out[i] = spi.write(0xFF);
//     }
//     cs = 1;
//     (*length_out) = len;
//     (*id_out) = ((id_h << 3) + ((id_l & 0xE0) >> 5)); //repack identifier
    
// }
 
// //Full-Frame Data Read for Receiver Buffer 1
// void readDATA_ff_1(byte* length_out,byte *data_out,unsigned short *id_out){
 
//     byte id_h,id_l,len,i;
 
//     cs = 0;
//     spi.write(READ_RX_BUF_1_ID);
//     id_h = spi.write(0xFF); //id high
//     id_l = spi.write(0xFF); //id low
//     spi.write(0xFF); //extended id high(unused)
//     spi.write(0xFF); //extended id low(unused)
//     len = (spi.write(0xFF) & 0x0F); //data length code
//     for (i = 0;i<len;i++) {
//         data_out[i] = spi.write(0xFF);
//     }
//     cs = 1;
 
//     (*length_out) = len;
//     (*id_out) = ((((unsigned short) id_h) << 3) + ((id_l & 0xE0) >> 5)); //repack identifier
// }
 
// //Added method to read status register
// //MCP2515 may be used to determine whether a frame was received.
// //(readStatus() & 0x80) == 0x80 means frame in buffer 0
// //(readStatus() & 0x40) == 0x40 means frame in buffer 1
// byte readStatus() 
// {
//     byte retVal;
//     cs = 0;
//     spi.write(READ_STATUS);
//     retVal = spi.write(0xFF);
//     cs = 1;
//     return retVal;
 
// }

// //Load ID and DATA into Transmit Buffer 0 
// void load_0(byte identifier, byte data)
// {
//     cs = 0;
//     wait_ms(10);
//     spi.write(LOAD_TX_BUF_0_ID);
//     spi.write(identifier);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
 
//     cs = 0;
//     wait_ms(10);
//     spi.write(LOAD_TX_BUF_0_DATA);
//     spi.write(data);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
// }
 
// //Load ID and DATA into Transmit Buffer 1
// void load_1(byte identifier, byte data)
// {
//     cs = 0;
//     wait_ms(10);
//     spi.write(LOAD_TX_BUF_1_ID);
//     spi.write(identifier);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
 
//     cs = 0;
//     wait_ms(10);
//     spi.write(LOAD_TX_BUF_1_DATA);
//     spi.write(data);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
// }

// //Load ID and DATA into Transmit Buffer 2 
// void load_2(byte identifier, byte data)
// {
//     cs = 0;
//     wait_ms(10);
//     spi.write(LOAD_TX_BUF_2_ID);
//     spi.write(identifier);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
 
//     cs = 0;
//     wait_ms(10);
//     spi.write(LOAD_TX_BUF_2_DATA);
//     spi.write(data);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
// }
 
// //Load Full Frame into Transmit Buffer 0
// void load_ff_0(byte length,unsigned short identifier,byte *data)
// {   
//     byte i,id_high,id_low;
 
//     //generate id bytes before spi write
//     id_high = (byte) (identifier >> 3);
//     id_low = (byte) ((identifier << 5) & 0x00E0);
 
//     cs = 0;
//     spi.write(LOAD_TX_BUF_0_ID);
//     spi.write(id_high);         //identifier high bits
//     spi.write(id_low);          //identifier low bits
//     spi.write(0x00);            //extended identifier registers(unused)
//     spi.write(0x00);
//     spi.write(length);
//     for (i=0;i<length;i++)
//     {                           //load data buffer
//         spi.write(data[i]);
//     }
 
//     cs = 1;
 
// }

// //Load Full Frame into Transmit Buffer 1
// void load_ff_1(byte length,unsigned short identifier,byte *data)
// {
    
//     byte i,id_high,id_low;
 
//     //generate id bytes before spi write
//     id_high = (byte) (identifier >> 3);
//     id_low = (byte) ((identifier << 5) & 0x00E0);
 
//     cs = 0;
//     spi.write(LOAD_TX_BUF_1_ID);
//     spi.write(id_high);         //identifier high bits
//     spi.write(id_low);          //identifier low bits
//         spi.write(0x00);        //extended identifier registers(unused)
//     spi.write(0x00);
//     spi.write(length);
//     for (i=0;i<length;i++)
//     {                           //load data buffer
//         spi.write(data[i]);
//     }
 
//     cs = 1;
// }

// //Load Full Frame into Transmit Buffer 2 
// void load_ff_2(byte length,unsigned short identifier,byte *data)
// {   
//     byte i,id_high,id_low;
 
//     //generate id bytes before spi write
//     id_high = (byte) (identifier >> 3);
//     id_low = (byte) ((identifier << 5) & 0x00E0);
 
//     cs = 0;
 
//     spi.write(LOAD_TX_BUF_2_ID);
//     spi.write(id_high);         //identifier high bits
//     spi.write(id_low);          //identifier low bits
//     spi.write(0x00);            //extended identifier registers(unused)
//     spi.write(0x00);
//     spi.write(length);          //data length code
//     for (i=0;i<length;i++)
//     {                           //load data buffer
//         spi.write(data[i]);
//     }
 
//     cs = 1;
// }
 
// //------------------------------------------------------------------------------

// //Write a byte to a register
// void writeRegister(byte address, byte data)
// {
//     cs = 0;
//     wait_ms(10);
//     spi.write(WRITE);
//     spi.write(address);
//     spi.write(data);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
// }

// //Read a byte from a register
// void readRegister(byte address, byte *data_out)
// {
//     cs = 0;
//     wait_ms(10);
//     spi.write(READ);
//     spi.write(address);
//     *data_out = spi.write(0xFF);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
// }

// //Return the RXB0 Standard ID as a single hex value
// unsigned short readSDI0_asHex()
// {
//     unsigned short retVal;
//     unsigned short HiAddr;
//     unsigned short LoAddr;
//     cs = 0;
//     //wait_ms(10);
//     spi.write(READ_RX_BUF_0_ID);
//     HiAddr = (unsigned short) spi.write(0xFF);
//     LoAddr = (unsigned short) spi.write(0xFF);
//     HiAddr = HiAddr <<3;
//     LoAddr = LoAddr >>5;
//     retVal = HiAddr | LoAddr;
//     cs = 1;
//     return retVal;
// }

// //Return the RXB1 Standard ID as a single hex value
// unsigned short readSDI1_asHex()
// {
//     unsigned short retVal;
//     unsigned short HiAddr;
//     unsigned short LoAddr;
//     cs = 0;
//     //wait_ms(10);
//     spi.write(READ_RX_BUF_1_ID);
//     HiAddr = (unsigned short) spi.write(0xFF);
//     LoAddr = (unsigned short) spi.write(0xFF);
//     HiAddr = HiAddr <<3;
//     LoAddr = LoAddr >>5;
//     retVal = HiAddr | LoAddr;
//     cs = 1;
//     return retVal;
// } 

// //Alternate MCP2515 Reset operation
// void reset()
// {
//     cs = 0;
//     wait_ms(10);
//     spi.write(RESET_REG);
//     wait_ms(10);
//     cs = 1;
//     wait_ms(10);
// }

// //Read the Receiver Status Byte 
// byte readRXStatus()
// {
//     byte retVal;
//     cs = 0;
//     spi.write(RX_STATUS);
//     retVal = spi.write(0xFF);
//     cs = 1;
//     return retVal;
// }

// //Bit-Modify function.  Address for a register, mask byte identifies bits to change
// //with a 1, unchanged bits identified with a 0.  Set bits you want changed to their
// //new state in data byte. 
// void bitModify(byte address, byte mask, byte data)
// {
//     cs = 0;
//     spi.write(BIT_MODIFY);
//     spi.write(address);
//     spi.write(mask);
//     spi.write(data);
//     cs = 1;
// }

// //Masks and filter routines for Standard 11-bit addresses
// //Set address for a Mask
// void setMask(unsigned short identifier)
// {
//     setMask_0(identifier);
//     setMask_1(identifier);
// }

// //Write a mask into Mask 0 register 
// void setMask_0(unsigned short identifier)
// {
//     writeRegister(RXM0SIDH, (byte)(identifier>>3));
//     writeRegister(RXM0SIDL, (byte)(identifier<<5));
// }

// //Write a mask into Mask 1 register 
// void setMask_1(unsigned short identifier)
// {
//     writeRegister(RXM1SIDH, (byte)(identifier>>3));
//     writeRegister(RXM1SIDL, (byte)(identifier<<5));
// }

// //Write an 11-bit Standard ID into Filter 0
// void setFilter_0(unsigned short identifier)
// {
//     writeRegister(RXF0SIDH, (byte)(identifier>>3));
//     writeRegister(RXF0SIDL, (byte)(identifier<<5));
// }

// //Write an 11-bit Standard ID into Filter 1
// void setFilter_1(unsigned short identifier)
// {
//     writeRegister(RXF1SIDH, (byte)(identifier>>3));
//     writeRegister(RXF1SIDL, (byte)(identifier<<5));
// }

//  //Write an 11-bit Standard ID into Filter 2
// void setFilter_2(unsigned short identifier)
// {
//     writeRegister(RXF2SIDH, (byte)(identifier>>3));
//     writeRegister(RXF2SIDL, (byte)(identifier<<5));
// }

//  //Write an 11-bit Standard ID into Filter 3
// void setFilter_3(unsigned short identifier)
// {
//     writeRegister(RXF3SIDH, (byte)(identifier>>3));
//     writeRegister(RXF3SIDL, (byte)(identifier<<5));
// }

//  //Write an 11-bit Standard ID into Filter 4
// void setFilter_4(unsigned short identifier)
// {
//     writeRegister(RXF4SIDH, (byte)(identifier>>3));
//     writeRegister(RXF4SIDL, (byte)(identifier<<5));
// }

//  //Write an 11-bit Standard ID into Filter 5
// void setFilter_5(unsigned short identifier)
// {
//     writeRegister(RXF5SIDH, (byte)(identifier>>3));
//     writeRegister(RXF5SIDL, (byte)(identifier<<5));
// }
  
// //===== end of file MCP2515JT.cpp =====          