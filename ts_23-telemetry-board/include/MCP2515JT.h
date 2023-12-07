
#include "mbed.h"
/*
Originally created by: Kyle Crockett
Modified and comments added by Jon Titus
tituskz1g (at) gmail (dot) com
21 June 2016
*/

#ifndef MCP2515JT_h       //Avoid loading MCP2515.h more than once
#define MCP2515JT_h

//SPI RESET command, resets MCP2515 
#define RESET_REG 0xC0          

//Basic SPI read and write commands
#define READ  0x03 
#define WRITE 0x02
 
//Read RX buffer ID and Data bytes
//SPI Commands point to the first byte in the buffer.
//For example, READ_RX_BUF_0_ID points to RXB0SIDH byte
#define READ_RX_BUF_0_ID 0x90       //Read Receiver Buffer-0 Standard ID
#define READ_RX_BUF_0_DATA 0x92     //Read Receiver Buffer-0 Data byte
#define READ_RX_BUF_1_ID 0x94       //Read Receiver Buffer-1 Standard ID
#define READ_RX_BUF_1_DATA 0x96     //Read Receiver Buffer-1 Data byte 

//Load Transmitter Buffers with Standard Address and Data Bytes
#define LOAD_TX_BUF_0_ID 0x40       //Points to TXB0SIDH byte
#define LOAD_TX_BUF_0_DATA 0x41     //Points to Data byte TXB0D0
#define LOAD_TX_BUF_1_ID 0x42       //Points to TXB1SIDH byte
#define LOAD_TX_BUF_1_DATA 0x43     //Points to Data byte TXB1D0
#define LOAD_TX_BUF_2_ID 0x44       //Points to TXB2SIDH byte
#define LOAD_TX_BUF_2_DATA 0x45     //Points to Data byte TXB2D0

//Send Transmitter-Buffer Frame 
#define SEND_TX_BUF_0 0x81          //Transmit TXB0 frame
#define SEND_TX_BUF_1 0x82          //Transmit TXB1 frame
#define SEND_TX_BUF_2 0x84          //Transmit TXB2 frame
 
#define READ_STATUS 0xA0            //Read Status register
#define RX_STATUS 0xB0              //Read Receiver Status register
#define BIT_MODIFY 0x05             //Bit-Modify SPI Command

//Registers
#define CANCTRL 0x0F                //CAN CTRL Register
#define CNF0 0x2A                   //Timing Register CNF1
#define CNF1 0x29                   //Timing Register CNF2
#define CNF2 0x28                   //Timing Register CNF3

#define TXB0CTRL 0x30               //Transmitter Buffer-Control registers 
#define TXB1CTRL 0x40
#define TXB2CTRL 0x50 

#define TXB0DLC 0x35                //Transmitter Data-Length registers
#define TXB1DLC 0x45
#define TXB2DLC 0x55

#define MCP2515CTRL 0x0F            //Mode control register
#define MCP2515STAT 0x0E            //Mode status register
 
//Mode-configuration bit settings
#define MODE_CONFIG     0x80        //Configuration mode
#define MODE_LISTEN     0x60        //Listen-only mode
#define MODE_LOOPBACK   0x40        //Loopback mode for testing
#define MODE_SLEEP      0x20        //Sleep mode for low power
#define MODE_NORMAL     0x00        //Normal operating mode
 
//Bus-Error Counter Registers
#define TEC            0x1C         //Transmit and Receive Error Countera
#define REC            0x1D
 
//Receiver-mask register addresses
//Mask 0
#define RXM0SIDH    0x20            //Mask-0 Standard ID high bits SID10--SID3
#define RXM0SIDL    0x21            //Mask-0 Standard ID low bits, SID2--DID0 (as MSBs)
#define RXM0EID8    0x22            //Mask-0 Extended ID high bits EID15--EID8
#define RXM0EID0    0x23            //Mask-0 Extended ID low bits, EID7--EID0
                                    //EDI17 and EID16 bits are 2 LSBs in RXM0SIDL register
 
//Mask 1
#define RXM1SIDH    0x24            //Mask-1 Standard ID high bits SID10--SID3
#define RXM1SIDL    0x25            //Mask-1 Standard ID low bits, SID2--DID0 (as MSBs)
#define RXM1EID8    0x26            //Mask-1 extended ID high bits EID15--EID8
#define RXM1EID0    0x27            //Mask-1 extended ID low bits, EID7--EID0
                                    //EDI17 and EID16 bits are 2 LSBs in RXM0SIDL register

//Receiver-filter register addresses (NOT including extended-address registers)                                
//Filter 0  Standard 11-bit address
#define RXF0SIDH    0x00            //Filter-0 Standard ID high bits SID10--SID3
#define RXF0SIDL    0x01            //Filter-0 Standard ID low bits SID2--SID0

//Filter 1  Standard 11-bit address
#define RXF1SIDH    0x04            //Filter-1 Standard ID high bits SID10--SID3
#define RXF1SIDL    0x05            //Filter-1 Standard ID low bits SID2--SID0

//Filter 2  Standard 11-bit address
#define RXF2SIDH    0x08            //Filter-2 Standard ID high bits SID10--SID3
#define RXF2SIDL    0x09            //Filter-2 Standard ID low bits SID2--SID0

//Filter 3  Standard 11-bit address
#define RXF3SIDH    0x10            //Filter-3 Standard ID high bits SID10--SID3
#define RXF3SIDL    0x11            //Filter-3 Standard ID low bits SID2--SID0

//Filter 4  Standard 11-bit address
#define RXF4SIDH    0x14            //Filter-4 Standard ID high bits SID10--SID3
#define RXF4SIDL    0x15            //Filter-4 Standard ID low bits SID2--SID0

//Filter 5  Standard 11-bit address
#define RXF5SIDH    0x18            //Filter-4 Standard ID high bits SID10--SID3
#define RXF5SIDL    0x19            //Filter-4 Standard ID low bits SID2--SID0

//Interrupt and error-flag register addresses
#define MCP2515INTE  0x2B           //Interrupt-Enable register
#define MCP2515INTF  0x2C           //Interrrupt-Flag register
#define EFLG         0x2D           //Error-flag register
 
#define MASK_SID_ALL_HIT   0x0000   //Mask all
#define MASK_SID_CPL_MATCH 0x07FF   //Disable mask
 
 
#define MCP2515_RTS         0x80
#define MCP2515_READ_BUFFER 0x90
#define MCP2515_LOAD_BUFFER 0X40
 

//Interrupt-Flag Bit Masks
//Use these masks with a bitwise AND to identify or test for specific interrupts
#define RX0IF  0x01             //Receiver Buffer-0-Full Interrupt Flag bit
#define RX1IF  0x02             //Receiver Buffer-1-Full Interrupt Flag bit     
#define TX0IF  0x04             //Transmit Buffer-0-Empty Interrupt Flag bit
#define TX1IF  0x08             //Transmit Buffer-1-Empty Interrupt Flag bit
#define TX2IF  0x10             //Transmit Buffer-2-Empty Interrupt Flag bit
#define ERRIF  0x20             //Error-Interrupt Flag bit
#define WAKIF  0x40             //Wake-Up-Interrupt Flag bit
#define MERRF  0x80             //Message-Error-Interrupt Flag bit
 
//Configuration Registers used for I/O control of unused pins.
#define BFPCTRL         0x0C    //Receiver pin and pin-status bits used for I/O control
#define TXRTSCTRL       0x0D    //Transmitter pin and pin-status bits used for I/O control
 
//Transmitter Registers
//Transmitter Buffer-0 Register identifier
#define TXB0CTRL        0x30    //Transmitter Control register
#define TXB0SIDH        0x31    //Standard address, bits SID10--SID3
#define TXB0SIDL        0x32    //Standard address, bits SID2--SID0 as MSBs
#define TXB0EID8        0x33    //Extended ID high bits EID15--EID8
#define TXB0EID0        0x34    //Extended ID low bits, EID7--EID0
                                //EDI17 and EID16 bits are 2 LSBs in RXM0SIDL register
#define TXB0DLC         0x35    //Data-Length-value register, DLC3--DLC0
 
///Transmitter Buffer-1 Register identifier
#define TXB1CTRL        0x40
#define TXB1SIDH        0x41
#define TXB1SIDL        0x42
#define TXB1EID8        0x43
#define TXB1EID0        0x44
#define TXB1DLC         0x45
 
//Transmitter Buffer-2 Register identifier
#define TXB2CTRL        0x50
#define TXB2SIDH        0x51
#define TXB2SIDL        0x52
#define TXB2EID8        0x53
#define TXB2EID0        0x54
#define TXB2DLC         0x55
 
//Receiver Registers
//Receiver Buffer-0 Register Identification
#define RXB0CTRL        0x60        //Receiver Control Register
#define RXB0SIDH        0x61        //Standard address, bits SID10--SID3
#define RXB0SIDL        0x62        //Standard address, bits SID2--SID0 as MSBs
#define RXB0EID8        0x63        //Extended ID high bits EID15--EID8
#define RXB0EID0        0x64        //Extended ID low bits, EID7--EID0
                                    //EDI17 and EID16 bits are 2 LSBs in RXM0SIDL register
#define RXB0DLC         0x65        //Data-Length-value register, DLC3--DLC0
 
//Receiver Buffer-1 Register Identification
#define RXB1CTRL        0x70
#define RXB1SIDH        0x71
#define RXB1SIDL        0x72
#define RXB1EID8        0x73
#define RXB1EID0        0x74
#define RXB1DLC         0x75
 
//Buffer Bit Masks
#define RXB0            0x00
#define RXB1            0x02
#define TXB0            0x01
#define TXB1            0x02
#define TXB2            0x04
#define TXB_ALL         TXB0 | TXB1 | TXB2
 
#define RXB_RX_STDEXT   0x00
#define RXB_RX_MASK     0x60
#define RXB_BUKT_MASK   (1<<2)
 
typedef unsigned char byte;
 
enum MCP2515Mode {CONFIGURATION,NORMAL,SLEEP,LISTEN,LOOPBACK};

//Set CAN bit rate in kilobits/second; that is 125,000 bits/sec is 125
    void baudConfig(int bitRate);//sets up baud
 
//Method added to enable testing in loopback mode.(pcruce_at_igpp.ucla.edu)
    void setMode(MCP2515Mode mode) ;//put MCP2515 controller in one of five modes

//Transmit requests 
    void send_0();  //Transmit buffer 0
    void send_1();  //Transmit buffer 1
    void send_2();  //Transmit buffer 2
 
//Read from Receiver buffers
    char readID_0();            //read ID from receiver buffer 0
    char readID_1();            //Read ID from receiver buffer 1
 
    char readDATA_0();          //Read data from receiver buffer 0
    char readDATA_1();          //Read data from receiver buffer 1
 
//Extending MCP2515 data-read of full frames added by (pcruce_at_igpp.ucla.edu)
//Your data_out array should hold 8-bytes.
    void readDATA_ff_0(byte* length_out,byte *data_out,unsigned short *id_out);
    void readDATA_ff_1(byte* length_out,byte *data_out,unsigned short *id_out);
 
//MCP2515 read-status register info, added by (pcruce_at_igpp.ucla.edu)
//MCP2515 be used to determine whether a frame was received.
//(readStatus() & 0x80) == 0x80 means frame in buffer 0
//(readStatus() & 0x40) == 0x40 means frame in buffer 1
    byte readStatus();
 
 //Load Transmitter Buffers
    void load_0(byte identifier, byte data);    //Load a byte into transmitter buffer 0
    void load_1(byte identifier, byte data);    //Load a byte into transmitter buffer 1
    void load_2(byte identifier, byte data);    //Load a byte into transmitter buffer 2
 
//Write a full frame (ff), added by (pcruce_at_igpp.ucla.edu)
//Identifier should be a value between 0 and 2047, longer identifiers get truncated
//(no extended frames)
    void load_ff_0(byte length,unsigned short identifier,byte *data);
    void load_ff_1(byte length,unsigned short identifier,byte *data);
    void load_ff_2(byte length,unsigned short identifier,byte *data);
    
//Read and Write Register bytes    
    void writeRegister(byte address, byte data);
    void readRegister(byte address, byte *data_out);
    
//Read ID bytes from a receiver register and return a hex address value, added by Jon Titus
    unsigned short readSDI0_asHex();
    unsigned short readSDI1_asHex();

//Reset MCP2515
    void reset();
    
//Read Receiver Status
    byte readRXStatus();
    
//Bit-Modify functions
    void bitModify(byte address, byte mask, byte data);
    
//Mask-load routines
    void setMask(unsigned short identifier);
    void setMask_0(unsigned short identifier);
    void setMask_1(unsigned short identifier);
    
//Filter-load routines
    void setFilter_0(unsigned short identifier);
    void setFilter_1(unsigned short identifier);
    void setFilter_2(unsigned short identifier);
    void setFilter_3(unsigned short identifier);
    void setFilter_4(unsigned short identifier);
    void setFilter_5(unsigned short identifier); 
#endif
//===== end of file MCP2515JT.h =====
            
