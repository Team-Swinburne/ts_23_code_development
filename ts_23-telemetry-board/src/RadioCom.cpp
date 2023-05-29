#include "RadioCom.h"

//------------------------------------------------------------------------------------------------
RadioCom::RadioCom(PinName tx, PinName rx) 
{
    m_pSerial = new Serial(PB_10, PB_11);
}

//------------------------------------------------------------------------------------------------
void RadioCom::begin(int baud)
{
    m_pSerial->baud(baud);
    m_pSerial->printf("RadioCom initialized\n");
}

//------------------------------------------------------------------------------------------------
void RadioCom::transmit(uint8_t *arg_pPayload, uint8_t arg_payloadLen, uint16_t arg_canId)
{
    RadioComBuffer txBuffer;
    uint8_t packetLen = arg_payloadLen + 4; // 4 bytes for canId and checksum

    memset(txBuffer.data, 0, sizeof(txBuffer.data));            // clearing the txBuffer

    txBuffer.fields.startFlags[0]   = 0x33;                     // start flag MSB & LSB
    txBuffer.fields.startFlags[1]   = 0x17;
    txBuffer.fields.packetLength    = packetLen;                // packet length MSB & LSB
    txBuffer.fields.canId[0] = (arg_canId >> 8) & 0xFF;         // CAN ID MSB & LSB
    txBuffer.fields.canId[1] = arg_canId & 0xFF;

    // copying the payload to txBuffer
    for (int i = 0; i < arg_payloadLen; i++) {
        txBuffer.fields.payload[i] = arg_pPayload[i];
    }

    // calculate checksum
    uint16_t checksum = crc16(&txBuffer.data[3],packetLen-2); // from canId to end of payload
    txBuffer.fields.payload[arg_payloadLen] = (checksum >> 8) & 0xFF;
    txBuffer.fields.payload[arg_payloadLen+1] = checksum & 0xFF;

    // transmitting data
    //Serial.write("Data transmitting: ");
    for (int i=0; i<packetLen+3; i++) {
        //send to RadioCom Device
        m_pSerial->putc(txBuffer.data[i]);
        //send to PC 
       // Serial.print(txBuffer.data[i],HEX);
        //Serial.print(" ");
    }
    //Serial.println(" ");
}

//------------------------------------------------------------------------------------------------
uint16_t RadioCom::crc16(uint8_t const *arg_pData, uint8_t arg_size)
{
    int i, crc = 0;
    for (; arg_size>0; arg_size--)         /* Step through bytes in memory */
    {
        crc = crc ^ (*arg_pData++ << 8);   /* Fetch byte from memory, XOR into CRC top byte*/
        for (i=0; i<8; i++)                /* Prepare to rotate 8 bits */
        {
            crc = crc << 1;                /* rotate */
            if (crc & 0x10000)             /* bit 15 was set (now bit 16)... */
            crc = (crc ^ 0x1021) & 0xFFFF; /* XOR with XMODEM polynomic */
                                           /* and ensure CRC remains 16-bit value */
        }                                  /* Loop for 8 bits */
    }                                      /* Loop until num=0 */
    return(crc);                           /* Return updated CRC */
}