/*!
 *  @file RadioCom.h
 *
 *  Serial/UART handler, responible for packing and transmitting serial data
 *
 */

#include "mbed.h"

#define BUFFER_SIZE 64 //buffer size of the i/o package

// default packet fields
struct PacketFields {
    uint8_t     startFlags[2];                      // start flags
    uint8_t     packetLength;                    // packet length
    uint8_t     canId[2];                              // protocol (reserved)
    uint8_t     payload[BUFFER_SIZE - 5];           // payload (variable length) and crc
};

// creating this union makes it easier to copy tx/rx data into the buffer
union RadioComBuffer {
  	PacketFields    fields;
  	uint8_t         data[BUFFER_SIZE];
};

/**
 * @brief Class that handle UART i/o packages
 * 
 */
class RadioCom {
private:
    Serial *m_pSerial; // tx, rx
    /**
    * @brief Calculate CRC16-XMODEM checksum with polynomial of 0x1021

    * @param arg_pData  Array to calculate checksum
    * @param arg_size	Length of the array
    * @return          	CRC16 Checksum
     */
    uint16_t crc16(uint8_t const *arg_pData, uint8_t arg_size);

public:

    RadioCom(PinName tx, PinName rx);

	/**
 	* @brief Begin Serial Communication to RF Module
 	*/
    void begin(int baud);

	/**
	 * @brief Packing and sending the payload
	 * 
	 * @param arg_pPayload 		The actual payload
	 * @param arg_payloadLen 	The payload's length 
	 */
    void transmit(uint8_t *arg_pPayload, uint8_t arg_payloadLen, uint16_t arg_canId);
};