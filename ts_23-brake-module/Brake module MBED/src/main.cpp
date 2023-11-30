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
#include "ads7028.h"
#include "PDM_info.h"

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
/*                 Other functions added in to help this works                */
/* -------------------------------------------------------------------------- */
void setCS(uint8_t state)
{
    CS_PIN->write(state);
};

void spiSendReceiveArray(uint8_t *dataTx, uint8_t *dataRx, uint8_t numberOfBytes)
{
    // Require that dataTx and dataRx are not NULL pointers
    assert(dataTx && dataRx);   
    // Set the nCS pin LOW

    setCS(0);
    ////wait_ms(1);

    uint8_t i;
    for (i = 0; i < numberOfBytes; i++)
    {
        dataRx[i] = spi.write(dataTx[i]);
    }

    ////wait_ms(1);
    setCS(1);
};

//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/** Array used to recall device register map configurations */
static uint8_t      registerMap[MAX_REGISTER_ADDRESS + 1];


//****************************************************************************
//
// Internal Function prototypes
//
//****************************************************************************
static void         restoreRegisterDefaults(void);
static int16_t      signExtend(const uint8_t dataBytes[]);

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

//*****************************************************************************
//
//! \brief Example start up sequence for the ADS7028.
//!
//! \fn void initADS7038(void)
//!
//! Before calling this function, the device must be powered,
//! the SPI pins of the MCU must have already been configured.
//!
//! \return None.
//
//*****************************************************************************
void initADS7028(void)
{
    // (OPTIONAL) Provide additional delay time for power supply settling
    ////wait_ms(50);

    // Reset device
    resetDevice();

    // Clear BOR flag
    setRegisterBits(SYSTEM_STATUS_ADDRESS, SYSTEM_STATUS_BOR_MASK);

    // (OPTIONAL) Configure initial register settings here

    // (RECOMMENDED) If you plan to modify the CRC_EN or CPOL_CPHA bits,
    // do so here (and only here) to be simplify the code implementation.

    // (OPTIONAL) Read back registers and check STATUS register for faults

}


//*****************************************************************************
//
//! \brief  Resets the device and reinitializes the register map array
//!         maintained in firmware to default values.
//!
//! \fn     void resetDevice()
//!
//! \return None
//
//*****************************************************************************
void resetDevice()
{
    // Set the RST bit high to reset the device
    setRegisterBits(GENERAL_CFG_ADDRESS, GENERAL_CFG_RST_MASK);

    // Update internal register map array
    restoreRegisterDefaults();
}


//*****************************************************************************
//
//! Starts periodic ADC conversions
//!
//! \fn void startConversions(uint32_t samplesPerSecond, uint8_t OSR)
//! \param CHID channel number (0-7) selected as analog input
//! \param samplesPerSecond desired sampling rate (specified as an integer number of SPS)
//! \return None.
//
//*****************************************************************************
void startManualConversions(uint8_t channelID, uint32_t samplesPerSecond)
{
    // Select manual mode
    writeSingleRegister(SEQUENCE_CFG_ADDRESS, SEQUENCE_CFG_SEQ_MODE_MANUAL);

    // Configure pin as analog input
    setChannelAsAnalogInput(channelID);

    // Select channel as MUX input
    writeSingleRegister(CHANNEL_SEL_ADDRESS, channelID);

    // Set nCS pin LOW, next rising edge will trigger start of conversion
    setCS(0);

    // Start conversion timer
    //startTimer(samplesPerSecond);
}

void SetupADS7028(DigitalOut *cs, ADC_CONFIG adc_type)
{
    CS_PIN = cs;//Assign CS Pin to desired corresponding chip
    initADS7028();

    if (adc_type == GPIO_OUT){ //If ADC is being used for digital outputs
        writeSingleRegister(PIN_CFG_ADDRESS, 0b11111111); 
        //wait_ms(50);
        writeSingleRegister(GPIO_CFG_ADDRESS, 0b11111111);
        //wait_ms(50);
        writeSingleRegister(GPO_DRIVE_CFG_ADDRESS, 0b11111111);
        //wait_ms(50);
        writeSingleRegister(GPO_OUTPUT_VALUE_ADDRESS, 0b00000000);
    }
    else{ //If ADC is being used for analog inputs
        for (int i = 0; i<8; i++){
            setChannelAsAnalogInput(i);
        } //Configure all pins as analog in
        //wait_ms(50);
        writeSingleRegister(OPMODE_CFG_ADDRESS, OPMODE_CFG_CONV_MODE_MANUAL_MODE); //Select Manual Mode
        //wait_ms(50);
        writeSingleRegister(SEQUENCE_CFG_ADDRESS, SEQUENCE_CFG_SEQ_MODE_MANUAL); //Manual Channel Selection Mode
        //wait_ms(50);
    }   
    //wait_ms(50);
}


//*****************************************************************************
//
//! Stops ADC conversions
//!
//! \fn void stopConversions(void)
//!
//! \return None.
//
//*****************************************************************************
void stopConversions(void)
{
    // Stop conversion timer
    //stopTimer();

    // Set nCS pin HIGH, allows MCU to communicate with other devices on SPI bus
    setCS(1);
}


//*****************************************************************************
//
//! \brief  Reads ADC conversion result and returns 16-bit sign-extended value.
//!
//! \fn void readData(uint8_t dataRx[])
//!
//! \param *dataRx points to receive data byte array
//!
//! \return int16_t (sign-extended data).
//
//*****************************************************************************
int16_t readData()
{
    uint8_t dataRx[4] = {0}; 
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // NULL command
    uint8_t dataTx[4] = { 0 };
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
        //device.printf("DataTX[3] =")
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    //uint16_t Data = 0;
    //Data = (dataRx[0] << 8) + dataRx[1];

    //device.printf("DataRx[0] = %\n",dataRx[0]);
    //device.printf("DataRx[1] = %\n",dataRx[1]);
    //device.printf("DataRx[2] = %\n",dataRx[2]);
    //device.printf("DataRx[3] = %\n",dataRx[3]);
    return signExtend(dataRx);

}


//*****************************************************************************
//
//! \brief  Reads the contents of a single register at the specified address.
//!
//! \fn     uint8_t readSingleRegister(uint8_t address)
//!
//! \param  address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//
//*****************************************************************************
uint8_t readSingleRegister(uint8_t address)
{
    // Check that the register address is in range
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t dataTx[4] = {0};
    uint8_t dataRx[4] = {0};
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;
    bool crcError = false;

    //
    // [FRAME 1] RREG command
    //
    dataTx[0] = OPCODE_RREG;
    dataTx[1] = address;
    dataTx[2] = OPCODE_NULL;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    //
    // [FRAME 2] NULL command
    //
    dataTx[0] = OPCODE_NULL;
    dataTx[1] = OPCODE_NULL;
    dataTx[2] = OPCODE_NULL;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // Check for CRC error
    if (SPI_CRC_ENABLED)
    {
        // To check the CRC validity you can test either of the following conditions:
        // 1) "dataRx[1] == calculateCRC(dataRx, 1, CRC_INITIAL_SEED)" - true means no CRC error occurred.
        // 2) "0x00 == calculateCRC(dataRx, 2, CRC_INITIAL_SEED) - including the CRC byte in the calculation should return 0x00.
        crcError = (bool) calculateCRC(dataRx, 2, CRC_INITIAL_SEED);
    }
    if (crcError)
    {
        // Update internal register array
        registerMap[SYSTEM_STATUS_ADDRESS] = registerMap[SYSTEM_STATUS_ADDRESS] || SYSTEM_STATUS_CRCERR_IN_MASK;

        // (OPTIONAL) Consider notifying the system of the error and repeating the previous command.
    }
    else
    {
        registerMap[address] = dataRx[0];
    }

    return registerMap[address];
}


//*****************************************************************************
//
//! Getter function to access registerMap array from outside of this module.
//!
//! \fn uint8_t getRegisterValue(uint8_t address)
//!
//! NOTE: The internal registerMap array stores the last known register value,
//! since the last read or write operation to that register. This function
//! does not communicate with the device to retrieve the current register value.
//! For the most up-to-date register data or retrieving the value of a hardware
//! controlled register, it is recommend to use readSingleRegister() to read the
//! current register value.
//!
//! \return unsigned 8-bit register value.
//
//*****************************************************************************
uint8_t getRegisterValue(uint8_t address)
{
    assert(address <= MAX_REGISTER_ADDRESS);
    return registerMap[address];
}


//*****************************************************************************
//
//! \brief  Writes data to a single register and reads it back for confirmation.
//!
//! \fn void writeSingleRegister(uint8_t address, uint8_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! \return None.
//
//*****************************************************************************
void writeSingleRegister(uint8_t address, uint8_t data)
{
    // Check that the register address is in range
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t dataTx[4] = { 0 };
    uint8_t dataRx[4] = { 0 };
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // (OPTIONAL) Check for and clear CRC error to proceed with register write.
    // Once a CRC error has occurred, writes are only allowed to the SYSTEM_STATUS and GENERAL_CFG registers
    if (SPI_CRC_ENABLED && (address > GENERAL_CFG_ADDRESS))
    {
        // Read STATUS register to check whether CRC error has occurred or not.
        readSingleRegister(SYSTEM_STATUS_ADDRESS);
        if (SPI_CRCERR_IN)
        {
            // (OPTIONAL) Clear the CRC error by writing 1b to CRCERR_IN bit
            setRegisterBits(SYSTEM_STATUS_ADDRESS, SYSTEM_STATUS_CRCERR_IN_MASK);

            // (OPTIONAL) Consider notifying the system of the error and repeating the previous command.
        }
    }

    // WREG command
    dataTx[0] = OPCODE_WREG;
    dataTx[1] = address;
    dataTx[2] = data;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // Update internal register map array (assume command was successful).
    // NOTE: This is required for writing to the CRC_EN bit to ensure read back uses the correct mode.
    registerMap[address] = data;

    // NOTE: If you modify the CPOL_CPHA bits in the DATA_CFG register, the SPI perhiperal will need to be reconfigured here.

    // (RECOMMENDED) Read back register to confirm register write was successful
    registerMap[address] = readSingleRegister(address);
}


//*****************************************************************************
//
//! \fn void setRegisterBits(uint8_t address, uint8_t bitMask)
//!
//! \param address is the address of the register to write to.
//! \param bitMask indicates which bit(s) in the register to set.
//!
//! This function does not perform a read back of the register value.
//!
//! \return None.
//
//*****************************************************************************
void setRegisterBits(uint8_t address, uint8_t bitMask)
{
    // Check that the register address is in range
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t dataTx[4] = {0};
    uint8_t dataRx[4] = {0};
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // SETBIT command
    dataTx[0] = OPCODE_SETBIT;
    dataTx[1] = address;
    dataTx[2] = bitMask;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // Update internal register map array (assume command was successful).
    // NOTE: This is required for writing to the CRC_EN bit to ensure read back uses the correct mode.
    registerMap[address] = registerMap[address] | bitMask;

    // (OPTIONAL) Check if a CRC error occurred
}


//*****************************************************************************
//
//!
//! \fn void clearRegisterBits(uint8_t address, uint8_t bitMask)
//!
//! \param address is the address of the register to write to.
//! \param bitMask indicates which bit(s) in the register to clear.
//!
//! This function does not perform a read back of the register value.
//!
//! \return None.
//
//*****************************************************************************
void clearRegisterBits(uint8_t address, uint8_t bitMask)
{
    // Check that the register address is in range
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t dataTx[4] = {0};
    uint8_t dataRx[4] = {0};
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // CLRBIT command
    dataTx[0] = OPCODE_CLRBIT;
    dataTx[1] = address;
    dataTx[2] = bitMask;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // Update internal register map array (assume command was successful).
    // NOTE: This is required for writing to the CRC_EN bit to ensure read back uses the correct mode.
    registerMap[address] = registerMap[address] & ~bitMask;

    // (OPTIONAL) Check if a CRC error occurred
}


//*****************************************************************************
//
//! \brief  Calculates the 8-bit CRC for the selected CRC polynomial.
//!
//! \fn uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
//!
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes number of bytes to be used in CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use CRC_INITIAL_SEED when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and is not optimized for speed.
//!
//! \return 8-bit calculated CRC word
//
//*****************************************************************************
uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
{
    // Check that "dataBytes" is not a null pointer
    assert(dataBytes != 0x00);

    int         bitIndex, byteIndex;
    bool        dataMSb;                        /* Most significant bit of data byte */
    bool        crcMSb;                         /* Most significant bit of crc byte  */

    // Initial value of crc register
    // Use 0x00 when starting a new computation OR provide result of previous CRC calculation when continuing an on-going calculation.
    uint8_t crc = initialValue;

    // CRC polynomial = x^8 + x^2 + x^1 + 1
    const uint8_t poly = 0x07;

    /* CRC algorithm */

    // Loop through all bytes in the dataBytes[] array
    for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
    {
        // Point to MSb in byte
        bitIndex = 0x80u;

        // Loop through all bits in the current byte
        while (bitIndex > 0)
        {
            // Check MSB's of data and crc
            dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
            crcMSb  = (bool) (crc & 0x80u);

            // Update crc register
            crc <<= 1;
            if (dataMSb ^ crcMSb) { crc ^= poly; }

            // Shift MSb pointer to the next data bit
            bitIndex >>= 1;
        }
    }

    return crc;
}


//*****************************************************************************
//
//! Configure the selected channel as an analog Input
//!
//! \fn void setChannelAsAnalogInput(uint8_t channelID)
//!
//! \param channelID is the channel number.
//!
//! \return None
//
//*****************************************************************************
void setChannelAsAnalogInput(uint8_t channelID)
{
    // Check that channel ID is in range.
    assert(channelID < 8);

    // Clear the corresponding channel bit to configure channel as an analog input
    clearRegisterBits(PIN_CFG_ADDRESS, (1 << channelID));
}


//****************************************************************************
//
// Helper functions
//
//****************************************************************************

//*****************************************************************************
//
//! \brief  Updates the registerMap[] array to its default values.
//!
//! \fn static void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADS7038 register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device (either by
//! hardware pin control or SPI software command).
//!
//! - Reading back all of the registers after resetting the device can
//! accomplish the same result; however, this might be problematic if the
//! device was previously in CRC mode , since resetting the device exits
//! these modes. If the MCU is not aware of this mode change, then read
//! register commands will fail.
//!
//! \return None.
//
//*****************************************************************************
static void restoreRegisterDefaults(void)
{
    registerMap[SYSTEM_STATUS_ADDRESS]          = SYSTEM_STATUS_DEFAULT;
    registerMap[GENERAL_CFG_ADDRESS]            = GENERAL_CFG_DEFAULT;

    registerMap[DATA_CFG_ADDRESS]               = DATA_CFG_DEFAULT;
    registerMap[OSR_CFG_ADDRESS]                = OSR_CFG_DEFAULT;
    registerMap[OPMODE_CFG_ADDRESS]             = OPMODE_CFG_DEFAULT;
    registerMap[PIN_CFG_ADDRESS]                = PIN_CFG_DEFAULT;

    registerMap[GPIO_CFG_ADDRESS]               = GPIO_CFG_DEFAULT;
    registerMap[GPO_DRIVE_CFG_ADDRESS]          = GPO_DRIVE_CFG_DEFAULT;
    registerMap[GPO_OUTPUT_VALUE_ADDRESS]       = GPO_OUTPUT_VALUE_DEFAULT;
    registerMap[GPI_VALUE_ADDRESS]              = GPI_VALUE_DEFAULT;

    registerMap[SEQUENCE_CFG_ADDRESS]           = SEQUENCE_CFG_DEFAULT;
    registerMap[CHANNEL_SEL_ADDRESS]            = CHANNEL_SEL_DEFAULT;
    registerMap[AUTO_SEQ_CHSEL_ADDRESS]         = AUTO_SEQ_CHSEL_DEFAULT;

    registerMap[ALERT_CH_SEL_ADDRESS]           = ALERT_CH_SEL_DEFAULT;
    registerMap[ALERT_MAP_ADDRESS]              = ALERT_MAP_DEFAULT;
    registerMap[ALERT_PIN_CFG_ADDRESS]          = ALERT_PIN_CFG_DEFAULT;

    registerMap[EVENT_FLAG_ADDRESS]             = EVENT_FLAG_DEFAULT;
    registerMap[EVENT_HIGH_FLAG_ADDRESS]        = EVENT_HIGH_FLAG_DEFAULT;
    registerMap[EVENT_LOW_FLAG_ADDRESS]         = EVENT_LOW_FLAG_DEFAULT;
    registerMap[EVENT_RGN_ADDRESS]              = EVENT_RGN_DEFAULT;

    registerMap[HYSTERESIS_CH0_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH1_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH2_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH3_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH4_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH5_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH6_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH7_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;

    registerMap[EVENT_COUNT_CH0_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH1_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH2_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH3_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH4_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH5_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH6_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH7_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;

    registerMap[HIGH_TH_CH0_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH1_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH2_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH3_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH4_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH5_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH6_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH7_ADDRESS]            = HIGH_TH_CHx_DEFAULT;

    registerMap[LOW_TH_CH0_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH1_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH2_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH3_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH4_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH5_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH6_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH7_ADDRESS]             = LOW_TH_CHx_DEFAULT;

    registerMap[MAX_CH0_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH1_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH2_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH3_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH4_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH5_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH6_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH7_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;

    registerMap[MAX_CH0_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH1_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH2_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH3_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH4_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH5_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH6_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH7_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;

    registerMap[MIN_CH0_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH1_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH2_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH3_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH4_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH5_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH6_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH7_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;

    registerMap[MIN_CH0_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH1_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH2_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH3_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH4_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH5_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH6_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH7_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;

    registerMap[RECENT_CH0_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH1_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH2_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH3_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH4_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH5_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH6_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH7_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;

    registerMap[RECENT_CH0_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH1_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH2_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH3_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH4_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH5_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH6_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH7_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;

    registerMap[GPO0_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO1_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO2_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO3_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO4_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO5_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO6_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO7_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;

    registerMap[GPO_TRIGGER_CFG_ADDRESS]        = GPO_TRIGGER_CFG_DEFAULT;
    registerMap[GPO_VALUE_TRIG_ADDRESS]         = GPO_VALUE_TRIG_DEFAULT;
}


//*****************************************************************************
//
//! Called by readData() to convert ADC data from multiple unsigned
//! bytes into a single, signed 16-bit word.
//!
//! \fn int16_t signExtend(const uint8_t dataBytes[])
//!
//! \param dataBytes pointer to data array (big-endian).
//!
//! \return Returns the signed-extend 16-bit result.
//
//*****************************************************************************
static int16_t signExtend(const uint8_t dataBytes[])
{
    int16_t upperByte = (int32_t)(dataBytes[0] << 8);
    int16_t lowerByte = (int32_t)(dataBytes[1] << 0);

    // NOTE: This right-shift operation on signed data maintains the sign bit
    uint8_t shiftDistance = AVERAGING_ENABLED ? 0 : 4;
    return (((int16_t)(upperByte | lowerByte)) >> shiftDistance);
}

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
  startManualConversions(4, 100);
  int16_t testData = readData();
  stopConversions();
  float Brake1_Voltage = abs((testData*1.0/4096.0)*5.0);

  startManualConversions(2, 100);
  testData = readData();
  stopConversions();
  float Brake2_Voltage = abs((testData*1.0/4096.0)*5.0);


  BrakeModule.brake1_raw      = Brake1_Voltage;
  BrakeModule.brake2_raw      = Brake2_Voltage;
  BrakeModule.High_Pressure   = !(HighPressure.read()); //Must be inverted
  BrakeModule.Low_Pressure    = LowPressure.read();
  BrakeModule.five_kW         = CurrentSensor.read();
  BrakeModule.BSPD_OK         = BSPD.read();
  BrakeModule.BSPD_OK_delay   = BSPD_Delay.read();

  BrakeModule.brake1_percent    = raw_to_percent(BrakeModule.brake1_raw, brake_calibration.brake1_max, brake_calibration.brake1_min);
  BrakeModule.brake2_percent    = raw_to_percent(BrakeModule.brake2_raw, brake_calibration.brake2_max, brake_calibration.brake2_min);
  BrakeModule.brake_avg_percent = (BrakeModule.brake1_percent + BrakeModule.brake2_percent)/2.0;

  BrakeModule.brake_low_ref   = 3.3*lowRef.read();
  BrakeModule.brake_high_ref  = 3.3*highRef.read();
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

  TX_data[CAN_DIGITAL_1_BRAKE_HIGH_PRESSURE] = BrakeModule.High_Pressure;
  TX_data[CAN_DIGITAL_1_BRAKE_LOW_PRESSURE] = BrakeModule.Low_Pressure;
  TX_data[CAN_DIGITAL_1_BRAKE_5KW] = BrakeModule.five_kW;
  TX_data[CAN_DIGITAL_1_BRAKE_BSPD_OK] = BrakeModule.BSPD_OK;
  TX_data[CAN_DIGITAL_1_BRAKE_BSPD_OK_DELAY] = BrakeModule.BSPD_OK_delay;

  can1.write(CANMessage((CAN_BRAKE_MODULE_BASE_ADDRESS + TS_DIGITAL_1_ID), TX_data, 5));
}

void CAN_brakeModule_TX_Analog_1()
{
  char TX_data[3] = { 0 };

  TX_data[CAN_ANALOG_1_BRAKE1_PERCENT] = BrakeModule.brake1_percent;
  TX_data[CAN_ANALOG_1_BRAKE2_PERCENT] = BrakeModule.brake2_percent;
  TX_data[CAN_ANALOG_1_BRAKE_AVG_PERCENT]     = BrakeModule.brake_avg_percent;
  //TX_data[CAN_ANALOG_1_BRAKE1_RAW]            = uint8_t(BrakeModule.brake1_raw*10);
  //TX_data[CAN_ANALOG_1_BRAKE2_RAW]            = uint8_t(BrakeModule.brake2_raw*10);
  //TX_data[CAN_ANALOG_1_BRAKE_LOW_REF]         = uint8_t(BrakeModule.brake_low_ref*10);
  //TX_data[CAN_ANALOG_1_BRAKE_HIGH_REF]        = uint8_t(BrakeModule.brake_high_ref*10);
  //TX_data[CAN_TRAILBRAKE_PERCENT]             = BrakeModule.trailbrake_percent;
  
  can1.write(CANMessage((CAN_BRAKE_MODULE_BASE_ADDRESS + TS_ANALOGUE_1_ID), TX_data, 3));
}

void CAN_brakeModule_TX_Analog_2()
{
  char TX_data[8] = { 0 };

  startManualConversions(4, 100);
  int16_t testData1 = readData();
  stopConversions();
  float Brake1_Voltage = abs((testData1*1.0/4096.0)*5.0);

  startManualConversions(2, 100);
  int16_t testData2 = readData();
  stopConversions();
  float Brake2_Voltage = abs((testData2*1.0/4096.0)*5.0);

  //Convert to integers with 1mV resolution
  int brake1_Voltage = uint16_t(Brake1_Voltage*1000);
  int brake2_Voltage = uint16_t(Brake2_Voltage*1000);
  int highRef_Voltage = uint16_t(BrakeModule.brake_high_ref*1000);
  int lowRef_Voltage = uint16_t(BrakeModule.brake_low_ref*1000);

  TX_data[0] = (brake1_Voltage >> 8) & 0xFF;
  TX_data[1] = (brake1_Voltage) & 0xFF;

  TX_data[2] = (brake2_Voltage >> 8) & 0xFF;
  TX_data[3] = (brake2_Voltage) & 0xFF;

  TX_data[4] = (highRef_Voltage >> 8) & 0xFF;
  TX_data[5] = (highRef_Voltage) & 0xFF;

  TX_data[6] = (lowRef_Voltage >> 8) & 0xFF;
  TX_data[7] = (lowRef_Voltage) & 0xFF;
  
  can1.write(CANMessage((CAN_BRAKE_MODULE_BASE_ADDRESS + TS_ANALOGUE_2_ID), TX_data, 8));
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
  // Disable interrupts for smooth startup routine.
	////wait_ms(1000);

  ADC_CS = 1;

  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8,0);
  spi.frequency(100000);
	
	__disable_irq();

  can1.frequency(CANBUS_FREQUENCY);
  can1.filter(CAN_BRAKE_MODULE_BASE_ADDRESS+TS_SETPOINT_1_ID, 0xFFF, CANStandard, 0); // set filter #0 to accept only standard messages with ID == RX_ID
	can1.attach(&CAN_brakeModule_RX);

  //Setup ADCS7028 chips
  SetupADS7028(&ADC_CS, ADC);

  //Configure tickers
  ticker_CAN_HeartBeat.attach(&CAN_brakeModule_TX_Heartbeat, CAN_HEARTBEAT_PERIOD);
  ticker_CAN_Digital_1.attach(&CAN_brakeModule_TX_Digital_1, CAN_DIGITAL_1_PERIOD);
  ticker_CAN_Analog_1.attach(&CAN_brakeModule_TX_Analog_1, CAN_ANALOG_1_PERIOD);
  ticker_CAN_Analog_2.attach(&CAN_brakeModule_TX_Analog_2, CAN_ANALOG_1_PERIOD);

  // Re-enable interrupts again, now that interrupts are ready.
	__enable_irq();

	// Allow some time to settle!
	////wait_ms(1000);

  while(1) 
  {
    BrakeModuleUpdate();
    //Serial_Print(); //Used for debugging.
  }

  return 0;
}

