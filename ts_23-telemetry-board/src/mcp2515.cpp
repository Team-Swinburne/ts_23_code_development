#include <mcp2515.h>

MCP2515::MCP2515(SPI& spi, PinName cs) : m_spi(spi), m_cs(cs) {}

bool MCP2515::begin(int busFrequency) 
{
    bool res = true;
    uint8_t cnf1, cnf2, cnf3;

    reset();
    wait_ms(10);

    setMode(MCP2515Mode::CONFIGURATION);
    // set up timing registers
    switch (busFrequency)
    {
        case 1000000:
            cnf1 = MCP_8MHz_1000kBPS_CFG1;
            cnf2 = MCP_8MHz_1000kBPS_CFG2;
            cnf3 = MCP_8MHz_1000kBPS_CFG3;
            break;
        case 500000:
            cnf1 = MCP_8MHz_500kBPS_CFG1;
            cnf2 = MCP_8MHz_500kBPS_CFG2;
            cnf3 = MCP_8MHz_500kBPS_CFG3;
            break;
        case 250000:
            cnf1 = MCP_8MHz_250kBPS_CFG1;
            cnf2 = MCP_8MHz_250kBPS_CFG2;
            cnf3 = MCP_8MHz_250kBPS_CFG3;
            break;
    }
    res != writeRegisterConfirm(MCP_CNF1_REGISTER,(cnf1 & 0x7F));
    res != writeRegisterConfirm(MCP_CNF2_REGISTER,(cnf2 & 0x7F));
    res != writeRegisterConfirm(MCP_CNF3_REGISTER,(cnf3 & 0x7F));

    // Set up filter for RX buffer 0
    writeRegister(MCP_RXF0SIDL_REGISTER, 0x00);
    writeRegister(MCP_RXF1SIDL_REGISTER, 0x00);
    writeRegister(MCP_RXM0SIDH_REGISTER, 0x00);
    writeRegister(MCP_RXM0SIDL_REGISTER, 0x00);

    // Interupt output for RX buffer 0
    bitModify(MCP_CANINTE_REGISTER, MCP_CANINTE_RXB0_MASK, 0xFF);

    // Set up CLKOUT pin
    bitModify(MCP_CANCTRL_REGISTER, MCP_CANCTRL_CLKPRE_MASK, 0x00);

    setMode(MCP2515Mode::NORMAL);
    return res;
}

void MCP2515::writeRegister(uint8_t reg, uint8_t data) 
{
    m_cs = 0;
    m_spi.write(MCP_CMD_WRITE);
    m_spi.write(reg);
    m_spi.write(data);
    m_cs = 1;
}

uint8_t MCP2515::readRegister(uint8_t reg) 
{
    uint8_t res;

    m_cs = 0;
    m_spi.write(MCP_CMD_READ);
    m_spi.write(reg);
    res = m_spi.write(0x00); // to trigger a response
    m_cs = 1;

    return res;
}

void MCP2515::receive(uint8_t* length_out,uint8_t *data_out,unsigned short *id_out) { 
    uint8_t len,i;
    unsigned short id_h,id_l;
 
    m_cs = 0;
    m_spi.write(MCP_CMD_READ_RX_BUFFER_0_ID);
    id_h = (unsigned short) m_spi.write(0xFF); //id high
    id_l = (unsigned short) m_spi.write(0xFF); //id low
    m_spi.write(0xFF); //extended id high(unused)
    m_spi.write(0xFF); //extended id low(unused)
    len = (m_spi.write(0xFF) & 0x0F); //data length code
    for (i = 0;i<len;i++) {
        data_out[i] = m_spi.write(0xFF);
    }
    m_cs = 1;
    (*length_out) = len;
    (*id_out) = ((id_h << 3) + ((id_l & 0xE0) >> 5)); //repack identifier

    bitModify(MCP_CANINTF_REGISTER, 0x03, 0x00);
}

/**
 * @brief send a message to the CAN bus using buffer 0
 * 
 * @param len 
 * @param id 
 * @param data 
 */
void MCP2515::send(uint8_t len, unsigned short id, uint8_t *data) {
    
    uint8_t i,id_high,id_low;
 
    //generate id bytes before spi write
    id_high = (uint8_t) (id >> 3);
    id_low = (uint8_t) ((id << 5) & 0x00E0);
 
    m_cs = 0;
    m_spi.write(MCP_CMD_LOAD_TX_BUFFER_0_ID);
    m_spi.write(id_high);         //identifier high bits
    m_spi.write(id_low);          //identifier low bits
    m_spi.write(0x00);            //extended identifier registers(unused)
    m_spi.write(0x00);
    m_spi.write(len);
    for (i=0;i<len;i++)
    {                           //load data buffer
        m_spi.write(data[i]);
    }
 
    m_cs = 1;

    wait_ms(10);
    // request to send
    m_cs = 0;
    m_spi.write(MCP_CMD_REQUEST_TO_SEND_BUFFER_0);
    m_cs = 1;
}
void MCP2515::setMode(MCP2515Mode mode) 
{
    bitModify(MCP_CANCTRL_REGISTER, MCP_CANCTRL_REQOP_MASK, mode);
}

/* -------------------------------------------------------------------------- */
/*                               Private methods                              */
/* -------------------------------------------------------------------------- */
/**
 * @brief reset the MCP2515
 * 
 */
void MCP2515::reset() {
    m_cs = 0;
    m_spi.write(MCP_CMD_RESET);
    m_cs = 1;
}

/**
 * @brief 
 * 
 * @param reg   : register address 
 * @param mask  : register mask
 * @param data  : data to write 
 */
void MCP2515::bitModify(uint8_t reg, uint8_t mask, uint8_t data) {
    m_cs = 0;
    m_spi.write(MCP_CMD_BIT_MODIFY);
    m_spi.write(reg);
    m_spi.write(mask);
    m_spi.write(data);
    m_cs = 1;
}

/**
 * @brief 
 * 
 * @param reg   : register address 
 * @param data  : data to write 
 * @return if the data was written correctly 
*/
bool MCP2515::writeRegisterConfirm(uint8_t reg, uint8_t data) 
{
    writeRegister(reg, data);
    uint8_t res = readRegister(reg);

    return res == data;
} 