#include <mbed.h>
// https://www.nutsvolts.com/magazine/article/February2017_CAN-Bus-Add-Controller-to-MCU

#define MCP_8MHz_1000kBPS_CFG1 (0x00)
#define MCP_8MHz_1000kBPS_CFG2 (0x80)
#define MCP_8MHz_1000kBPS_CFG3 (0x80)

#define MCP_8MHz_500kBPS_CFG1 (0x00)
#define MCP_8MHz_500kBPS_CFG2 (0x90)
#define MCP_8MHz_500kBPS_CFG3 (0x82)

#define MCP_8MHz_250kBPS_CFG1 (0x00)
#define MCP_8MHz_250kBPS_CFG2 (0xB1)
#define MCP_8MHz_250kBPS_CFG3 (0x85)

#define MCP_8MHz_125kBPS_CFG1 (0x01)
#define MCP_8MHz_125kBPS_CFG2 (0xB1)
#define MCP_8MHz_125kBPS_CFG3 (0x85)

enum MCP2515Mode 
{
    CONFIGURATION   = 0x80,
    NORMAL          = 0x00,
    SLEEP           = 0x20,
    LISTEN          = 0x60,
    LOOPBACK        = 0x40
};

class MCP2515 {
    public:
        MCP2515(SPI& spi, PinName cs);

        bool begin(int busFrequency);

        void send(uint8_t len, unsigned short id, uint8_t *data);
        void receive(uint8_t* length_out,uint8_t *data_out,unsigned short *id_out);

        void setMode(MCP2515Mode mode);
        void writeRegister(uint8_t reg, uint8_t data);
        uint8_t readRegister(uint8_t reg);

    private:
        void reset();
        void bitModify(uint8_t address, uint8_t mask, uint8_t data);

        bool writeRegisterConfirm(uint8_t reg, uint8_t data);
        /* ---------------------------- MCP2515 Commands ---------------------------- */
        const unsigned int MCP_CMD_RESET                     = 0xC0;

        const unsigned int MCP_CMD_READ                      = 0x03;
        const unsigned int MCP_CMD_READ_RX_BUFFER_0_ID       = 0x90;
        const unsigned int MCP_CMD_READ_RX_BUFFER_0_DATA     = 0x92;
        const unsigned int MCP_CMD_READ_RX_BUFFER_1_ID       = 0x94;
        const unsigned int MCP_CMD_READ_RX_BUFFER_1_DATA     = 0x96;
        
        const unsigned int MCP_CMD_WRITE                     = 0x02;
        const unsigned int MCP_CMD_LOAD_TX_BUFFER_0_ID       = 0x40;
        const unsigned int MCP_CMD_LOAD_TX_BUFFER_0_DATA     = 0x41;
        const unsigned int MCP_CMD_LOAD_TX_BUFFER_1_ID       = 0x42;
        const unsigned int MCP_CMD_LOAD_TX_BUFFER_1_DATA     = 0x43;
        const unsigned int MCP_CMD_LOAD_TX_BUFFER_2_ID       = 0x44;
        const unsigned int MCP_CMD_LOAD_TX_BUFFER_2_DATA     = 0x45;
        const unsigned int MCP_CMD_REQUEST_TO_SEND_BUFFER_0  = 0x80;
        const unsigned int MCP_CMD_REQUEST_TO_SEND_BUFFER_1  = 0x81;
        const unsigned int MCP_CMD_REQUEST_TO_SEND_BUFFER_2  = 0x82;

        const unsigned int MCP_CMD_RX_STATUS                 = 0xB0;
        const unsigned int MCP_CMD_BIT_MODIFY                = 0x05;
        /* ---------------------------- MCP2515 Registers --------------------------- */
        const unsigned int MCP_BFPCTRL_REGISTER   = 0x0C;
        const unsigned int MCP_TXRTSCTRL_REGISTER = 0x0D;
        const unsigned int MCP_CANSTAT_REGISTER   = 0x0E;
        const unsigned int MCP_CANCTRL_REGISTER   = 0x0F;
        const unsigned int MCP_TEC_REGISTER       = 0x1C;
        const unsigned int MCP_REC_REGISTER       = 0x1D;
        const unsigned int MCP_RXM0SIDH_REGISTER  = 0x20;
        const unsigned int MCP_RXM0SIDL_REGISTER  = 0x21;
        const unsigned int MCP_RXM1SIDH_REGISTER  = 0x24;
        const unsigned int MCP_RXF0SIDL_REGISTER  = 0x01;
        const unsigned int MCP_RXF1SIDL_REGISTER  = 0x05;
        const unsigned int MCP_CNF3_REGISTER      = 0x28;
        const unsigned int MCP_CNF2_REGISTER      = 0x29;
        const unsigned int MCP_CNF1_REGISTER      = 0x2A;
        const unsigned int MCP_CANINTE_REGISTER   = 0x2B;
        const unsigned int MCP_CANINTF_REGISTER   = 0x2C;
        const unsigned int MCP_EFLG_REGISTER      = 0x2D;
        const unsigned int MCP_TXB0CTRL_REGISTER  = 0x30;
        const unsigned int MCP_TXB0DLC_REGISTER   = 0x35;
        const unsigned int MCP_TXB1CTRL_REGISTER  = 0x40;
        const unsigned int MCP_TXB2CTRL_REGISTER  = 0x50;
        const unsigned int MCP_RXB0CTRL_REGISTER  = 0x60;
        const unsigned int MCP_RXB1CTRL_REGISTER  = 0x70;

        /* ------------------------- MCP2515 REGISTER MASKS ------------------------- */
        const unsigned int MCP_READ_STATUS_RX0IF 		= (1U << 0);
        const unsigned int MCP_READ_STATUS_RX1IF 		= (1U << 1);
        const unsigned int MCP_RX_STATUS_RX0IF_MASK     = (1U << 6);
        const unsigned int MCP_RX_STATUS_RX1IF_MASK 	= (1U << 7);
        const unsigned int MCP_CANCTRL_REQOP_MASK	    = (7U << 5);
        const unsigned int MCP_CNF3_MASK 			    = 0xC7;
        const unsigned int MCP_RXnCTRL_RXM_MASK		    = (3U << 5);
        const unsigned int MCP_CANCTRL_CLKOUT_MASK	    = (1U << 2);
        const unsigned int MCP_CANCTRL_CLKPRE_MASK	    = (3U << 0);
        const unsigned int MCP_RXFnSIDL_EXITE_MASK	    = (1U << 3);
        const unsigned int MCP_CANINTE_RXB0_MASK		= 1U;

        SPI &m_spi;
        DigitalOut m_cs;
};