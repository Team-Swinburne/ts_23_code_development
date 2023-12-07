#include <mbed.h>
#include "RadioCom.h"
#include "mcp2515.h"

#define CANBUS_FREQUENCY 500000

/* -------------------------------------------------------------------------- */
/*                                 INTERFACES                                 */
/* -------------------------------------------------------------------------- */

// Radiocom Interface
RadioCom radioCom(PB_10, PB_11);             // TXD, RXD

// SPI Interface
SPI spi(PB_5, PB_4, PB_3);                // mosi, miso, sclk

// CANBUS Interface
CAN can1(PB_8, PB_9);                     // RXD, TXD

// Ticker
Ticker ticker_heartbeat;

// I/O
DigitalOut led1(PC_13);

/* -------------------------------------------------------------------------- */
/*                                  CALLBACKS                                 */
/* -------------------------------------------------------------------------- */
void can1_recv_cb() 
{
  //led1 = !led1;
  CANMessage msg;

  if (can1.read(msg)) 
  {
    radioCom.transmit(msg.data, msg.len, msg.id);
    led1 = !led1;
  }
}

/* -------------------------------------------------------------------------- */
/*                                    Setup                                   */
/* -------------------------------------------------------------------------- */
void setup() 
{
    radioCom.begin(57600);

    // Disable interrupts for smooth startup routine.
    wait_ms(3000);
  
    __disable_irq();

    can1.monitor(true);
    can1.filter(0x250, 0x7F8, CANStandard, 0); //only allow 0x 25x
    can1.frequency(CANBUS_FREQUENCY);
    can1.attach(&can1_recv_cb);

    // Re-enable interrupts again, now that interrupts are ready.
	__enable_irq();

	// Allow some time to settle!
	wait_ms(3000);
}
int main() {

  setup();

  while(1) {
  }
}