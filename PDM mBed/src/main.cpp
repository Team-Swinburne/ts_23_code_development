#include "mbed.h"

Serial device(PA_9, PA_10);  // tx, rx

int main() {
    device.baud(9600);
    device.printf("Hello World\n");
while(1){
    device.printf("Hello World\n");
    wait_ms(1000);
}
}