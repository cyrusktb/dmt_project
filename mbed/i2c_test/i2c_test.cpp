#include "mbed.h"
#include "drv2605.h"
#include "multiplexer.h"

Serial pc(USBTX, USBRX);

int main() {
    // Initialise I2C
    I2C i2c(PB_7, PB_6);
    
    // Initialise Haptic Motor Driver
    Drv2605 lra_driver(&i2c);

    // Initialise Multiplexer
    Multiplexer multiplexer(&i2c, 0x70 << 1);

    // Set the multiplexer to output on channel 0
    char ch = 0;
    for(char i = 0; i < 8; i++) {
        multiplexer.select(i);

        lra_driver.setup();

        // Set mode to real time playback
        lra_driver.set_mode(DRV2605_MODE_REALTIME);

        lra_driver.set_realtime_value(0);
    }
    multiplexer.select(ch);

    char effect = 1;
    signed char dir = 1;

    pc.baud(115200);

    pc.printf("\r\nIt Begins\r\n");

    wait(1);

    while(1) {
        // Set the effect to play
        lra_driver.set_realtime_value(effect);

        // wait a bit
        wait(0.010f);
        
        effect += 1 * dir;
        if(effect == 0) {
            dir = -dir;
            if(dir > 0) {
                ch++;
                if(ch == 8)
                    ch = 0;
                int s = multiplexer.select(ch);
                pc.printf("CH:%d, s:%d, ", ch, s);
                char buf[2];
                i2c.read(0x70 << 1, buf, 1);
                pc.printf("Read: %d\r\n", buf[0]);
            }
        }
        //pc.printf("CH:%d\tE:%d\r\n", ch, effect);
    }
}
