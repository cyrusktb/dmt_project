#include "mbed.h"
#include "drv2605.h"
#include "tca9543a.h"

int main() {
    // Create switch controller
    DigitalIn switch_ctrl(PB_4);

    // Initialise I2C
    I2C i2c(PB_7, PB_6);
    
    // Initialise Haptic Motor Driver
    Drv2605 lra_driver(&i2c);

    // Initialise Multiplexer
    Tca9543a multiplexer(&i2c);

    // Set the multiplexer to output on channel 0
    multiplexer.select(1);

    // Set mode to real time playback
    lra_driver.set_mode(DRV2605_MODE_REALTIME);

    char effect = 1;
    signed char dir = 1;

    while(1) {
        // Set the effect to play
        lra_driver.set_realtime_value(effect);

        // wait a bit
        wait(0.010f);
        
        if(switch_ctrl) effect += 1 * dir;
        if(effect == 0) dir = -dir;
    }
}
