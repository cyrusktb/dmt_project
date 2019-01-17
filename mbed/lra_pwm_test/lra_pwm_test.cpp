#include "mbed.h"
#include "drv2605.h"

DigitalOut* leds[7];

void print_num(char num) {
    for(char i = 0; i < 7; i++) {
        if((num >> i) & 0x01) {
            (*leds[i]) = 1;
        }
        else {
            (*leds[i]) = 0;
        }
    }
}

int main() {
    // Create LEDs
    leds[0] = new DigitalOut(PA_0);
    leds[1] = new DigitalOut(PA_1);
    leds[2] = new DigitalOut(PA_3);
    leds[3] = new DigitalOut(PA_4);
    leds[4] = new DigitalOut(PF_0);
    leds[5] = new DigitalOut(PB_1);
    leds[6] = new DigitalOut(PA_7);

    // Create switch controller
    DigitalIn switch_ctrl(PB_4);

    // Initialise I2C
    I2C i2c(PB_7, PB_6);
    
    // Initialise Haptic Motor Driver
    Drv2605 lra_driver(&i2c);

    // Set mode to real time playback
    lra_driver.set_mode(DRV2605_MODE_REALTIME);

    char effect = 1;
    signed char dir = 1;

    while(1) {
        // Display effect
        print_num(effect);

        // Set the effect to play
        lra_driver.set_realtime_value(effect);

        // wait a bit
        wait(0.10f);
        
        if(switch_ctrl) effect += 1 * dir;
        if(effect == 0) dir = -dir;
    }
}
