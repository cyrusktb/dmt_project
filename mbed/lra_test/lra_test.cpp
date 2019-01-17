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

    // Select LRA effect library
    lra_driver.select_library(DRV2605_LIB_LRA);

    char effect = 1;

    while(1) {
        // Display effect
        print_num(effect);

        // Set the effect to play
        lra_driver.set_waveform(0, effect); // play effect
        lra_driver.set_waveform(1, 0); // end effect

        // Play the effect
        lra_driver.go();

        // wait a bit
        wait(1.0f);
        
        if(switch_ctrl) effect++;
        if(effect > 123) effect = 1;
    }
}
