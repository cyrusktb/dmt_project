#ifndef __TACTILE_CONTROLLER_H__
#define __TACTILE_CONTROLLER_H__

#include "mbed.h"
#include "multiplexer.h"
#include "drv2605.h"

#include "kinesthetix.h" // For serial for debugging

// The 7 bit i2c addresses are 0x70 and 0x72, but they need to be bit
// shifted by 1, giving 0xE0 and 0xE4 respectively
#define MULTI8ADDR 0xE0
#define MULTI2ADDR 0xE4

// Enum giving each motor a distinct number from 0 to 8
enum VibrationMotor {
    // Thumb: left, tip, right
    TMB_L = 0,
    TMB_T = 3,
    TMB_R = 4,

    // Index finger: left, tip, right
    INDX_L = 5,
    INDX_T = 6,
    INDX_R = 7,

    // Middle finger: left, tip, right
    MDL_L = 8,
    MDL_T = 1,
    MDL_R = 2
};

class TactileController {
public:
    // Constructor will initialise i2c on pins PB_7 and PB_6
    TactileController();
    ~TactileController();

    // Power ranges from 0 (none) to 127 (max)
    // Greater than 127 will be interpreted as 0
    void set_motor_power(char motor_power, VibrationMotor motor);
private:
    // I2C channel
    I2C i2c_;

    // 2-channel TCA9543A multiplexer
    Multiplexer multi_2_;
    // 8-channel TCA9548A multiplexer
    Multiplexer multi_8_;

    // The 9 vibration motor drivers are identical, and run on the same 
    // settings so we only need one instance to drive them via i2c
    Drv2605 lra_;
};

#endif // __TACTILE_CONTOLLER_H__
