#ifndef __TCA9543A_H__
#define __TCA9543A_H__

#include "mbed.h"

class Tca9543a {
public:
    // Constructor
    Tca9543a(I2C *i2c, char i2c_addr = 0xE4);

    // Enable a channel
    void select(char channel);
    
    // Disable all channels
    void disable();

private:
    I2C *i2c_;
    char i2c_addr_;
};

#endif // __TCA9543A_H__
