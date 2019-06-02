#ifndef __MULTIPLEXER_H__
#define __MULTIPLEXER_H__

#include "mbed.h"

class Multiplexer {
public:
    // Constructor
    Multiplexer(I2C *i2c, char i2c_addr = 0xE4);

    // Enable a channel
    int select(char channel);
    
    // Disable all channels
    int disable();

private:
    I2C *i2c_;
    char i2c_addr_;
};

#endif // __MULTIPLEXER_H__
