#include "multiplexer.h"

Multiplexer::Multiplexer(I2C *i2c, char i2c_addr) 
        :i2c_(i2c), i2c_addr_(i2c_addr) {
    // ctor
}

void Multiplexer::select(char channel) {
    char data = 0x08 | channel;
    i2c_->write(i2c_addr_, &data, 1);
}

void Multiplexer::disable() {
    char data = 0x00;
    i2c_->write(i2c_addr_, &data, 1);
}
