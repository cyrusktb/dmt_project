#include "multiplexer.h"

Multiplexer::Multiplexer(I2C *i2c, char i2c_addr) 
        :i2c_(i2c), i2c_addr_(i2c_addr) {
    // ctor
}

int Multiplexer::select(char channel) {
    char data[] = { 0x01 << channel };
    return i2c_->write(i2c_addr_, data, 1);
}

int Multiplexer::disable() {
    char data[] = { 0x00 };
    return i2c_->write(i2c_addr_, data, 1);
}
