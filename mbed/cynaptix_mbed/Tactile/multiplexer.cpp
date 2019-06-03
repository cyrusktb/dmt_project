#include "multiplexer.h"

Multiplexer::Multiplexer(I2C *i2c, char i2c_addr, bool is_double) 
        :i2c_(i2c), i2c_addr_(i2c_addr), is_double_(is_double) {
    // ctor
}

void Multiplexer::select(char channel) {
    char data = 0x01 << channel;
    // The two channel one uses a different addressing method
    if(is_double_) data = 0x08 | channel;
    i2c_->write(i2c_addr_, &data, 1);
}

void Multiplexer::disable() {
    char data = 0x00;
    i2c_->write(i2c_addr_, &data, 1);
}
