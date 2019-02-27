#include "tca9543a.h"

Tca9543a::Tca9543a(I2C *i2c, char i2c_addr) 
        :i2c_(i2c), i2c_addr_(i2c_addr) {
    // ctor
}

void Tca9543a::select(char channel) {
    char data = 0x08 | channel;
    i2c_->write(i2c_addr_, &data, 1);
}

void Tca9543a::disable() {
    char data = 0x00;
    i2c_->write(i2c_addr_, &data, 1);
}
