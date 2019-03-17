#include "tactile_controller.h"

TactileController::TactileController() 
        :multi_2_(&i2c_), multi_8_(&i2c_), lra_(&i2c_), i2c_(PB_7, PB_6) {
    // ctor
};

TactileController::~TactileController() {
    // dtor
};

void TactileController::set_motor_power(char motor_power, 
                                        VibrationMotor motor) {
    switch(motor) {
    case MDL_R:
        multi_2_.select(0);
        break;
    default:
        multi_8_.select(motor);
        break;
    }
    lra_.set_realtime_value(motor_power);
    switch(motor) {
    case MDL_R:
        multi_2_.disable();
        break;
    default:
        multi_8_.disable();
        break;
    }
}
