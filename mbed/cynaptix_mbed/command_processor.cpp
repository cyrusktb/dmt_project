#include "command_processor.h"

CommandProcessor::CommandProcessor() {
    // ctor
}

CommandProcessor::~CommandProcessor() {
    // dtor
}

void CommandProcessor::process_message(std::string msg) {
    // Used when a motor position is set
    char mtr_num;
    float mtr_pos;

    // Loop through each character in the string
    for(unsigned short i = 0; i < msg.size(); i++) {
        switch(msg[i]) {
        // Target motor position message
        case Msgs::TRGT_MTR_POS:
            // Read the next 5 bytes

            // Byte 1 is the motor number
            mtr_num = msg[++i];

            // Bytes 2-5 are a 32 bit float representing the position
            for(char j = 0; j < 4; j++)
                ((char*)&mtr_pos)[j] = msg[++i];
            
            break;
        // Target vibration strength message
        case Msgs::VIB_STR:
            // Read the next 9 bytes

            // Each byte is a vibration motor amplitude
            for(char j = 0; j < 9; j++)
                tactile_controller_.set_motor_power(msg[++i], 
                                                   (VibrationMotor)j);
            break;
        // Other commands don't make sense to us
        default:
            break;
        }
    }
}
