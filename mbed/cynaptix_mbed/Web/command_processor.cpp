#include "command_processor.h"

CommandProcessor::CommandProcessor(KinestheticController *k_ctrl)
        :kinesthetic_controller_(k_ctrl) {
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
            
            // Set the target position of the finger
            kinesthetic_controller_->set_finger_target(mtr_num, mtr_pos);
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

std::string CommandProcessor::create_message() {
    std::string msg;
    msg += Msgs::MSG_STRT;
    // Get the position of each servo
    for(char i = 0; i < 3; i++) {
        float pos = kinesthetic_controller_->get_finger_pos(i);
        
        // Message contains a motor position
        msg += Msgs::MTR_POS;

        // Which motor is it?
        msg += i;

        // Convert the float into 4 bytes and store each as a character
        char *char_pos = (char*)&pos;
        for(char j = 0; j < 4; j++)
            msg += char_pos++;
    }
    msg += Msgs::MSG_END;
    return msg;
}
