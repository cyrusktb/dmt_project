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
            switch(msg[++i]) {
            case Msgs::THUMB:
                mtr_num = 0;
                break;
            case Msgs::INDEX:
                mtr_num = 1;
                break;
            case Msgs::MIDDLE:
                mtr_num = 2;
                break;
            default:
                // Invalid so return
                pc.printf("Invalid motor: 0x%.2X\r\n", msg[i]);
                return;
            }

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
            for(char j = 0; j < 9; j++) {
                VibrationMotor m;
                switch(j) {
                    case 0:
                        m = VibrationMotor::TMB_L;
                        break;
                    case 1:
                        m = VibrationMotor::TMB_T;
                        break;
                    case 2:
                        m = VibrationMotor::TMB_R;
                        break;
                    case 3:
                        m = VibrationMotor::INDX_L;
                        break;
                    case 4:
                        m = VibrationMotor::INDX_T;
                        break;
                    case 5:
                        m = VibrationMotor::INDX_R;
                        break;
                    case 6:
                        m = VibrationMotor::MDL_L;
                        break;
                    case 7:
                        m = VibrationMotor::MDL_T;
                        break;
                    case 8:
                        m = VibrationMotor::MDL_R;
                        break;
                }
                tactile_controller_.set_motor_power(msg[++i], 
                                                    m);
            }
            break;
        // Other commands don't make sense to us
        default:
            pc.printf("Invalid command: 0x%.2X\r\n", msg[i]);
            return;
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
        switch(i) {
        case 0:
            msg += Msgs::THUMB;
            break;
        case 1:
            msg += Msgs::INDEX;
            break;
        case 2:
            msg += Msgs::MIDDLE;
            break;
        }

        // Convert the float into 4 bytes and store each as a character
        char *char_pos = (char*)&pos;
        for(char j = 0; j < 4; j++)
            msg += char_pos[j];
    }
    msg += Msgs::MSG_END;
    return msg;
}
