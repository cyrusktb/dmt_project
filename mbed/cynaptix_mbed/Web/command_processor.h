#ifndef __COMMAND_PROCESSOR_H__
#define __COMMAND_PROCESSOR_H__

#include <string>

#include "messages.h"
#include "tactile_controller.h"
#include "kinesthetic_controller.h"

class CommandProcessor {
public:
    CommandProcessor(KinestheticController* k_ctrl);
    ~CommandProcessor();

    // Process a message from the server and call respective functions
    void process_message(std::string msg);

    // Create a message to send to the server and return
    std::string create_message();
private:
    // Control the vibration motors
    TactileController tactile_controller_;

    // Control the servo motors
    KinestheticController *kinesthetic_controller_;
};

#endif // __COMMAND_PROCESSOR_H__
