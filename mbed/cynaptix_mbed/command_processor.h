#ifndef __COMMAND_PROCESSOR_H__
#define __COMMAND_PROCESSOR_H__

#include <string>

#include "messages.h"
#include "tactile_controller.h"

class CommandProcessor {
public:
    CommandProcessor();
    ~CommandProcessor();

    void process_message(std::string msg);
private:
    TactileController tactile_controller_;
};

#endif // __COMMAND_PROCESSOR_H__
