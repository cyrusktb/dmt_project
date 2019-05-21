#include "web_client.h"
#include "command_processor.h"
#include "kinesthetic_controller.h"

// Serial to computer for debugging
// Serial pc(USBTX, USBRX);

int main() {
    // Set PC baud to 115200
    pc.baud(115200);

    pc.printf("\r\n --------------- So It Begins... --------------- \r\n");

    // Create the kinesthetic feedback controller
    KinestheticController kin_ctrl;

    // Create the command processor
    CommandProcessor cmd_proc(&kin_ctrl);

    // Debug print statement
    pc.printf("\r\nSetting Up ESP8266...............");
    // Create a web client, giving it the serial pins and reset pin that
    // the ESP is connected to. Also pass the process message command of
    // the command processor in a lambda wrapper, to be called when a 
    // message is received.
    WebClient client(PA_9, PA_10, PA_0, [&cmd_proc](std::string msg) {
        cmd_proc.process_message(msg); 
    });
    // Debug print statement
    std::string err = client.get_error();
    if(err == "") {
        pc.printf("[\033[33mOK\033[0m]\r\n");
    }
    else {
        pc.printf("[\033[31mFAIL\033[0m]\r\n");
        pc.printf("\033[31mError\033[0m: %s\r\n", err.c_str());
    }
    char it = 0;
    while(true) {
        // Update the web client
        // This will call the given callback, telling the command processor
        // to process the commands in the message
        // The command processor will then set target motor positions //
        // motor vibration strengths.
        client.update();

        std::string msg = "";

        msg += Msgs::VIB_STR;
        msg += (char)it;
        msg += (char)it;
        msg += (char)it;
        msg += (char)it;
        msg += (char)it;
        msg += (char)it;
        msg += (char)it;
        msg += (char)it;
        msg += (char)it;

        it = (it+1) %128;

        cmd_proc.process_message(msg);

        pc.printf("it: %d\tmsg: %s\r\n", it, msg.c_str());

        // Debug print statement
        err = client.get_error();
        if(err != "")
            pc.printf("\033[31mERROR\033[0m: %s\r\n", err.c_str());

        // Update the control of the servos
        kin_ctrl.update();

        // Send the position of the servos back to the server
//        client.send(cmd_proc.create_message());
    }
}
