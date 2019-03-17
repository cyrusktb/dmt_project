#include "web_client.h"
#include "command_processor.h"

// Serial to computer for debugging
Serial pc(USBTX, USBRX);

int main() {
    // Set PC baud to 115200
    pc.baud(115200);
    
    // Create the command processor
    CommandProcessor cmd_proc_;
    
    // Debug print statement
    pc.printf("\nSetting Up ESP8266...............");
    // Create a web client, giving it the serial pins and reset pin that
    // the ESP is connected to. Also pass the process message command of
    // the command processor in a lambda wrapper, to be called when a 
    // message is received.
    WebClient client(PA_9, PA_10, PA_0, [&cmd_proc_](std::string msg) {
        cmd_proc_.process_message(msg); 
    });
    // Debug print statement
    std::string err = client.get_error();
    if(err == "") {
        pc.printf("[\033[33mOK\033[0m]\n");
    }
    else {
        pc.printf("[\033[31mFAIL\033[0m]\n");
        pc.printf("\033[31mError\033[0m: %s\n", err.c_str());
    }

    while(true) {
        // Update the web client
        // This will call the given callback, telling the command processor
        // to process the commands in the message
        // The command processor will then set target motor positions //
        // motor vibration strengths.
        client.update();

        // Debugprint statement
        err = client.get_error();
        if(err != "")
            pc.printf("\033[31mERROR\033[0mm: %s\n", err.c_str());
    }
}
