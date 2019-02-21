#include "mbed.h"
#include "ATParser.h"

Serial pc(USBTX, USBRX);
// ESP-01 tx and rx pins
BufferedSerial *esp_bs;
// AT command parser
ATParser *esp;

DigitalOut reset(PA_0);

#define REMOTE_IP "10.42.0.1"
#define REMOTE_PORT 53821

char ssid[32] = "nickick-laptop";
char pwd[32] = "auSEtdYV";

// Make a single TCP connection - multiple connections are not allowed
// Returns success // fail
bool connect_TCP() {
    // Single TCP connection
    esp->send("AT+CIPMUX=0") && esp->recv("OK");

    // Make the connection
    if(!(esp->send("AT+CIPSTART=TCP,%s,%d", REMOTE_IP, REMOTE_PORT) &&
         esp->recv("OK"))) {
        return false;
    }
    return true;
}

void setup() {
    // Hardware reset
    reset = 0;
    
    // Setup PC communication
    pc.baud(115200);
    pc.printf("\n---------- Setting Up ESP8266 ----------\n");

    wait(1);

    // Finish reset
    reset = 1;
    
    // Wait for crazy stuff to disappear
    wait(2);
    
    // Setup serial and parser
    esp_bs = new BufferedSerial(PA_9, PA_10);
    esp = new ATParser(*esp_bs);

    // Set baud rate of esp
    esp_bs->baud(115200);

    // Check that all is ok
    if(! (esp->send("AT") && esp->recv("OK")))
        pc.printf("ESP8266 not ok :(");
    
    // Turn off AT command echoing
    esp->send("ATE0") && esp->recv("OK");

    // Connect via TCP
    if(!connect_TCP()) 
        pc.printf("Failed to connect to \"%s\". RIP", ssid);

    pc.printf("\n---------- Setup Complete ----------\n");
}

int main() {
    setup();

    while(true) {

    }
}
