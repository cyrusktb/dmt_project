#include "mbed.h"
#include "ATParser.h"

Serial pc(USBTX, USBRX);
// ESP-01 tx and rx pins
BufferedSerial *esp_bs;
// AT command parser
ATParser *esp;

DigitalOut reset(PA_0);

#define REMOTE_IP "10.42.0.1"
#define REMOTE_PORT 13 /*53821*/
#define DEFAULT_TIMEOUT 50

char ssid[32] = "nickick-laptop";
char pwd[32] = "auSEtdYV";

bool connect_to_network() {
    bool ret = true;
    
    // Longer timeout
    esp->setTimeout(8000);
    
    // Try to connect
    if(!(esp->send("AT+CWJAP=\"%s\",\"%s\"", ssid, pwd) && 
         esp->recv("OK")))
        ret = false;
    
    // Reset timeout
    esp->setTimeout(DEFAULT_TIMEOUT);

    if(ret)
        pc.printf("Successfully connected to \"%s\".\n", ssid);

    return ret;
}

// Make a single TCP connection - multiple connections are not allowed
// Returns success // fail
bool connect_TCP() {
    bool ret = false;
    
    // Single TCP connection
    esp->send("AT+CIPMUX=0") && esp->recv("OK");

    // Longer timeout for this
    esp->setTimeout(8000);

    // Make the connection
    if(!(esp->send("AT+CIPSTART=\"TCP\",\"%s\",%d", REMOTE_IP, REMOTE_PORT)
         && esp->recv("OK"))) {
        ret = true;
    }
    else {
        // If the connection was closed by the server we have to close it
        // Before we can open a new one, causing the above command to fail
        esp->send("AT+CIPCLOSE") && esp->recv("OK");

        // Try to connect again
        if(!(esp->send("AT+CIPSTART=\"TCP\",\"%s\",%d", REMOTE_IP, REMOTE_PORT)
             && esp->recv("OK"))) {
            ret = true;
        }
    }

    // Reset timeout
    esp->setTimeout(DEFAULT_TIMEOUT);

    if(ret) 
        pc.printf("Successfully connected to \"%s\".\n", REMOTE_IP);

    return ret;
}

bool check_connection_status() {
    bool ret = false;

    esp->send("AT+CIPSTATUS");
    int status;
    esp->recv("STATUS:%d", &status);

    switch(status) {
    // TCP or UDP connection
    case 3:
    ret = true;
        break;
    // Connected to network and IP obtained - not connected to TCP
    case 2:
    // TCP or UDP is disconnected
    case 4:
        if(!connect_TCP())
            pc.printf("Lost connection with \"%s\".\n", REMOTE_IP);
        else
            ret = true;
        break;
    // Not connected to a network
    case 5:
        if(!connect_to_network())
            pc.printf("Lost connection with \"%s\".\n", ssid);
        else if(!connect_TCP())
            pc.printf("Failed to connect to \"%s\".\n", REMOTE_IP);
        else
            ret = true;
        break;
    }
    
    return ret;
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
    
    // Set timeout
    esp->setTimeout(DEFAULT_TIMEOUT);

    // Set baud rate of esp
    esp_bs->baud(115200);

    // Check that all is ok
    if(!(esp->send("AT") && esp->recv("OK")))
        pc.printf("ESP8266 not ok :(");
    
    // Turn off AT command echoing
    esp->send("ATE0") && esp->recv("OK");

    // Connect to WiFi network
    if(!connect_to_network())
        pc.printf("Failed to connect to \"%s\". Soz m8", ssid);

    // Connect via TCP
    if(!connect_TCP()) 
        pc.printf("Failed to connect to \"%s\". RIP", REMOTE_IP);

    pc.printf("\n---------- Setup Complete ----------\n");
}

int main() {
    setup();
    char buf[256];
    int len;
    
    // Set the timeout to be very low now as we are looking for many 
    // different messages
    // esp->setTimeout(5);

    Timer status_timer;
    status_timer.start();

    int status_check_delay_ms = 2000;

    while(true) {
        // Read incoming TCP messages if existant
        if(esp->recv("+IPD,%d:", &len)) {
            memset(buf, '\0', len);
            int data = esp->read(buf, len);
            pc.printf("Received: %.*s\n", data, buf);
        }
        
        // Check status
        if(status_timer.read_ms() > status_check_delay_ms) {
            bool connected = check_connection_status();
            if(connected)
                status_check_delay_ms = 2000;
            else
                status_check_delay_ms = 200;
            status_timer.reset();
        }
    }
}
