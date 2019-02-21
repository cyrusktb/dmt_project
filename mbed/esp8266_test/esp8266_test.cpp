#include "mbed.h"
 
Serial pc(USBTX, USBRX);
Serial esp(PA_9, PA_10); // tx, rx
DigitalOut reset(PA_0);

#define REMOTE_IP "192.168.1.1"
#define REMOTE_PORT 54321

// Circular for serial receive data
const unsigned int data_buf_size = 256;
volatile char data_buf[data_buf_size];
volatile unsigned int data_start;
volatile unsigned int data_end;

// Serial interrupt -> read ESP data
void esp_callback() {
    while(esp.readable()) {
        data_buf[data_end] = esp.getc();

        // Wrap around circular buffer
        data_end = ++data_end % data_buf_size;
    }
}

// Get current serial data buffer
// Returns the size of the contents
int get_serial_data(char *out, int len) {
    int out_pos = 0;
    // Loop until either string is full, 
    // or all of the data buffer has been read
    while(data_start != data_end && out_pos < len-1) {
        // Get the next character
        out[out_pos] = data_buf[data_start];
        // Wrap around circular buffer
        data_start = ++data_start % data_buf_size;
        // Don't overwrite characters
        out_pos++;
    }
    // Set null character to terminate the string
    out[out_pos] = '\0';
    return out_pos;
}

// Make a single TCP connection - multiple connections are not allowed
// Returns success // fail
bool connect_TCP() {
    // Single TCP connection
    esp.printf("AT+CIPMUX=0\r\n");

    wait(0.5);

    // Clear buffer
    data_start = data_end;

    // Make the connection
    esp.printf("AT+CIPSTART=TCP,%s,%d\r\n", REMOTE_IP, REMOTE_PORT);

    wait(1);

    // Check result
    char result[32];
    get_serial_data(result, 32);

    pc.printf("Result: %s", result);

    if(result == "OK" || result == "ALREADY CONNECTED")
        return true;
    else
        return false;
}

// Send a message to the host server
void send_message(char *msg, int len) {
    // Tell volume of data to be transmitted
    esp.printf("AT+CIPSEND=%d\r\n", len);

    // Send the data, no carriage return
    esp.printf(msg);
}

void setup() {
    // Hardware reset
    reset = 0;
    // Set baud rate
    pc.baud(115200);
    pc.printf("\n------------- Setting Up ESP8266 -------------\n");

    // Setup serial buffers
    data_start = 0;
    data_end = 0;

    wait(1);

    // Finish reset
    reset = 1;

    // Set esp baud rate
    esp.baud(115200);

    // Attach callback to esp to get called whenever it interrupts
    esp.attach(&esp_callback);

    // Turn off AT command echoing (set to 1 to help debugging)
    esp.printf("ATE0\r\n");

    wait(1);

    // Set the TCP receive mode to active mode:
    // Whenever data is received, +IPD,<len>:<data> will be sent to us
    esp.printf("AT+CIPRECVMODE?\r\n");

    wait(1);
    
    // Connect via TCP
    connect_TCP();
    
    wait(0.5);
    // Check connection status

    pc.printf("\n------------- Setup Completed -------------\n");
}

int main() {
    setup();
    
    const int buf_size = 16;
    char buf[buf_size];
    int count = 0;

    Timer timer;
    timer.start();
    while(1) {
        int size = get_serial_data(buf, buf_size);
        if(size) {
            char msg[size];
            for(char i = 0; i < size; i++)
                msg[i] = buf[i];
            pc.printf("%s\r\n", msg);
        }
        if(timer.read_ms() > 1000) {
            timer.reset();
            count++;
            pc.printf("\n---------- Runtime: %d s ----------\n", count);
        }
        wait(0.1);
    }
}
