#include "web_client.h"

WebClient::WebClient(PinName tx, PinName rx, PinName reset, 
                     Callback<void(std::string)> message_callback) 
        :reset_(reset), error_(""), message_callback_(message_callback) {
    // Hardware reset
    reset_ = 0;

    wait(1);

    // Finish reset
    reset_ = 1;

    // Wait for initial status message to disappear
    wait(2);

    // Setup setial and parser
    esp_bs_ = new BufferedSerial(tx, rx);
    esp_ = new ATParser(*esp_bs_);

    // Set timeout
    esp_->setTimeout(DEFAULT_TIMEOUT);

    // Set the baud rate - the default baud is 115200
    esp_bs_->baud(115200);

    // Check that all is ok
    if(!(esp_->send("AT") && esp_->recv("OK")))
        set_error("ESP8266 did not respond to 'AT'");

    // Turn off AT command echoing
    esp_->send("ATE0") && esp_->recv("OK");

    // Connect to WiFi network
    if(!connect_to_network()) 
        set_error("Failed to connect to '" + SSID + "'.");

    // Connect via TCP
    if(!connect_TCP()) 
        set_error("Failed to connect to '" + REMOTE_IP + "'.");
}

WebClient::~WebClient() {
    // Free memory
    delete esp_;
    delete esp_bs_;
}

std::string WebClient::get_error() {
    // Copy the error
    std::string ret = error_;

    // Error has been processed to reset
    error_ = "";

    return ret;
}

void WebClient::update() {
    int len;
    // Read incoming TCP messages if existant
    if(esp_->recv("+IPD,%d:", &len)) {
        memset(buf_, '\0', len);
        // Add data read to the storage string
        data_ += buf_;
    }

    // Check if a full message has been received yet
    if(data_.size() > 0) {
        bool started = false;
        char start;
        char end;
        std::string message;
        for(char i = 0; i < data_.size(); i++) {
            // If there's a start of message byte and we haven't started yet
            if(data_[i] == Msgs::MSG_STRT && !started) {
                start = i;
                started = true;
            }
            // If there's an end of bessage byte, and we've started
            else if(data_[i] == Msgs::MSG_END && started) {
                // Store the last end position
                end = i;

                // Add the data to the message string
                for(char j = start+1; j != end; j++) {
                    message += data_[j];
                }
            }
        }

        // If a message was fully received then call the callback
        if(message.size()) {
            message_callback_(message);
        }
    }
}

bool WebClient::connect_to_network() {
    bool ret = true;
    
    // Longer timeout
    esp_->setTimeout(8000);
    
    // Try to connect
    if(!(esp_->send("AT+CWJAP=\"%s\",\"%s\"", SSID, PWD) && 
         esp_->recv("OK")))
        ret = false;
    
    // Reset timeout
    esp_->setTimeout(DEFAULT_TIMEOUT);

    return ret;
}

bool WebClient::connect_TCP() {
    bool ret = false;
    
    // Single TCP connection
    esp_->send("AT+CIPMUX=0") && esp_->recv("OK");

    // Longer timeout for this
    esp_->setTimeout(8000);

    // Make the connection
    if(!(esp_->send("AT+CIPSTART=\"TCP\",\"%s\",%d", REMOTE_IP, REMOTE_PORT)
         && esp_->recv("OK"))) {
        ret = true;
    }
    else {
        // If the connection was closed by the server we have to close it
        // Before we can open a new one, causing the above command to fail
        esp_->send("AT+CIPCLOSE") && esp_->recv("OK");

        // Try to connect again
        if(!(esp_->send("AT+CIPSTART=\"TCP\",\"%s\",%d", REMOTE_IP, REMOTE_PORT)
             && esp_->recv("OK"))) {
            ret = true;
        }
    }

    // Reset timeout
    esp_->setTimeout(DEFAULT_TIMEOUT);

    return ret;
}

bool WebClient::check_connection_status() {
    bool ret = false;

    esp_->send("AT+CIPSTATUS");
    int status;
    esp_->recv("STATUS:%d", &status);

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
            set_error("Lost connection with '" + REMOTE_IP + "'.");
        else
            ret = true;
        break;
    // Not connected to a network
    case 5:
        if(!connect_to_network())
            set_error("Lost connection with '" + SSID + "'.");
        else if(!connect_TCP())
            set_error("Failed to connect to '" + REMOTE_IP + "'.");
        else
            ret = true;
        break;
    }
    
    return ret;
}

void WebClient::set_error(std::string err) {
    // Don't overwrite the previous error, as errors often 
    // end up causing other errors
    if(error_ == "") {
        error_ = err;
    }
}
