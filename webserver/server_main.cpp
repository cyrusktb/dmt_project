#include <ctime>
#include <iostream>
#include <string>

#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using namespace std;

std::string make_daytime_string() {
    time_t now = time(0);
    return ctime(&now);
}

int main() {
    try {
        // Required for other items to communicate
        boost::asio::io_service io_service;
    
        // Listens for new tcp connections
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), 13));
        
        // Loop forever listening for new connections
        while(true) {
            // Create a socket
            tcp::socket socket(io_service);
            acceptor.accept(socket);
            
            std::string message = make_daytime_string();

            boost::system::error_code ignored_error;
            boost::asio::write(socket, 
                               boost::asio::buffer(message), 
                               ignored_error);
        }
    }
    catch(std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}
