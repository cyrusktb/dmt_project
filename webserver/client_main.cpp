#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main(int argc, char **argv) {
    try {
        if(argc != 2) {
            std::cerr << "Usage: WebClient <host>" << std::endl;
            return 1;
        }
        // Create required io_service
        boost::asio::io_service io_service;
        
        // Server name resolver
        tcp::resolver resolver(io_service);
        
        // Query needs to know the name of the server and the service
        tcp::resolver::query query(argv[1], "daytime");

        // Get the list of end points
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

        // Create and connect the socket
        tcp::socket socket(io_service);
        boost::asio::connect(socket, endpoint_iterator);

        // Loop forever
        while(true) {
            boost::array<char, 128> buf;
            boost::system::error_code error;

            size_t len = socket.read_some(boost::asio::buffer(buf), error);

            // EOF means the server has closed the connection
            if(error == boost::asio::error::eof)
                break;
            else if(error)
                throw boost::system::system_error(error);

            std::cout.write(buf.data(), len);
        }
    }
    catch(std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}
