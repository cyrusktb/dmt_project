#ifndef __CONNECTION_MANAGER_HPP__
#define __CONNECTION_MANAGER_HPP__

#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
    
#include <mutex>

using boost::asio::ip::tcp;

/* 
 * Represents a TCP connection. Inherits from shared_ptr to allow 
 * lifespan beyond its first creation point
 */
class TCPConnection :public boost::enable_shared_from_this<TCPConnection> {
public:
    // Destructor
    virtual ~TCPConnection() {
        // Free memory
        delete timer_;
    }

    // Convenience typedef
    typedef boost::shared_ptr<TCPConnection> ptr;
    
    // Creation function - note constructor is private
    static ptr create(boost::asio::io_service &io_service) {
        return ptr(new TCPConnection(io_service));
    };

    tcp::socket &socket() {
        return socket_;
    };

    // Sets the message to be sent by this connection - thread safe
    void set_message(std::string message);
    
    // Start the socket and begin regular communication
    void start();
    
    // Reject the socket if we already have the required number 
    // of connections (one)
    void reject_full() {};

    // Check whether the connection is still working
    bool is_valid;
private:
    TCPConnection(boost::asio::io_service& io_service)
            : socket_(io_service), is_valid(true), interval_(20) {};
    
    // Continuously send a message via tcp
    void send_message(const boost::system::error_code& err);
    
    // Callback for when data needs to be sent along this connection
    void handle_write(const boost::system::error_code& err);

    tcp::socket socket_;
    std::string message_;
    std::mutex mutex_;

    // Deadline timer to control writing rate
    boost::asio::deadline_timer *timer_;
    // Delay
    boost::posix_time::milliseconds interval_;
};

/*
 * Manages incomming connections and handles sending messages to connected
 * clients. Only one client may be connected at a time, as we only have
 * one glove and one frame.
 */
class ConnectionManager {
public:
    ConnectionManager(boost::asio::io_service &io_service, 
                      unsigned int port);
    virtual ~ConnectionManager();
protected:
    // Main update loop for this object
    void main_loop();

    /*
     * This function will listen for incoming TCP connections and accept
     * them.
     */
    void start_accept();

    /*
     * Callback which runs whenever a new TCP connection is made
     */
    void handle_accept(TCPConnection::ptr new_connection,
                       const boost::system::error_code& error);
    
    // io_service to access I/O services
    boost::asio::io_service io_service_;
    // acceptor to accept incoming connections
    tcp::acceptor acceptor_;
    // active connection
    TCPConnection::ptr active_connection_;
};

#endif // __CONNECTION_MANAGER_HPP__
