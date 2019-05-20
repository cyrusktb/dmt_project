#include "connection_manager.hpp"

void TCPConnection::set_message(std::string msg) {
    // Prevent msg being changed while a message is being sent or modified
    std::lock_guard<std::mutex> lock(mutex_);

    // Change the string
    message_ = msg;
}

void TCPConnection::start() {
    // Todo: Send the start message and initiate writing loop 

    // Create the timer object
    timer_ = new boost::asio::deadline_timer(socket_.get_io_service(), 
                                             interval_);

    // Start the infinite send_message loop
    timer_->async_wait(boost::bind(&TCPConnection::send_message,
                                  this,
                                  boost::asio::placeholders::error));
}

void TCPConnection::send_message(const boost::system::error_code &err) {
    // Send the message
    handle_write(err);

    // Set the timer to run again after the interval
    // Keep any overshot time
    timer_->expires_at(timer_->expires_at() + interval_);

    // Run the function again
    timer_->async_wait(boost::bind(&TCPConnection::send_message,
                                  this,
                                  boost::asio::placeholders::error));
}

void TCPConnection::handle_write(const boost::system::error_code& err) {
    // Prevent msg being changed while message is being sent
    std::lock_guard<std::mutex> lock(mutex_);

    // Send the message
    boost::asio::write(socket_, boost::asio::buffer(message_));
}

ConnectionManager::ConnectionManager(boost::asio::io_service &io_service,
                                     unsigned int port)
        : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)) {
    // Create initial fake connection to prevent pointer errors
    // This also tests whether valid works or not
    active_connection_ = TCPConnection::create(io_service);
    active_connection_->is_valid = false;
}

ConnectionManager::~ConnectionManager() {
    // dtor
}

void ConnectionManager::start_accept() {
    TCPConnection::ptr new_connection = 
        TCPConnection::create(acceptor_.get_io_service());

    // Accept the connection and handle asynchronously via handle_accept
    acceptor_.async_accept(new_connection->socket(),
                           boost::bind(&ConnectionManager::handle_accept,
                                       this,
                                       new_connection,
                                       boost::asio::placeholders::error));
}

void ConnectionManager::handle_accept(TCPConnection::ptr new_connection,
                                const boost::system::error_code& error) {
    if(!error) {
        // Check if our current connection is valid and reject if it is
        if(active_connection_->is_valid) {
            new_connection->reject_full();
        }
        else {
            new_connection->start();
        }
    }

    start_accept();
}
