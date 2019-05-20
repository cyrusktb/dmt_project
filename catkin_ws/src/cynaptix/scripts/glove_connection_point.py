#!/usr/bin/env python

import rospy

import socket
import sys
import threading

import glove_connection.message_processor as mp

def process_connection(conn, rate):
    # Double the frequency as we call the sleep function twice
    time_keeper = rospy.Rate(2*rate)

    data = ""

    while not rospy.is_shutdown():
        # Send data to the glove
        msg = mp.get_glove_message()
        try:
            conn.send(msg)
        except socket.timeout:
            rospy.loginfo("Connection timed out.")
            return

        time_keeper.sleep()
        
        # Receive data from the glove
        try:
            data = conn.recv()
        except:
            rospy.loginfo("Connection timed out.")
            return

        mp.process_glove_data(data)

        time_keeper.sleep()

    # Close the connection if the node is shut down
    conn.close()

if __name__ == "__main__":
    rospy.init_node("glove_connection_point")

    # Get the port
    port = rospy.get_param("~port", 0)
    if port == 0:
        rospy.logfatal("Failed to get param 'port'!")
        sys.exit(1)

    # Get the communication frequency
    rate = rospy.get_param("~communication_frequency", 0)
    if rate == 0:
        rospy.logwarn("Failed to get param 'rate', using default of 10 Hz.")
        rate = 10

    # Get the timeout value
    timeout = rospy.get_param("~connection_timeout", 0)
    if timeout == 0:
        rospy.logwarn("Failed to get param 'connection_timeout', using default of 0.5s.")
        timeout = 0.5

    # Create the thread to run the ROS pub/sub
    ros_thread = threading.Thread(target=mp.ros_worker)
    ros_thread.start()

    # Get the local machine hostname
    host = socket.gethostname()
    
    # Create a socket and bind it to the host and port   
    s = socket.socket()
    s.bind((host, port))

    # Listen for incoming connections
    s.listen(1024)
    while not rospy.is_shutdown():
        # Accept an incoming connection
        conn, addr = s.accept()
        
        rospy.loginfo("Incoming connection from " + addr + "accepted!")

        # Set the timeout for the connection
        conn.settimeout(timeout)

        # Process the connection while it is open
        process_connection(conn, rate)

        rospy.loginfo("Lost connection with " + addr + ".");

    ros_thread.join()

