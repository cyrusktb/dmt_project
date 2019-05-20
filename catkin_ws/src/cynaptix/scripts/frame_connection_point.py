#!/usr/bin/env python

import rospy

import socket
import sys
import threading

import frame_connection.message_processor as mp

def process_connection(conn, rate):
    # Double the frequency as we'll call the sleep function twice
    time_keeper = rospy.Rate(2*rate)

    data = ""

    while not rospy.is_shutdown():
        # Send data to the frame
        msg = mp.get_frame_message()
        try:
            conn.send(msg)
        except socket.timeout:
            rospy.loginfo("Connection timed out.")
            return

        time_keeper.sleep()

        # Receive data from the frame
        try:
            data = conn.recv(64)
        except:
            rospy.loginfo("Connection timed out.")
            return

        mp.process_frame_data(data)

        time_keeper.sleep()

    # Close the connection if the node is shut down
    conn.close()


if __name__ == "__main__":
    rospy.init_node("frame_connection_point")

    # Get the port
    port = rospy.get_param("~port", 0)
    if port == 0:
        rospy.logfatal("Failed to get param 'port'!")
        sys.exit(1)

    # Get the communication frequency
    rate = rospy.get_param("~communication_frequency", 0)
    if rate == 0:
        rospy.logwarn("Failed to get param 'communication_frequency', using default of 10 Hz.")
        rate = 10

    # Get the timeout value
    timeout = rospy.get_param("~connection_timeout", 0)
    if timeout == 0:
        rospy.logwarn("Failed to get param 'connection_timeout', using default of 0.5s.")
        timeout = 0.5

    # Get the hostname
    host = rospy.get_param("~hostname", 0)
    if host == 0:
        # Get the local machine hostname
        host = socket.gethostname()
        rospy.logwarn("Failed to get param 'hostname', using default of '%s'.", host)

    # Create the thread to run the ROS pub/sub
    ros_thread = threading.Thread(target=mp.ros_worker)
    ros_thread.start()

    rospy.loginfo("Starting frame server on '%s:%d'", host, port)

    # Create a socket and bind it to the host and port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))

    # Listen for incoming connections
    s.listen(1024)
    while not rospy.is_shutdown():
        conn, addr = s.accept()
        rospy.loginfo("Incoming connection from '" + addr[0] + "' accepteted!")

        # Set the timeout for the connection
        conn.settimeout(timeout)

        # Process the connection while it is open
        process_connection(conn, rate)

        rospy.loginfo("Lost connection with '" + addr[0] + "'.")

    ros_thread.join()
    s.close()
