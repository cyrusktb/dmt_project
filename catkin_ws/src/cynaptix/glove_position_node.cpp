#include <ros/ros.h>

#include "glove_position/ray_intersector.hpp"

int main(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "glove_tracker");

    // Create the ray intersector
    RayIntersector intersector;

    // Wait for message updates
    ros::spin();
}
