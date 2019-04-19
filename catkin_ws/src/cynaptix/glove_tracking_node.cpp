#include <ros/ros.h>

#include "object_tracker.hpp"
#include "pose_calculator_3d.hpp"

int main(int argc, char **argv) {
    // Intialise ROS
    ros::init(argc, argv, "glove_tracker");

    // Get the node handle
    ros::NodeHandle nh;

    // Create the object trackers
    std::string left_object_channel, right_object_channel;
    if(!nh.param<std::string>("left_object_channel", 
                              left_object_channel,
                              "/left_camera/objects")) {
        ROS_WARN("Failed to get param 'left_object_channel'. Defaulting to '/left_camera/objects'");
    }
    if(!nh.param<std::string>("right_object_channel", 
                              right_object_channel,
                              "/right_camera/objects")) {
        ROS_WARN("Failed to get param 'right_object_channel'. Defaulting to '/right_camera/objects'");
    }
    
    ObjectTracker left_tracker(&nh, left_object_channel);
    ObjectTracker right_tracker(&nh, right_object_channel);

    std::vector<Object> left_objects;
    std::vector<Object> right_objects;

    PoseCalculator3d pose_calculator(nh);

    // Publishing frequency
    float freq;
    if(!nh.param<float>("publishing_frequency", freq, 10)) {
        ROS_INFO("Defaulting to publishing frequency of 10 Hz");
    }
    
    ros::Rate sleeper(freq);

    while(ros::ok()) {
        // Get the most recent set of objects
        left_objects = left_tracker.get_most_recent_objects();
        right_objects = right_tracker.get_most_recent_objects();

        // Compare objects
        for(int l = 0; l < left_objects.size(); l++) {
            const Object& lobj = left_objects[l];
            ROS_ERROR("Left ID: %d", lobj.id);
            for(int r = 0; r < right_objects.size(); r++) {
                const Object& robj = right_objects[r];
                ROS_ERROR("Right ID: %d", robj.id);
                if(robj.id == lobj.id) {
                    // TODO: Calculate position of corners in 3d space
                    pose_calculator.add_object_pair(lobj, robj);
                }
            }
        }
        pose_calculator.calculate_and_publish_pose();

        // Handle events
        ros::spinOnce();

        // Maintain update frequency
        sleeper.sleep();
    }
}
