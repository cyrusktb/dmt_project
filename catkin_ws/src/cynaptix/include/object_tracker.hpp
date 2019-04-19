#ifndef __OBJECT_TRACKER_HPP__
#define __OBJECT_TRACKER_HPP__

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <QTransform>
#include <chrono>
#include <string>
#include <map>

// A storage struct for information about objects detected in the image
struct Object {
    int id;
    float width;
    float height;
    QTransform qtHomography;
    QPointF top_left;
    QPointF top_right;
    QPointF bot_left;
    QPointF bot_right;
    std::chrono::time_point<std::chrono::system_clock> time_last_detected; 
};

// Class which stores which objects have been detected
class ObjectTracker {
public:
    ObjectTracker(ros::NodeHandle *nh, std::string object_channel);
    virtual ~ObjectTracker();

    // Get the most recent object received by the node
    std::vector<Object> get_most_recent_objects();
private:
    // Callback for when an object message arrives
    void objects_detected_callback(
        const std_msgs::Float32MultiArray::ConstPtr& msg
    );

    // Map to store all the objects
    std::map<int, Object> objects_;
    // Vector to store all the object ids
    std::vector<int> object_ids_;

    // The total number of objects available
    int object_count_;
};

#endif // __OBJECT_TRACKER_HPP__
