#ifndef __POINT_PICKER_HPP__
#define __POINT_PICKER_HPP__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cynaptix/LedRayArray.h>

#include <vector>
#include <string>

#include "led_detector/led_point.hpp"

// Class to take the points found on the screen, pick the best 3, 
// and publish the rays on the "rays" topic
class PointPicker {
public:
    PointPicker();

    void led_callback(const std::vector<LedPoint>& green_leds,
                      const std::vector<LedPoint>& blue_leds,
                      const cv::Mat& img);
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    
    ros::Publisher rviz_marker_pub_;

    std::string frame_id_;

    // How many previous sets of leds should we remember?
    int memory_;
    std::vector<std::vector<LedPoint>> prev_leds_;

    bool debug_;
};

#endif // __POINT_PICKER_HPP__
