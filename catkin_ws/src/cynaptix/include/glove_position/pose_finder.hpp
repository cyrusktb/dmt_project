#ifndef __POSE_FINDER_HPP__
#define __POSE_FINDER_HPP__

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>


#include <opencv2/opencv.hpp>

#include <vector>

#include "glove_position/maths.hpp"

// Helper struct to storea point and its two rays
struct LedPoint {
    cv::Point3f point;
    int ray_id_1;
    int ray_id_2;
    bool shares_a_ray(LedPoint& other) {
        if(other.ray_id_1 == ray_id_1 || other.ray_id_2 == ray_id_1 ||
           other.ray_id_1 == ray_id_2 || other.ray_id_2 == ray_id_2)
            return true;
        return false;
    }
};

// This class assumes that the LEDs are numbered starting from the green
// LED in a clockwise direction
class PoseFinder {
public:
    PoseFinder();
    virtual ~PoseFinder();

    void find_pose(std::vector<LedPoint>& green_points,
                   std::vector<LedPoint>& blue_points);
private:
    // Find the error in a given combination of points
    // Error indicates how close to the correct shape this combination is
    float find_error(cv::Point3f green, 
                     cv::Point3f blue_1, 
                     cv::Point3f blue_2);

    // Find the pose of the glove and publish on TF
    void find_glove_pose(cv::Point3f green,
                         cv::Point3f blue_1,
                         cv::Point3f blue_2);
 
    ros::NodeHandle nh_;

    // TF broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Positions of the LEDs relative to the origin
    Triangle led_pos_;

    // Distances between LEDs
    float gb1_len_, gb2_len_, bb_len_;

    // Adjustments for linear errors in position estimates
    float x_grad_, x_intercept_,
          y_grad_, y_intercept_,
          z_grad_, z_intercept_,
          yaw_grad_, yaw_intercept_;
};

#endif // __POSE_FINDER_HPP__
