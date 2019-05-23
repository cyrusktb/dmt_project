#ifndef __RAY_INTERSECTOR_HPP__
#define __RAY_INTERSECTOR_HPP__

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <cynaptix/LedRayArray.h>

#include <opencv2/opencv.hpp>

#include <vector>

#include "glove_position/maths.hpp"
#include "glove_position/pose_finder.hpp"

class RayIntersector {
public:
    RayIntersector();
    virtual ~RayIntersector();

private:
    // Message callbacks
    void left_callback(const cynaptix::LedRayArray& msg);
    void right_callback(const cynaptix::LedRayArray& msg);

    // Find the best combination of ray intersections to match 
    // the known shape of the LEDs
    void intersect_rays();

    // Intersect a single pair of rays
    bool intersect_single_rays(geometry_msgs::Vector3 left,
                               geometry_msgs::Vector3 right,
                               LedPoint *point);

    // Publish an array of points for debugging
    void debug_publish_points(std::vector<LedPoint> g,
                              std::vector<LedPoint> b);

    // Pose Finder to find the pose
    PoseFinder finder_;

    // Node handle
    ros::NodeHandle nh_;

    // Standard subscribers
    ros::Subscriber left_sub_;
    ros::Subscriber right_sub_;

    // TF subscriber
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Camera transforms
    geometry_msgs::TransformStamped left_cam_tf_;
    geometry_msgs::TransformStamped right_cam_tf_;

    // Arrays of most recently received led vectors
    cynaptix::LedRayArray left_rays_;
    cynaptix::LedRayArray right_rays_;

    // Maximum amount of time between message arrivals for them 
    // to be accepted as coming from the same image
    float message_delay_;

    // Boundary region outside of which the glove will be ignored
    float min_x_, max_x_,
          min_y_, max_y_,
          min_z_, max_z_;

    // Maximum accepted distance for "intersection" of rays
    float intersection_tol_;
    
    // Debug mode
    bool debug_;
};

#endif // __RAY_INTERSECTOR_HPP__
