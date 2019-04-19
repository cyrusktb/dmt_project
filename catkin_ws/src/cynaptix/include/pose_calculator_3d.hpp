#ifndef __POSE_CALCULATOR_3D_HPP__
#define __POSE_CALCULATOR_3D_HPP__

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include "object_tracker.hpp"
#include "pose_calculator.hpp"

class PoseCalculator3d {
public:
    PoseCalculator3d(const ros::NodeHandle& nh);
    ~PoseCalculator3d();

    // Uses data given previously to calculate the pose and publish on tf
    void calculate_and_publish_pose();
    
    // Give a pair of matching objects
    void add_object_pair(Object left, Object right);
private:
    // Camera intrinsic matrices
    cv::Matx33f left_intrinsic_;
    cv::Matx33f right_intrinsic_;

    // Lists of paired objects
    std::vector<Object> left_objects_;
    std::vector<Object> right_objects_;

    // tf2 publisher and message and listener
    tf2_ros::TransformBroadcaster tf_br_;
    geometry_msgs::TransformStamped transform_stamped_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    // Camera poses
    geometry_msgs::TransformStamped left_cam_tf_;
    geometry_msgs::TransformStamped right_cam_tf_;

    // Get the ray from a camera to a point
    cv::Vec3f get_ray(cv::Point2f p, bool is_left_camera);

    // Intersect two rays
    void intersect_rays(cv::Vec3f left,
                        cv::Vec3f right,
                        cv::Point3f *center,
                        float *distance);

    // Subscribers to listen for CameraInfo
    image_transport::ImageTransport *image_transport_;
    image_transport::CameraSubscriber left_cam_sub_;
    image_transport::CameraSubscriber right_cam_sub_;

    // Message callbacks
    void left_camera_info_callback(
        const sensor_msgs::ImageConstPtr& img, 
        const sensor_msgs::CameraInfoConstPtr& msg
    );
    void right_camera_info_callback(
        const sensor_msgs::ImageConstPtr& img, 
        const sensor_msgs::CameraInfoConstPtr& msg
    );
};

#endif // __POSE_CALCULATOR_3D_HPP__
