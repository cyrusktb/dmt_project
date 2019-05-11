#ifndef __LED_DETECTOR_HPP__
#define __LED_DETECTOR_HPP__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <iostream>

#include "led_detector/led_point.hpp"

// A class to detect all points which look like LEDs in an image
// It doesn't care how many it detects
class LedDetector {
public:
    LedDetector(const std::function<void (
                    const std::vector<LedPoint>& green_leds,
                    const std::vector<LedPoint>& blue_leds,
                    const cv::Mat& img
                )>& callback);
private:
    // To get public topics and parameters
    ros::NodeHandle pub_nh_;
    // To get private topics and parameters
    ros::NodeHandle nh_;

    cv::Matx33f camera_intrinsic;

    // Callback for when leds have been found on an image
    std::function<void (
        const std::vector<LedPoint>& green_leds,
        const std::vector<LedPoint>& blue_leds,
        const cv::Mat& img
    )> leds_found_callback;
    
    // To get images from a ros topic
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;

    // Thresholding values - { min, max }
    int green_hue_[2];
    int blue_hue_[2];
    int sat_[2];
    int val_[2];
    int min_radius_;
    int max_radius_;

    // Callback for new image arrivals
    void image_callback(const sensor_msgs::ImageConstPtr& msg,
                        const sensor_msgs::CameraInfoConstPtr& info);

    // Find the position of a potential led given a contour
    LedPoint find_led_pos(std::vector<cv::Point> &contour);

    // Split and threshold channels from the image into hsv
    // Note that the h channel is duplicated at the start
    void split_and_threshold_channels(cv::Mat &img, cv::Mat *hhsv);

    // Find contours in the image
    void find_contours(std::vector<std::vector<cv::Point>> &g_contours,
                       std::vector<std::vector<cv::Point>> &b_contours,
                       cv::Mat *hhsv);

    // Calculate a normalised ray out of the camera towards the led point
    void calculate_ray(LedPoint& point, cv::Matx33f& inv_intrinsic);
};

#endif // __LED_DETECTOR_HPP__
