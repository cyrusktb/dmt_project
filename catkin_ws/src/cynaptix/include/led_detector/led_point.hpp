#ifndef __LED_POINT_HPP__
#define __LED_POINT_HPP__

#include <opencv2/opencv.hpp>

// Helper struct to store information about an LED detected in the image
struct LedPoint {
    cv::Point center;
    float avg_radius;
    cv::Vec3f ray;
};

#endif // __LED_POINT_HPP__
