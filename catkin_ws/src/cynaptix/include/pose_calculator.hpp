#ifndef __POSE_CALCULATOR_HPP__
#define __POSE_CALCULATOR_HPP__

#include <opencv2/opencv.hpp>

// Struct representing a triangle (of leds...)
struct Triangle {
    cv::Point3f A;
    cv::Point3f B;
    cv::Point3f C;
};

// Vector magnitude
float mag(cv::Vec3f v);

// Vector dot product
float dot(cv::Vec3f a, cv::Vec3f b);

// Vector cross product
cv::Vec3f cross(cv::Vec3f a, cv::Vec3f b);

// Calculate the angle between two vectors about a given axis
float calc_angle(cv::Vec3f axis, cv::Vec3f A, cv::Vec3f B);

// Get the rotation matrix required to rotate from new_pos to old_pos
void get_rotation_matrix(cv::Matx33f &rot, 
                         Triangle old_pos, 
                         Triangle new_pos);

#endif // __POSE_CALCULATOR_HPP__
