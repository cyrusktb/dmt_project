#ifndef __MATHS_HPP__
#define __MATHS_HPP__

#include <opencv2/opencv.hpp>

#include <cmath>

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

// Struct representing a quaternion
struct Quaterniond {
    double x, y, z, w;
};

Quaterniond operator * (const Quaterniond& q1, const Quaterniond& q2);
cv::Vec3f operator * (const Quaterniond& q, const cv::Vec3f& v);
cv::Point3f operator * (const Quaterniond& q, const cv::Point3f& p);

// Get the quaternion to rotate v2 onto v1 about axis
Quaterniond get_quaternion_about_axis(cv::Vec3f axis,
                                      cv::Vec3f v1,
                                      cv::Vec3f v2);

// Get the quaternion needed to rotate from old_pos to new_pos
Quaterniond get_quaternion(Triangle& old_pos, Triangle& new_pos);

#endif // __MATHS_HPP__
