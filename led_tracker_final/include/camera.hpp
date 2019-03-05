#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <opencv2/opencv.hpp>

class Camera {
public:
    Camera(float fu, float fv, float cu, float cv,
           float k1, float k2, float p1, float p2,
           unsigned int camera_num);
    ~Camera();

    void get_image(cv::Mat &dest);
    void get_image(cv::Mat &dest, cv::Mat &raw);

    // Calculate the ray corresponding to a point on the camera's image
    cv::Vec3f get_ray(cv::Point2f p);

    bool isOpened() { return !failed_; };

private:
    // The actual camera
    cv::VideoCapture cap_;

    // Camera intrinsic matrix
    cv::Matx33f intrinsic_;

    // Camera distortion coefficients
    cv::Mat distCoeffs_;

    // Failed to open camera if true
    bool failed_;
};

#endif // __CAMERA_HPP__
