#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <opencv2/opencv.hpp>

class Camera {
public:
    Camera(int height, int width, float fu, float fv, float cu, float cv,
                                  float k1, float k2, float p1, float p2,
                                  unsigned int camera_num);
    ~Camera();

    void get_image(cv::Mat &dest);
    void get_image(cv::Mat &dest, cv::Mat &raw);

    bool isOpened() { return !failed_; };

private:
    // The actual camera
    cv::VideoCapture cap_;

    // Camera intrinsic matrix
    cv::Mat intrinsic_;

    // Camera distortion coefficients
    cv::Mat distCoeffs_;

    // Image size
    int height_, width_;

    // Focal Lengths
    float fu_, fv_;

    // Image center
    float cu_, cv_;

    // Camera distortion parameters
    float k1_, k2_, p1_, p2_;

    // Failed to open camera if true
    bool failed_;
};

#endif // __CAMERA_HPP__
