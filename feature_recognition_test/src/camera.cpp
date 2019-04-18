#include "camera.hpp"

Camera::Camera(int height, int width, 
               float fu, float fv, 
               float cu, float cv, 
               float k1, float k2, 
               float p1, float p2,
               unsigned int camera_num) 
                                    : height_(height), width_(width), 
                                      fu_(fu), fv_(fv), cu_(cu), cv_(cv), 
                                      k1_(k1), k2_(k2), p1_(p1), p2_(p2) {
    // Create the distortion matrix
    distCoeffs_ = cv::Mat(5, 1, CV_32FC1);
    distCoeffs_.at<float>(0,0) = k1;
    distCoeffs_.at<float>(1,0) = k2;
    distCoeffs_.at<float>(2,0) = p1;
    distCoeffs_.at<float>(3,0) = p2;
    distCoeffs_.at<float>(4,0) = 0;
    
    // Create the intrinsic matrix
    intrinsic_ = cv::Mat(3, 3, CV_32FC1);
    intrinsic_.at<float>(0,0) = fu;
    intrinsic_.at<float>(0,1) = 0;
    intrinsic_.at<float>(0,2) = cu;
    intrinsic_.at<float>(1,0) = 0;
    intrinsic_.at<float>(1,1) = fv;
    intrinsic_.at<float>(1,2) = cv;
    intrinsic_.at<float>(2,0) = 0;
    intrinsic_.at<float>(2,1) = 0;
    intrinsic_.at<float>(2,2) = 1;

    cap_.open(camera_num);
    if(!cap_.isOpened()) 
        failed_ = true;
    else
        failed_ = false;
}

Camera::~Camera() {
    // dtor
}

void Camera::get_image(cv::Mat &dest) {
    cv::Mat raw;

    // Capture the frame
    cap_ >> raw;
    
    // Undistort the image
    cv::undistort(raw, dest, intrinsic_, distCoeffs_);

    // Flip to mirror user (for debugging)
    cv::flip(dest, dest, 1);
}

void Camera::get_image(cv::Mat &dest, cv::Mat &raw) {
    // Capture the frame
    cap_ >> raw;
    
    // Undistort the image
    cv::undistort(raw, dest, intrinsic_, distCoeffs_);

    // Flip to mirror user (for debugging)
    cv::flip(dest, dest, 1);
}

