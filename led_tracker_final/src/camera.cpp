#include "camera.hpp"

Camera::Camera(float fu, float fv, 
               float cu, float cv, 
               float k1, float k2, 
               float p1, float p2,
               unsigned int camera_num) {
    // Create the distortion matrix
    distCoeffs_ = cv::Mat(5, 1, CV_32FC1);
    distCoeffs_.at<float>(0,0) = k1;
    distCoeffs_.at<float>(1,0) = k2;
    distCoeffs_.at<float>(2,0) = p1;
    distCoeffs_.at<float>(3,0) = p2;
    distCoeffs_.at<float>(4,0) = 0;
    
    // Create the intrinsic matrix
    intrinsic_(0,0) = fu;
    intrinsic_(0,1) = 0;
    intrinsic_(0,2) = cu;
    intrinsic_(1,0) = 0;
    intrinsic_(1,1) = fv;
    intrinsic_(1,2) = cv;
    intrinsic_(2,0) = 0;
    intrinsic_(2,1) = 0;
    intrinsic_(2,2) = 1;

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

cv::Vec3f Camera::get_ray(cv::Point2f p) {
    // Convert to homogenous
    cv::Matx31f hom_p(p.x, p.y, 1);

    // Multiply by the inverse of the intrinsic matrix
    hom_p = intrinsic_.inv() * hom_p;

    // Normalise
    cv::Vec3f dir(hom_p(0), hom_p(1), hom_p(2));
    dir /= cv::norm(dir);

    return dir;
}
