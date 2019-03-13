#ifndef __LED_TRACKER_3D_HPP__
#define __LED_TRACKER_3D_HPP__

#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>

#include "led_tracker_2d.hpp"

// LED positions and magnitudes

#define GREEN_POS cv::Point3f(0.007, 0.033, 0)
#define BLUE_L_POS cv::Point3f(0.038, 0, 0)
#define BLUE_R_POS cv::Point3f(0, 0, 0)

#define GBL_LEN mag(GREEN_POS - BLUE_L_POS)
#define GBR_LEN mag(GREEN_POS - BLUE_R_POS)
#define BLBR_LEN mag(BLUE_L_POS - BLUE_R_POS)


// Maximum distance for an intersection to be considered an intersection

#define INTERSECTION_TOLERANCE 5


// Camera callibration data for initialising cameras
// For now the same callibration numbers seem to work for both cameras
// Left camera is the first camera, right camera is the second

#define LEFT_CAMERA_PARAMS   722.106766, 723.389819, \
                             344.447625, 271.702332, \
                             -0.430658, 0.235174, \
                             0.000098, -0.000494 , \
                             0

#define RIGHT_CAMERA_PARAMS  722.106766, 723.389819, \
                             344.447625, 271.702332, \
                             -0.430658, 0.235174, \
                             0.000098, -0.000494 , \
                             1


// Rotation and translation of cameras in the world space

#define LEFT_CAMERA_ROT cv::Matx33f(cos(-0.0872665), -sin(-0.0872665), 0, \
                                    sin(-0.0872665), cos(-0.0872665) , 0, \
                                    0              , 0               , 1) 

#define RIGHT_CAMERA_ROT cv::Matx33f(cos(0.0872665) , -sin(0.0872665) , 0, \
                                     sin(0.0872665) , cos(0.0872665)  , 0, \
                                     0              , 0               , 1)

#define LEFT_CAMERA_POS cv::Vec3f(-0.1, 0.01763, 0)

#define RIGHT_CAMERA_POS cv::Vec3f(0.1, 0.01763, 0)

class LedTracker3D {
public:
    LedTracker3D();
    ~LedTracker3D();

    // Get the current left and right images from the camera
    // Useful for visual debugging
    void get_imgs(cv::Mat &left, cv::Mat &right);

    // Get the current 3D position of the glove
    void get_glove_pos(cv::Point3f &pos, cv::Point3f &ypr);
private:
    // Main threaded processing loop
    void loop();

    // Finds the best potential led combination to fit the physical 
    // position constraints and thus calculate the glove position
    void find_glove_pos();

    // Find the *center* of the closest points of two rays, and the
    // *distance* between the rays at this point
    void intersect_rays(cv::Vec3f left,
                        cv::Vec3f right,
                        cv::Point3f *center,
                        float *distance);

    // Find the error in shape given three points
    float calculate_error(cv::Point3f green, 
                          cv::Point3f blue_l, 
                          cv::Point3f blue_r);
    
    // Find potential 3d points given the green and blue leds
    void find_pot_3d_points(std::vector<cv::Point3f> *green,
                            std::vector<std::array<int, 2>> *green_its,
                            std::vector<cv::Point3f> *blue,
                            std::vector<std::array<int, 2>> *blue_its); 

    // Most recent images
    cv::Mat imgs_[2];

    // Calculated roll pitch and yaw and translation
    cv::Point3f glove_ypr_;
    cv::Point3f glove_pos_;

    // Vector of potential LEDs
    std::vector<PotentialLed> pot_leds_[2];

    // 2D LED trackers
    LedTracker2D trackers_[2];

    // Main loop thread
    std::thread thread_;

    // Thread running flag
    bool running_;

    // Image mutex
    std::mutex img_mutex_;

    // Potential LED mutex
    std::mutex pot_mutex_;
};

#endif // __LED_TRACKER_3D_HPP__
