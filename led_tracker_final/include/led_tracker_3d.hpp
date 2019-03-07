#ifndef __LED_TRACKER_3D_HPP__
#define __LED_TRACKER_3D_HPP__

#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>

#include "led_tracker_2d.hpp"

class LedTracker3D {
public:
    LedTracker3D();
    ~LedTracker3D();

    // Get the current left and right images from the camera
    // Useful for visual debugging
    void get_imgs(cv::Mat &left, cv::Mat &right);

    // Get the current 3D position of the glove
    void get_glove_pos(cv::Point3f &pos, cv::Point3f &rot);
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

    // Most recent images
    cv::Mat imgs_[2];

    // Vector of potential LEDs
    std::vector<PotentialLed> pot_leds_[2];

    // 2D LED trackers
    LedTracker2D[2] trackers_;

    // Main loop thread
    std::thread loop_thread_;

    // Thread running flag
    bool running_;

    // Image mutex
    std::mutex img_mutex_;

    // Potential LED mutex
    std::mutex pot_mutex_;
};

#endif // __LED_TRACKER_3D_HPP__
