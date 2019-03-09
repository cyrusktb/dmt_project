#ifndef __LED_TRACKER_2D_HPP__
#define __LED_TRACKER_2D_HPP__

// OpenCV hue ranges from 0 to 179

// HSV blue is 120
#define BLUE_HUE_MAX 125
#define BLUE_HUE_MIN 112
// HSV green is 60
#define GREEN_HUE_MAX 72
#define GREEN_HUE_MIN 53


// Saturation ranges from 0 to 255

#define SAT_MAX 25
#define SAT_MIN 0


// Value ranges from 0 to 255

#define VAL_MAX 255
#define VAL_MIN 245


// When finding the colour of an LED, what is the largest search multiplier?

#define CIRCLE_MULT_MAX 4.0f
#define CIRCLE_MULT_MIN 2.0f
#define CIRCLE_MULT_STEP 0.1f


// Minimum weighting for hue based weighting, below this is '0'

#define MIN_HUE_WEIGHT 0.05

// Minimum weighting for position based weighting, below this is '0'

#define MIN_POS_WEIGHT 0.2

// Minimum total weighting, below this is '0'

#define MIN_TOTAL_WEIGHT 0.05

#include <opencv2/opencv.hpp>
#include <vector>
#include <thread>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <cmath>

#include "camera.hpp"

// Potential LED struct represents a highlighted contour for consideration
struct PotentialLed {
    std::vector<cv::Point> contour;
    cv::Point center;
    float avg_radius;
    float hue_weight[3];
    float pos_weight[3];
    float total_weight[3];
    cv::Vec3f ray;
};

class LedTracker2D {
public:
    LedTracker2D(float fu, float fv, float cu, float cv,
           float k1, float k2, float p1, float p2,
           unsigned int camera_num);
    ~LedTracker2D();

    // Get the potential leds for the current image
    std::vector<PotentialLed> get_points();
    
    // Get the most recent image
    cv::Mat get_img();

    // Say which leds were chosen to be kept
    void set_prev_leds(std::vector<PotentialLed> prev_leds);

    // Rotation and position matrices of this tracker
    cv::Matx33f rot;
    cv::Point3f pos;

private:
    // Weight the hue value of all the found contours
    void weigh_hue();

    // Weight the position of all the found contours
    // Returns false if there's not yet enough data
    bool weigh_pos();

    // Combine the position and hue weightings
    void combine_weights();

    // Find contours in the image
    void find_contours();

    // Choose which contours are potentially actually LEDs
    void choose_leds();

    // Update the rays of the potential LEDs
    void update_rays();

    // Find the position of a potential led given its contour
    void find_led_pos(PotentialLed *pl);

    // Split and threshold channels from the image into hsv
    void split_and_threshold_channels();

    // Camera and accompanying image
    Camera cam_;
    cv::Mat img_;

    // HSV channels - note that the H channel is duplicated at the start
    cv::Mat hsv_[4];

    // Potential LEDs
    std::vector<PotentialLed> potential_leds_;

    // Previous led positions
    cv::Point prev_led_pos_[3];
};

#endif // __LED_TRACKER_2D_HPP__
