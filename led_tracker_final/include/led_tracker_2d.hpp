#ifndef __LED_TRACKER_HPP__
#define __LED_TRACKER_HPP__

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


// How many previous positions should we store when considering previous
// led positions?

#define LED_POS_MEMORY 10


// Camera callibration data for initialising cameras
// For now the same callibration numbers seem to work for both cameras
// Left camera is the first camera, right camera is the second

#define LEFT_CAMERA_PARAMS 722.106766, 723.389819, \
                           344.447625, 271.702332, \
                           -0.430658, 0.235174, \
                           0.000098, -0.000494, \
                           1

#define RIGHT_CAMERA_PARAMS 722.106766, 723.389819, \
                            344.447625, 271.702332, \
                            -0.430658, 0.235174, \
                            0.000098, -0.000494, \
                            2

// Rotation and translation of cameras in the world space

#define LEFT_CAMERA_ROT cv::Matx33f(cos(-0.0872665), -sin(-0.0872665), 0, \
                                    sin(-0.0872665), cos(-0.0872665) , 0, \
                                    0              , 0               , 1) 

#define RIGHT_CAMERA_ROT cv::Matx33f(cos(0.0872665) , -sin(0.0872665) , 0, \
                                     sin(0.0872665) , cos(0.0872665)  , 0, \
                                     0              , 0               , 1)

#define LEFT_CAMERA_POS cv::Vec3f(-50, 0, 0)

#define RIGHT_CAMERA_POS cv::Vec3f(50, 0, 0)


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

// Enum to represent the three available LEDs
// When GREEN_LED is at the top, BLUE_LED_L is on the left and BLUE_LED_R
// is on the right
enum LedNum {
    GREEN_LED,
    BLUE_LED_L,
    BLUE_LED_R
};

// Available colours to look for
enum LedColour {
    BLUE,
    GREEN
};

// Struct to store position and size of LED illumination on a camera
struct LedPos {
    cv::Point center;
    float average_radius;
};

class LedTracker {
public:
    LedTracker();
    ~LedTracker();

    // Get the 3d position of an led
    cv::Vec3f get_led_pos(LedNum led);

    // Get images - useful for debugging
    void get_images(cv::Mat &left_dest, cv::Mat &right_dest) {
        // Threading lock
        std::lock_guard<std::mutex> lock(image_mutex_);
        
        // Copy the images
        img_[0].copyTo(left_dest);
        img_[1].copyTo(right_dest);
    }
private:
    // Running member variable
    bool running_;

    // Left and right cameras
    Camera camera_[2];

    // Positions and rotations of the cameras in world space (mm)
    // Rotate about center of camera (0,0,0) and then translate to pos
    cv::Point3f camera_pos_[2];
    cv::Matx33f camera_rot_[2];
    
    // Current left and right images
    cv::Mat img_[2];

    // Image left and right hsv channels 
    // 0 & 1 are thresholded green and blue H channels,
    // 2 & 3 are thresholded S and V channels
    cv::Mat hsv_[2][4];

    // Estimated positions in 3d space of each led
    // Previous positions are stored up to LED_POS_MEMORY
    cv::Vec3f led_pos_[3][LED_POS_MEMORY];

    // Flag to say whether position estimation has occurred
    bool position_estimated_;

    // Current contours found on the left and right images based 
    // on saturation and value thresholding
    std::vector<std::vector<cv::Point>> contours_[2];

    // Weighting for left image and right image contours based on hue
    // First weight is the green colour likelihood
    // Second weight is the blue colour likelihood
    std::vector<std::pair<float, float>> hue_weighting_[2];

    // Centerpoints and average radii of the contours which 
    // have non-zero hue weighting 
    std::vector<LedPos> potential_leds_[2];

    // Weighting for contour pairs based on position
    // 0. float: weight relative to GREEN position
    // 1. float: weight relative to BLUE_L position
    // 2. float: weight relative to BLUE_R position
    // 3. int: iterator of corresponding left potential_led_
    // 4. int: iterator of corresponding right potential_led_
    std::vector<std::tuple<float, float, float, int, int>> pair_weighting_;
    
    // Main loop thread
    std::thread thread_;

    // Mutex for thread-safe image usage
    std::mutex image_mutex_;

    // Main updating loop
    void loop();

    // Get new images from the cameras
    void update_images();

    // Updates the stores contour vector with new data from the new image
    void update_contours();

    // Updates the hue-based contour weighting vector
    void update_hue_weighting();

    // Updates the position-based contour weighting vector
    void update_position_weighting();

    // Pick the 3 most likely positions of the leds
    void calculate_LED_positions();

    // Finds the average radius and centroid of a single *contour*
    LedPos find_led_pos(std::vector<cv::Point> contour);

    // Takes an image (in BGR) *src* and splits it into 4 HSV channels.
    // Binary thresholds each channel and outputs to the 4 images *dest*
    // The 1st image is green, the 2nd is blue, 3rd and 4th are sat and val
    void split_and_threshold_channels(cv::Mat *src, 
                                      cv::Mat *dest);

    // Find the *center* of the closest points of two rays, and the
    // *distance* between the rays at this point
    void intersect_rays(cv::Vec3f left, 
                        cv::Vec3f right, 
                        cv::Point3f *center,
                        float *distance);

    // Used to access led-based arrays of 3
    const char GREEN = 0;
    const char BLUE_L = 1;
    const char BLUE_R = 2;

    const cv::Vec3f green_pos;
    const cv::Vec3f blue_l_pos;
    const cv::Vec3f blue_r_pos;
};

#endif // __LED_TRACKER_HPP__
