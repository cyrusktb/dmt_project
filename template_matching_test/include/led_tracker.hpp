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

#define BLUE_SAT_MAX 25
#define BLUE_SAT_MIN 0
#define GREEN_SAT_MAX 25
#define GREEN_SAT_MIN 0

// Value ranges from 0 to 255

#define BLUE_VAL_MAX 255
#define BLUE_VAL_MIN 245
#define GREEN_VAL_MAX 255
#define GREEN_VAL_MIN 245

// When finding the colour of an LED, what is the largest search multiplier?
#define CIRCLE_MULT_MAX 4.0f
#define CIRCLE_MULT_MIN 2.0f
#define CIRCLE_MULT_STEP 0.1f


#include <opencv2/opencv.hpp>
#include <vector>
#include <math.h>

// Available colours to look for
enum LedColour {
    BLUE,
    GREEN
};

// Struct to store position and size of LED illumination
struct LedPos {
    cv::Point center;
    float average_radius;
};


// Circles all LEDs of colour *led_colour* in *src_img* and outputs 
// the cicles on *dest_img* in the colour *bgr_draw_colour*
void circle_led(cv::Mat &src_img,
                cv::Mat &dest_img, 
                LedColour led_colour, 
                cv::Scalar bgr_draw_colour);

// Locates all LEDs of colour *led_colour* in the image *src_img*
std::vector<LedPos> find_leds(cv::Mat &src_img, LedColour led_colour);

// Finds the center and average radius of a single *contour*, 
// used by *find_leds*
LedPos find_led_pos(std::vector<cv::Point> contour);

// Gets the corresponding *min* and *max* threshold values for a 
// given *led_colour*
void get_colour_threshold_min_max(LedColour led_colour, 
                                  cv::Scalar &min,
                                  cv::Scalar &max);

// Takes an image (in BGR) *src* and splits it into 3 HSV channels.
// Binary thresholds each channel according to *min* and *max* 
// values and outputs to the 3 images *dest*
void split_and_threshold_channels(cv::Mat &src, 
                                  cv::Mat *dest,
                                  cv::Scalar min,
                                  cv::Scalar max);

#endif // __LED_TRACKER_HPP__
