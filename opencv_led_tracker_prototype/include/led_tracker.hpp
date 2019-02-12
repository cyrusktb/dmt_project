#ifndef __LED_TRACKER_HPP
#define __LED_TRACKER_HPP

// OpenCV hue ranges from 0 to 179

// HSV blue is 120
#define BLUE_HUE_MAX 125
#define BLUE_HUE_MIN 105
// HSV green is 60
#define GREEN_HUE_MAX 75
#define GREEN_HUE_MIN 50

// Saturation ranges from 0 to 255

#define BLUE_SAT_MAX 40
#define BLUE_SAT_MIN 0
#define GREEN_SAT_MAX 40
#define GREEN_SAT_MIN 0

// Value ranges from 0 to 255

#define BLUE_VAL_MAX 255
#define BLUE_VAL_MIN 240
#define GREEN_VAL_MAX 255
#define GREEN_VAL_MIN 240



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
    unsigned int average_radius;
};


// Circles all LEDs of colour *led_colour* in *src_img* and outputs 
// the cicles on *dest_img* in the colour *bgr_draw_colour*
void circle_led(cv::Mat &src_img,
                cv::Mat &dest_img, 
                LedColour led_colour, 
                cv::Scalar bgr_draw_colour);

// Locates all LEDs of colour *led_colour* in the image *src_img*
std::vector<LedPos> find_leds(cv::Mat &src_img, LedColour led_colour);

// Locates all LEDs of colour *led_colour* in the image *src_img*
// Second method attempt
std::vector<LedPos> find_leds2(cv::Mat &src_img, LedColour led_colour);

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

#endif // __LED_TRACKER_HPP
