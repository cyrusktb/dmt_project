#ifndef __LED_TRACKER_HPP
#define __LED_TRACKER_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class LedTracker {
public:
    LedTracker();
    ~LedTracker();

    void draw_leds(cv::Mat &src, cv::Mat &dest, 
                   cv::Scalar bgr_min, cv::Scalar bgr_max,
                   cv::Scalar bgr_draw_colour);
protected:
    cv::Mat m_bgr[3];
    std::vector<std::vector<cv::Point>> m_contours;

    void split_and_threshold_channels(cv::Scalar min, cv::Scalar max);

    void find_contours(cv::Mat &img);
};

#endif // __LED_TRACKER_HPP
