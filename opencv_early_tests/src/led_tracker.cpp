#include "led_tracker.hpp"

LedTracker::LedTracker() {
    // ctor
}

LedTracker::~LedTracker() {
    // dtor
}

void LedTracker::draw_leds(cv::Mat &src,
                           cv::Mat &dest,
                           cv::Scalar bgr_min,
                           cv::Scalar bgr_max,
                           cv::Scalar bgr_draw_colour) {
    cv::Mat buffer;
    
    // Apply a gaussian blur to filter out noise, 
    // storing in the bufferination image
    cv::GaussianBlur(src, buffer, cv::Size(11, 11), 0);

    // Split the image into separate r g and b channels
    cv::split(buffer, m_bgr);
    
    // Apply the threshold to each channel
    split_and_threshold_channels(bgr_min, bgr_max);

    // Compare overlapping regions for each channel
    
    cv::bitwise_and(m_bgr[0], m_bgr[1], buffer);
    cv::bitwise_and(m_bgr[2], buffer, dest);

    // Find the contours of the remaining image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(buffer,
                     contours, 
                     hierarchy, 
                     CV_RETR_CCOMP, 
                     CV_CHAIN_APPROX_SIMPLE); 

    // Draw the contours on the original image
    src.copyTo(dest);
    cv::drawContours(dest, contours, -1, bgr_draw_colour, -1);
}

void LedTracker::split_and_threshold_channels(cv::Scalar min, 
                                              cv::Scalar max) {
    // Create temporary buffer image
    cv::Mat temp_img;
    // Loop through each channel
    for(char i = 0; i < 3; i++) {
        // Threshold everything above the minimum and store temporarily
        cv::threshold(m_bgr[i], 
                      temp_img,
                      min[i], 
                      255, 
                      cv::THRESH_BINARY);
        // Threshold everything below the maximum
        cv::threshold(m_bgr[i],
                      m_bgr[i],
                      max[i],
                      255,
                      cv::THRESH_BINARY_INV);
        // Compare the two images and keep the overlapping regions
        cv::bitwise_and(temp_img, m_bgr[i], m_bgr[i]);
    }
}

