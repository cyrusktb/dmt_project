#include <iostream>
#include <opencv2/opencv.hpp>

#include "led_tracker.hpp"

int main(int argc, char **argv) {
    // Open my webcam
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        std::cout << "Failed to open camera!\n";
        return -1;
    }

    cv::namedWindow("LedTracker", cv::WINDOW_AUTOSIZE);

    // HSV TEST
    cv::namedWindow("H", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("S", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("V", cv::WINDOW_AUTOSIZE);

    cv::moveWindow("LedTracker", 110, 10);
    cv::moveWindow("H", 110, 540);
    cv::moveWindow("S", 780, 10);
    cv::moveWindow("V", 780, 540);
    
    cv::Mat displayFrame;

    // HSV TEST
    cv::Mat buffer;
    cv::Mat hsv[3];

    // OpenCV uses BGR not RGB
    cv::Scalar my_red(30, 30, 150);

    while(1) {
        // Capture a frame from the camera
        cap >> displayFrame;


        // Flip so that it mirrors what I'm doing
        cv::flip(displayFrame, displayFrame, 1);

        // HSV TEST
        cv::GaussianBlur(displayFrame, buffer, cv::Size(11, 11), 0);
        cv::cvtColor(buffer, buffer, CV_BGR2HSV);
        cv::split(buffer, hsv);
        cv::imshow("H - unthresh", hsv[0]);
        //cv::imshow("S - unthresh", hsv[1]);
        //cv::imshow("V - unthresh", hsv[2]);
        
        // Circle any detected BLUE LEDs
        circle_led(displayFrame, displayFrame, LedColour::BLUE, my_red);
        // Circle any detected GREEN LEDs
        // circle_led(displayFrame, displayFrame, LedColour::GREEN, my_red);

        // Display the image
        cv::imshow("LedTracker", displayFrame);

        // Give 30ms to update the image display
        // 27 is the ESC key
        if(cv::waitKey(30) == 27) break;
    }

    std::cout << "Key pressed... Closing.\n";
    return 0;
}
