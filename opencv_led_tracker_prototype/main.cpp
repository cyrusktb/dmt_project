#include <iostream>
#include <opencv2/opencv.hpp>

#include "led_tracker.hpp"
#include "camera.hpp"

int main(int argc, char **argv) {
    // Create camera class instance
    float params[10] = {
        480, 640,
        722.106766, 723.389819,
        344.447625, 271.702332,
        -0.430658, 0.235174,
        0.000098, -0.000494
    };
    Camera cam1(params[0], params[1], params[2], params[3], params[4],
                params[5], params[6], params[7], params[8], params[9], 1);
    // If the camera failed to open, then return
    if(!cam1.isOpened()) {
        std::cout << "Failed to open Camera 1" << std::endl;
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
    
    cv::Mat undistort_test;

    while(1) {
        // Get image from camera
        cam1.get_image(undistort_test, displayFrame);

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
        cv::imshow("Undistorted", undistort_test);

        // Give 30ms to update the image display
        // 27 is the ESC key
        if(cv::waitKey(30) == 27) break;
    }

    std::cout << "Key pressed... Closing.\n";
    return 0;
}
