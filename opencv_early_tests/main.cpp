#include <iostream>
#include <opencv2/opencv.hpp>

#include "led_tracker.hpp"

int main(int argc, char **argv) {
    // Open the cv camera
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        std::cout << "Failed to open camera!\n";
        return -1;
    }
    
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);

    cv::Mat displayFrame;
    cv::Mat raw;

    LedTracker ledTracker;

    cv::Scalar led_mins[3] = {
        cv::Scalar(200, 0, 0),
        cv::Scalar(0, 120, 0),
        cv::Scalar(0, 0, 200)
    };
    cv::Scalar led_maxs[3] = {
        cv::Scalar(255, 100, 100),
        cv::Scalar(50, 255, 50),
        cv::Scalar(50, 50, 255)
    };
    cv::Scalar drawing_colours[3] = {
        cv::Scalar(30, 30, 150),
        cv::Scalar(150, 30, 30),
        cv::Scalar(30, 150, 30)
    };

    while(1) {
        cv::Mat frame;
        // Capture a frame from the camera
        cap >> frame;
        
        // Flip the frame
        cv::flip(frame, frame, 1);

        // Store raw image
        frame.copyTo(raw);

        // Draw the LEDs
        for(char i = 0; i < 1; i++) {
            ledTracker.draw_leds(frame, 
                                 displayFrame, 
                                 led_mins[i], 
                                 led_maxs[i], 
                                 drawing_colours[i]);
        }
       
        cv::imshow("Display Image", displayFrame);
        cv::imshow("Raw Data", raw);
        
        // 27 is ESCAPE
        if(cv::waitKey(30) == 27) break;
    }
    
    std::cout << "Key pressed... Closing.\n";

    return 0;
}
