#include <iostream>
#include <opencv2/opencv.hpp>

#include "led_tracker.hpp"
#include "camera.hpp"

int main(int argc, char **argv) {
    if(argc != 5) {
        std::cout << "Usage: " 
                  << argv[0] 
                  << " <blue_1> <blue_2> <green> <target_image>"
                  << std::endl;
        return -1;
    }

    // Read in the template images
    cv::Mat templates[3];
    for(char i = 0; i < 3; i++)
        templates[i] = cv::imread(argv[i+1], 0);

    // Read in the actual image
    cv::Mat displayFrame = cv::imread(argv[4], 0);

    cv::Mat template_match_res[3];

    // Match templates
    for(char i = 0; i < 3; i++) {
        cv::Mat match;
        // Perform the template matching
        cv::matchTemplate(displayFrame, templates[i], 
                          match, CV_TM_SQDIFF);

        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(match, &minVal, &maxVal, &minLoc, &maxLoc);

        displayFrame.copyTo(template_match_res[i]);
        cv::rectangle(template_match_res[i], minLoc, 
                      cv::Point(minLoc.x + templates[i].size().width,
                                minLoc.y + templates[i].size().height), 
                      255, 2);
    }

    cv::imshow("Blue 1", templates[0]);
    cv::imshow("Blue 2", templates[1]);
    cv::imshow("Green", templates[2]);

    cv::imshow("Blue 1 res", template_match_res[0]);
    cv::imshow("Blue 2 res", template_match_res[1]);
    cv::imshow("Green res", template_match_res[2]);

    // Display the image
    cv::imshow("Raw", displayFrame);
    
    // 27 is the ESC key
    while(cv::waitKey(0) != 27);

    std::cout << "Key pressed... Closing.\n";
    return 0;
}
