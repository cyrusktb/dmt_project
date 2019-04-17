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
    for(char i = 0; i < 3; i++) {
        templates[i] = cv::imread(argv[i+1], 0);
        cv::resize(templates[i], templates[i], cv::Size(0,0), 0.5, 0.5);
    }

    // Read in the actual image
    cv::Mat displayFrame = cv::imread(argv[4], 0);

    cv::Mat template_match_res[3];
    cv::Mat test[3];
    for(char i = 0; i < 3; i++) {
        template_match_res[i].create(
            displayFrame.cols - templates[i].cols + 1, 
            displayFrame.rows - templates[i].rows + 1, 
            CV_32FC1
        );
        template_match_res[i].copyTo(test[i]);
    }

    cv::Mat displayBoxed[3];

    // Match templates
    for(char i = 0; i < 3; i++) {
        // Perform the template matching
        cv::matchTemplate(displayFrame, templates[i], 
                          template_match_res[i], CV_TM_CCOEFF_NORMED);
        cv::normalize(template_match_res[i], 
                      template_match_res[i], 
                      0, 
                      1, 
                      cv::NORM_MINMAX);

        cv::matchTemplate(displayFrame, templates[i],
                          test[i], CV_TM_CCORR);
        cv::normalize(test[i], 
                      test[i], 
                      0, 
                      1, 
                      cv::NORM_MINMAX);

        template_match_res[i] += test[i];
        cv::normalize(template_match_res[i], 
                      template_match_res[i], 
                      0, 
                      1, 
                      cv::NORM_MINMAX);

        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(template_match_res[i], 
                      &minVal, &maxVal, &minLoc, &maxLoc);

        displayFrame.copyTo(displayBoxed[i]);
        cv::rectangle(displayBoxed[i], maxLoc, 
                      cv::Point(maxLoc.x + templates[i].size().width,
                                maxLoc.y + templates[i].size().height), 
                      0, 2);
    }

    cv::imshow("Blue 1", templates[0]);
    cv::imshow("Blue 2", templates[1]);
    cv::imshow("Green", templates[2]);

    cv::imshow("Blue 1 res", template_match_res[0]);
    cv::imshow("Blue 2 res", template_match_res[1]);
    cv::imshow("Green res", template_match_res[2]);

    cv::imshow("Blue 1 Box", displayBoxed[0]);
    cv::imshow("Blue 2 Box", displayBoxed[1]);
    cv::imshow("Green Box", displayBoxed[2]);

    // Display the image
    cv::imshow("Raw", displayFrame);
    
    // 27 is the ESC key
    while(cv::waitKey(0) != 27);

    std::cout << "Key pressed... Closing.\n";
    return 0;
}
