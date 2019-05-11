#include <ros/ros.h>
#include <led_detector/led_detector.hpp>


void led_callback(const std::vector<LedPoint>& green_leds,
                  const std::vector<LedPoint>& blue_leds,
                  const cv::Mat& img) {
    for(int i = 0; i < green_leds.size(); i++) {
        std::cout << green_leds[i].center << " || "
                  << green_leds[i].avg_radius << std::endl;
        cv::circle(img, 
                   green_leds[i].center, 
                   2*green_leds[i].avg_radius,
                   cv::Scalar(20, 255, 20),
                   2);
    }
    for(int i = 0; i < blue_leds.size(); i++) {
        cv::circle(img, 
                   blue_leds[i].center, 
                   2*blue_leds[i].avg_radius,
                   cv::Scalar(255, 20, 20),
                   2);
    }
    cv::imshow("Image", img);

    cv::Mat hsv[3], buffer;

    // Gaussian blur to filter out noise
    cv::GaussianBlur(img, buffer, cv::Size(11,11), 0);

    // Convert from BGR to HSV
    cv::cvtColor(buffer, buffer, CV_BGR2HSV);

    // Split the channels
    cv::split(buffer, hsv);


    cv::imshow("H", hsv[0]);
    cv::imshow("S", hsv[1]);
    cv::imshow("V", hsv[2]);

    cv::waitKey(1);
};

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "led_detector");

    // Create led detector
    LedDetector detector(&led_callback);

    // Process events until ROS closes the node
    ros::spin();
    return 0;
}
