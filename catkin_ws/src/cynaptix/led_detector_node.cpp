#include <ros/ros.h>
#include <led_detector/led_detector.hpp>
#include <led_detector/point_picker.hpp>

int main(int argc, char **argv) {
    // Init ROS
    ros::init(argc, argv, "led_detector");

    // Create point picker
    PointPicker picker;

    // Create led detector
    LedDetector detector([&picker](
        const std::vector<LedPoint>& greens,
        const std::vector<LedPoint>& blues,
        const cv::Mat& img) {
            picker.led_callback(greens, blues, img);
        }
    );

    // Process events until ROS closes the node
    ros::spin();
    return 0;
}
