#include "led_detector/point_picker.hpp"

PointPicker::PointPicker() :nh_("~") {
    // Estimate 30 Hz as that's the rate at which the camera runs at
    pub_ = nh_.advertise<cynaptix::LedRayArray>("rays", 30);

    // Debug parameter to allow viewing image output
    nh_.param<bool>("debug", debug_, false);

    if(debug_) {
        ROS_INFO("DEBUG MODE ON!");
        rviz_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
                                                    "rviz_marker", 5);
    }

    if(!nh_.param<int>("led_memory", memory_, 10))
        ROS_WARN("Failed to get param 'led_memory', defaulting to 10");

    if(!nh_.getParam("frame_id", frame_id_))
        ROS_FATAL("Failed to get param 'frame_id'");
}

void PointPicker::led_callback(const std::vector<LedPoint>& green_leds,
                               const std::vector<LedPoint>& blue_leds,
                               const cv::Mat& img) {
    /*
    weigh_green(green_leds);
    weigh_blue(blue_leds);

    remove_low_weights(green_leds);
    remove_low_weights(blue_leds);
    */
    // Create the message and send the LEDs
    cynaptix::LedRayArray msg;
    for(int i = 0; i < green_leds.size(); i++) {
        msg.greens.push_back(geometry_msgs::Vector3());
        msg.greens.back().x = green_leds[i].ray[0];
        msg.greens.back().y = green_leds[i].ray[1];
        msg.greens.back().z = green_leds[i].ray[2];
    }
    for(int i = 0; i < blue_leds.size(); i++) {
        msg.blues.push_back(geometry_msgs::Vector3());
        msg.blues.back().x = blue_leds[i].ray[0];
        msg.blues.back().y = blue_leds[i].ray[1];
        msg.blues.back().z = blue_leds[i].ray[2];
    }

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;

    pub_.publish(msg);


/// ==================================================================
// Drawing for debug purposes
/// ==================================================================
    if(!debug_) return;

    // Draw vectors in rviz for the green LEDs
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();

    marker.ns = frame_id_ + "green";
    marker.id = 0;
    
    marker.type =visualization_msgs::Marker::LINE_LIST;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.01;

    marker.color.g = 1.0f;
    marker.color.a = 1.0f;

    for(int i = 0; i < msg.greens.size(); i++) {
        geometry_msgs::Point p;
        marker.points.push_back(p);
        p.x += msg.greens[i].x / 5.f;
        p.y += msg.greens[i].y / 5.f;
        p.z += msg.greens[i].z / 5.f;
        marker.points.push_back(p);
    }

    rviz_marker_pub_.publish(marker);

    // Repeat for blue LEDs
    marker.ns = frame_id_ + "blue";
    
    marker.color.g = 0.0;
    marker.color.b = 1.0f;

    for(int i = 0; i < msg.blues.size(); i++) {
        geometry_msgs::Point p;
        marker.points.push_back(p);
        p.x += msg.blues[i].x / 5.f;
        p.y += msg.blues[i].y / 5.f;
        p.z += msg.blues[i].z / 5.f;
        marker.points.push_back(p);
    }

    rviz_marker_pub_.publish(marker);



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

