#include "led_detector/led_detector.hpp"

LedDetector::LedDetector(const std::function<void (
                    const std::vector<LedPoint>& green_leds,
                    const std::vector<LedPoint>& blue_leds,
                    const cv::Mat &img
                )>& callback) 
        :nh_("~"), it_(pub_nh_), leds_found_callback(callback) {
    // Get the camera topic name
    std::string topic;
    if(!nh_.getParam("image_topic", topic)) 
        ROS_FATAL("Failed to get param 'image_topic'");

    /// Get other launch parameters
    // Hue threshold values
    if(!nh_.param<int>("green_hue_min", green_hue_[0], 0)) 
        ROS_WARN("Failed to get param 'green_hue_min', defaulting to 0");
    if(!nh_.param<int>("green_hue_max", green_hue_[1], 180)) 
        ROS_WARN("Failed to get param 'green_hue_max', defaulting to 180");

    // Hue threshold values
    if(!nh_.param<int>("blue_hue_min", blue_hue_[0], 0)) 
        ROS_WARN("Failed to get param 'blue_hue_min', defaulting to 0");
    if(!nh_.param<int>("blue_hue_max", blue_hue_[1], 180)) 
        ROS_WARN("Failed to get param 'blue_hue_max', defaulting to 180");

    // Saturation threshold values
    if(!nh_.param<int>("sat_min", sat_[0], 0)) 
        ROS_WARN("Failed to get param 'sat_min', defaulting to 0");
    if(!nh_.param<int>("sat_max", sat_[1], 255)) 
        ROS_WARN("Failed to get param 'sat_max', defaulting to 255");

    // Value threshold values
    if(!nh_.param<int>("val_min", val_[0], 0)) 
        ROS_WARN("Failed to get param 'val_min', defaulting to 0");
    if(!nh_.param<int>("val_max", val_[1], 255)) 
        ROS_WARN("Failed to get param 'val_max', defaulting to 255");

    // Radius threshold values
    if(!nh_.param<int>("radius_min", min_radius_, 0)) 
        ROS_WARN("Failed to get param 'radius_min', defaulting to 0");
    if(!nh_.param<int>("radius_max", max_radius_, 10)) 
        ROS_WARN("Failed to get param 'radius_max', defaulting to 10");

    // Subscribe to input video to receive the image
    sub_=it_.subscribeCamera(topic, 1,
                             &LedDetector::image_callback, this);

    // Debug mode
    if(nh_.param<bool>("debug", debug_, false) && debug_) {
        ROS_INFO("DEBUG MODE ON!");

        cv::namedWindow("Debug Threshold Test", cv::WINDOW_AUTOSIZE);

        cv::createTrackbar("Blue min", "Debug Threshold Test", 
                           &(blue_hue_[0]), 255);
        cv::createTrackbar("Blue max", "Debug Threshold Test", 
                           &(blue_hue_[1]), 255);

        cv::createTrackbar("Green min", "Debug Threshold Test", 
                           &(green_hue_[0]), 255);
        cv::createTrackbar("Green max", "Debug Threshold Test", 
                           &(green_hue_[1]), 255);

        cv::createTrackbar("Sat min", "Debug Threshold Test", 
                           &(sat_[0]), 255);
        cv::createTrackbar("Sat max", "Debug Threshold Test", 
                           &(sat_[1]), 255);

        cv::createTrackbar("Val min", "Debug Threshold Test", 
                           &(val_[0]), 255);
        cv::createTrackbar("Val max", "Debug Threshold Test", 
                           &(val_[1]), 255);
    }
}

void LedDetector::image_callback(const sensor_msgs::ImageConstPtr& msg,
                                 const sensor_msgs::CameraInfoConstPtr& info) {
    // Convert the ROS image to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg,
                                     sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Calculate the intrinsic matrix
    cv::Matx33f intrinsic(info->K[0], 0         , info->K[2],
                          0         , info->K[4], info->K[5],
                          0         , 0         , 1         );
    // We only use the inverse of it though
    intrinsic = intrinsic.inv();

    // Get the h s and v channels from the image and threshold
    cv::Mat hhsv[4];
    split_and_threshold_channels(cv_ptr->image, hhsv);

    // Find the contours in the image
    std::vector<std::vector<cv::Point>> b_contours;
    std::vector<std::vector<cv::Point>> g_contours;
    find_contours(g_contours, b_contours, hhsv);

    // Find the position of each led in the contours vectors
    std::vector<LedPoint> g_led_points;
    std::vector<LedPoint> b_led_points;
    for(int i = 0; i < g_contours.size(); i++) {
        g_led_points.push_back(find_led_pos(g_contours[i]));
    }
    for(int i = 0; i < b_contours.size(); i++) {
        b_led_points.push_back(find_led_pos(b_contours[i]));
    }

    for(int i = 0; i < g_led_points.size(); i++) {
        if(g_led_points[i].avg_radius > max_radius_ ||
           g_led_points[i].avg_radius < min_radius_) {
            g_led_points.erase(g_led_points.begin() + i);
            i--;
        }
        else {
            calculate_ray(g_led_points[i], intrinsic);
        }
    }
    for(int i = 0; i < b_led_points.size(); i++) {
        if(b_led_points[i].avg_radius > max_radius_ ||
           b_led_points[i].avg_radius < min_radius_) {
            b_led_points.erase(b_led_points.begin() + i);
            i--;
        }
        else {
            calculate_ray(b_led_points[i], intrinsic);
        }
    }

    leds_found_callback(g_led_points, b_led_points, cv_ptr->image);
}

void LedDetector::find_contours(
        std::vector<std::vector<cv::Point>>& g_contours,
        std::vector<std::vector<cv::Point>>& b_contours,
        cv::Mat *hhsv) {
    // Create buffer matrix 
    cv::Mat buffer;

    // Compare overlapping hue, saturation and value regions
    cv::bitwise_and(hhsv[2], hhsv[3], buffer);
    cv::bitwise_and(hhsv[0], buffer, buffer);

    // Find the contours of saturation and value points
    cv::findContours(buffer,
                     g_contours,
                     CV_RETR_LIST,
                     CV_CHAIN_APPROX_NONE);

    // Compare overlapping hue, saturation and value regions
    cv::bitwise_and(hhsv[2], hhsv[3], buffer);
    cv::bitwise_and(hhsv[1], buffer, buffer);

    // Find the contours of saturation and value points
    cv::findContours(buffer,
                     b_contours,
                     CV_RETR_LIST,
                     CV_CHAIN_APPROX_NONE);
}

LedPoint LedDetector::find_led_pos(std::vector<cv::Point>& contour) {
    LedPoint p;

    // Use cv::moments to compute the moments of the contour
    auto m = cv::moments(contour);
    
    // The centroid is {x, y} = { m10/m00, m01/m00 }
    p.center = cv::Point(m.m10/m.m00, m.m01/m.m00);

    // Calculate the average radius as the average distance
    // from the centroid
    p.avg_radius = 0;
    for(int i = 0; i < contour.size(); i++) {
        p.avg_radius += sqrt(
            pow(contour[i].x - p.center.x, 2) 
          + pow(contour[i].y - p.center.y, 2));
    }
    p.avg_radius /= contour.size();
    if(p.avg_radius < 1) {
        p.avg_radius = 1;
    }

    return p;
}

void LedDetector::split_and_threshold_channels(cv::Mat& img, 
                                               cv::Mat *hhsv) {
    // Create a temporary buffer image
    cv::Mat buffer;

    // Gaussian blur to filter out noise
    cv::GaussianBlur(img, buffer, cv::Size(11,11), 0);

    // Convert from BGR to HSV
    cv::cvtColor(buffer, buffer, CV_BGR2HSV);

    // Split the channels
    cv::split(buffer, hhsv + 1);

    // Copy the hue channel from the blue channel to the green channel
    hhsv[1].copyTo(hhsv[0]);

    // Create scalars to loop through
    cv::Scalar min(green_hue_[0], blue_hue_[0], sat_[0], val_[0]);
    cv::Scalar max(green_hue_[1], blue_hue_[1], sat_[1], val_[1]);

    // Threshold all channels
    for(char i = 0; i < 4; i++) {
        // Threshold everything above the minimum and store temporarily
        // If the minimum is 0 then set all to 255
        if(min[i] == 0) {
            buffer = cv::Mat(hhsv[i].rows, hhsv[i].cols, CV_8U, 255);
        }
        else {
            cv::threshold(hhsv[i], buffer, min[i], 255, cv::THRESH_BINARY);
        }
        // Threshold everything below the maximum and store temporarily
        // If the maximum is 255 then set all to 255
        if(max[i] == 255) {
            hhsv[i] = cv::Mat(hhsv[i].rows, hhsv[i].cols, CV_8U, 255);
        }
        else {
            cv::threshold(hhsv[i], 
                          hhsv[i], 
                          max[i], 
                          255, 
                          cv::THRESH_BINARY_INV);
        }

        // Compare the two images and keep the overlapping regions
        cv::bitwise_and(buffer, hhsv[i], hhsv[i]);
    }
    
    //Debug mode allows live adjusting of threshold values
    if(debug_) {
        cv::bitwise_or(hhsv[0], hhsv[1], buffer);
        cv::bitwise_and(hhsv[2], buffer, buffer);
        cv::bitwise_and(hhsv[3], buffer, buffer);
        cv::imshow("Temp", buffer);
        cv::waitKey(1);
    }
}

void LedDetector::calculate_ray(LedPoint &p, cv::Matx33f& inv_intrinsic) {
    // Convert to homogenous
    cv::Matx31f hom(p.center.x, p.center.y, 1);

    // Multiply by the inverse of the intrinsic matrix
    hom = inv_intrinsic * hom;

    // Normalise
    cv::Vec3f dir(hom(0), hom(1), hom(2));
    dir /= cv::norm(dir);

    p.ray = dir;
}
