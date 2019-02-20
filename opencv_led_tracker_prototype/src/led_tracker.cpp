#include "led_tracker.hpp"

void get_colour_threshold_min_max(LedColour led_colour,
                                  cv::Scalar &min,
                                  cv::Scalar &max) {
    switch(led_colour) {
    case LedColour::BLUE:
        min = cv::Scalar(BLUE_HUE_MIN, BLUE_SAT_MIN, BLUE_VAL_MIN);
        max = cv::Scalar(BLUE_HUE_MAX, BLUE_SAT_MAX, BLUE_VAL_MAX);
        break;
    case LedColour::GREEN:
        min = cv::Scalar(GREEN_HUE_MIN, GREEN_SAT_MIN, GREEN_VAL_MIN);
        max = cv::Scalar(GREEN_HUE_MAX, GREEN_SAT_MAX, GREEN_VAL_MAX);
        break;
    }
}

void circle_led(cv::Mat &src_img,
                cv::Mat &dest_img,
                LedColour led_colour,
                cv::Scalar bgr_draw_colour) {
    // First find the locations of the LEDs
    std::vector<LedPos> leds = find_leds2(src_img, led_colour);

    // Iterate through each led and draw to the destination image
    for(int i = 0; i < leds.size(); i++) {
        // Draw circle
        cv::circle(dest_img, 
                   leds[i].center, 
                   leds[i].average_radius, 
                   bgr_draw_colour,
                   3);
       if(leds[i].average_radius > 5) { 
            // Write colour above circle
            std::string text;
            switch(led_colour) {
            case LedColour::BLUE:
                text = "Blue";
                break;
            case LedColour::GREEN:
                text = "Green";
                break;
            }
            cv::putText(dest_img, 
                        text.c_str(), 
                        leds[i].center - cv::Point(
                            10, 
                            leds[i].average_radius + 2
                        ), 
                        CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 
                        1, 
                        bgr_draw_colour,
                        1); 
        }
    }
}

std::vector<LedPos> find_leds(cv::Mat &src_img, LedColour led_colour) {
    // Store the src image in a temporary matrix
    cv::Mat buffer;

    // Apply gaussian blur to filter out some noise
    cv::GaussianBlur(src_img, buffer, cv::Size(11, 11), 0);

    // Create threshold values and populate
    cv::Scalar min, max;
    get_colour_threshold_min_max(led_colour, min, max);
    
    // Split the image into hsv channels
    cv::Mat hsv[3];
    split_and_threshold_channels(buffer, hsv, min, max);

    // Compare overlapping regions for each channel
    cv::bitwise_and(hsv[0], hsv[1], buffer);
    cv::bitwise_and(hsv[2], buffer, buffer);

    // Find the contours of the remaining image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(buffer,
                     contours,
                     CV_RETR_LIST,
                     CV_CHAIN_APPROX_NONE);
    
    // Create output data storage
    std::vector<LedPos> led_positions;
    // Loop through each contour and find the centre and average radius
    for(int i = 0; i < contours.size(); i++) {
        LedPos lp = find_led_pos(contours[i]);
        if(lp.average_radius < 100)
            led_positions.push_back(lp);
    }

    return led_positions;
}

std::vector<LedPos> find_leds2(cv::Mat &src_img, LedColour led_colour) {
    // Store the src image in a temporary matrix
    cv::Mat buffer;

    // Apply gaussian blur to filter out some noise
    cv::GaussianBlur(src_img, buffer, cv::Size(11, 11), 0);

    // Create threshold values and populate
    cv::Scalar min, max;
    get_colour_threshold_min_max(led_colour, min, max);
    
    // Split the image into hsv channels
    cv::Mat hsv[3];
    split_and_threshold_channels(buffer, hsv, min, max);
    
    // DEBUG
    cv::Mat debug_imgs[3];
    for(char i = 0; i < 3; i++)
        hsv[i].copyTo(debug_imgs[i]);

    // Compare overlapping saturation and value regions
    cv::bitwise_and(hsv[1], hsv[2], hsv[1]);

    // Find contours of saturation points
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(hsv[1],
                     contours,
                     CV_RETR_LIST,
                     CV_CHAIN_APPROX_NONE);
    // Loop through each contour and find the hue at the edge
    // If the majority of the edge hue is the correct colour then 
    // it's the LED we want. If it's black it's been thresholded.
    for(int i = 0; i < contours.size(); i++) {
        // Storage variables
        int hue_correct = 0;
        // Loop through each point and check the corresponding hue
        for(int j = 0; j < contours[i].size(); j++) {
            if(hsv[0].at<unsigned char>(contours[i][j].x, 
                                        contours[i][j].y)){
                hue_correct++;
            }
            else {
                hue_correct--;
            }
            std::cout << (int)hsv[0].at<unsigned char>(contours[i][j].x,
                                                       contours[i][j].y) 
                      << std::endl;
        }

        // If the result is positive then more than half are the 
        // correct colour so assume it's the right LED
        if(hue_correct < 0) {
            // Colour is in correct so remove the contour
            contours.erase(contours.begin() + i);
            // Vector automatically resizes so decrement i
            i--;
        }
    }

    // Create output data storage
    std::vector<LedPos> led_positions;
    // Loop through each contour and find the centre and average radius
    for(int i = 0; i < contours.size(); i++) {
        // DEBUG
        cv::drawContours(src_img, contours, -1, cv::Scalar(30, 30, 200));
        // Find minimum area rectangle
        cv::Point2f rect_points[4];
        cv::RotatedRect min_rect = cv::minAreaRect(contours[i]);
        min_rect.points(rect_points);
        for(char j = 0; j < 4; j++) {
            cv::line(src_img, rect_points[j], 
                     rect_points[(j+1)%4], cv::Scalar(30, 200, 30));
        }
        //LedPos lp = find_led_pos(contours[i]);
        cv::drawContours(debug_imgs[0], 
                         contours, 
                         -1, 
                         cv::Scalar(30, 30, 200));
        cv::drawContours(debug_imgs[1], 
                         contours, 
                         -1, 
                         cv::Scalar(30, 30, 200));
        cv::drawContours(debug_imgs[2], 
                         contours, 
                         -1, 
                         cv::Scalar(30, 30, 200));
        //if(lp.average_radius < 100)
        //    led_positions.push_back(lp);
    }
    
    // DEBUG
    cv::imshow("H", debug_imgs[0]);
    cv::imshow("S", debug_imgs[1]);
    cv::imshow("V", debug_imgs[2]);

    return led_positions;
}   

LedPos find_led_pos(std::vector<cv::Point> contour) {
    // Return value
    LedPos led_pos;
    
    // Use cv::moments to compute the moments of the contour
    auto m = cv::moments(contour);
    
    // The centroid is {x, y} = { m10/m00, m01/m00 }
    led_pos.center = cv::Point(m.m10/m.m00, m.m01/m.m00);
    
    // Calculate the average radius as the average distance
    // from the centroid
    led_pos.average_radius = 0;
    for(int i = 0; i < contour.size(); i++) {
        led_pos.average_radius += sqrt(
            pow(contour[i].x - led_pos.center.x, 2) 
          + pow(contour[i].y - led_pos.center.y, 2));
    }
    led_pos.average_radius /= contour.size();
    if(led_pos.average_radius < 1) {
        led_pos.average_radius = 1;
    }

    return led_pos;
}

void split_and_threshold_channels(cv::Mat &src,
                                  cv::Mat *dest,
                                  cv::Scalar min,
                                  cv::Scalar max) {
    // Create a temporary buffer image
    cv::Mat buffer;

    // Convert from BGR to HSV
    cv::cvtColor(src, buffer, CV_BGR2HSV);

    // Split the channels
    cv::split(buffer, dest);

    // Loop through each channel
    for(char i = 0; i < 3; i++) {
        // Threshold everything above the minimum and store temporarily
        // If the minimum is 0 then set all to 255
        if(min[i] == 0) {
            buffer = cv::Mat(dest[i].rows, dest[i].cols, CV_8U, 255);
        }
        else {
            cv::threshold(dest[i], buffer, min[i], 255, cv::THRESH_BINARY);
        }
        // Threshold everything below the maximum and store temporarily
        // If the maximum is 255 then set all to 255
        if(max[i] == 255) {
            dest[i] = cv::Mat(dest[i].rows, dest[i].cols, CV_8U, 255);
        }
        else {
            cv::threshold(dest[i], 
                          dest[i], 
                          max[i], 
                          255, 
                          cv::THRESH_BINARY_INV);
        }

        // Compare the two images and keep the overlapping regions
        cv::bitwise_and(buffer, dest[i], dest[i]);
    }
}
