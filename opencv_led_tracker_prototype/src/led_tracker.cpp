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
    std::vector<LedPos> leds = find_leds(src_img, led_colour);

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
    
    // Compare overlapping saturation and value regions
    cv::bitwise_and(hsv[1], hsv[2], hsv[1]);

    // Find contours of saturation and value points
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(hsv[1],
                     contours,
                     CV_RETR_LIST,
                     CV_CHAIN_APPROX_NONE);
    
    // Loop through each contour and find the center point and average
    // Radius. Check if the most common hue within a radius of the
    // center is the one we're looking for and match if it is
    std::vector<LedPos> led_positions;
    for(int i = 0; i < contours.size(); i++) {
        LedPos lp = find_led_pos(contours[i]);
        
        // Prevent overly large "LEDs" from slowing down the program
        // These are typically bright windows or ceiling lights etc.
        // Filter out tiny "LEDs" which can be caused by noise
        if(lp.average_radius < 20 && lp.average_radius > 3) {
            // See if there's a circle ranging from CIRCLE_MULT_MIN to 
            // CIRCLE_MULT_MAX which gives the correct colour
            bool correct_colour = false;
            float size_mult;
            for(size_mult = CIRCLE_MULT_MIN; 
                size_mult <= CIRCLE_MULT_MAX && !correct_colour;
                size_mult += CIRCLE_MULT_STEP) {
                
                float r = lp.average_radius * size_mult;
                
                // Create a (circular) ROI over this contour
                cv::Rect region(lp.center - cv::Point(r,r), 
                                cv::Size(2*r, 2*r));

                cv::Mat roi;
                // Ignore this if the roi leaves the screen
                try {
                    roi = cv::Mat(hsv[0], region);
                }
                catch(cv::Exception) {
                    continue;
                }
                
                cv::Mat mask(cv::Size(2*r, 2*r), CV_8U, cv::Scalar(0));
                cv::circle(mask, 
                           cv::Point2d(r, r), 
                           r, 
                           cv::Scalar(255), 
                           -1, 
                           cv::LINE_AA);
                cv::Mat c_roi;
                cv::bitwise_and(roi, roi, c_roi, mask);

                // Check the most common hue of the ROI
                int correct_count = 0;
                int total_count = 0;
                for(int i = 0; i < c_roi.rows; i++) {
                    for(int j = 0; j < c_roi.cols; j++) {
                        if(c_roi.at<char>(i, j))
                            correct_count++;
                        total_count++;
                    }
                }

                // If the counter is positive then we have our LED!!!
                if((float)correct_count / (float)total_count >= 0.33)
                    correct_colour = true;
            }

            if(correct_colour)
                led_positions.push_back(lp); 
        }
    }

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
