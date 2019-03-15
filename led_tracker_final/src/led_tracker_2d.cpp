#include "led_tracker_2d.hpp"

LedTracker2D::LedTracker2D(float fu, float fv, float cu, float cv,
                           float k1, float k2, float p1, float p2,
                           unsigned int camera_num)
        :cam_(fu, fv, cu, cv, k1, k2, p1, p2, camera_num),
         camera_num_(camera_num) {
    // Raise error if camera failed to open
    if(!cam_.isOpened()) {
        // Throw error in red
        throw std::runtime_error("\033[31mA camera failed to open. Are both connected?\033[0m");
    }
}

LedTracker2D::~LedTracker2D() {
    // dtor
}

std::vector<PotentialLed> LedTracker2D::get_points() {
    // Get the image
    cam_.get_image(img_);

    // Split into HSV channels and threshold
    split_and_threshold_channels();

    // Get the next set of contours
    find_contours();

    // Weigh by hue
    weigh_hue();

    // Weigh pos and combine if successful
    if(weigh_pos())
        combine_weights();
    else {
        // Copy hue weights to total weights
        for(int i = 0; i < potential_leds_.size(); i++) {
            for(char j = 0; j < 3; j++) {
                potential_leds_[i].total_weight[j] = 
                        potential_leds_[i].hue_weight[j];
            }
        }
    }
    
    // Update calculated rays
    update_rays();

    // Return our potential leds
    return potential_leds_;
}

void LedTracker2D::get_img(cv::Mat &img) {
    img_.copyTo(img);
}

void LedTracker2D::set_prev_leds(std::vector<PotentialLed> prev_leds) {
    if(prev_leds.size() < 3) {
        // Set all to zero, we don't have enough data to perform
        // position based estimation
        prev_led_pos_[0] = cv::Point(0, 0);
        prev_led_pos_[1] = cv::Point(0, 0);
        prev_led_pos_[2] = cv::Point(0, 0);
        return;
    }
    for(char i = 0; i < 3; i++) {
        if(prev_led_pos_[i] != cv::Point(0, 0))
            // Reduce noise by including previous estimations
            prev_led_pos_[i] = prev_leds[i].center * 0.7 
                             + prev_led_pos_[i] * 0.3;
        else
            prev_led_pos_[i] = prev_leds[i].center;
    }
}

void LedTracker2D::weigh_hue() {
    // Loop through each potential led
    for(int i = 0; i < potential_leds_.size(); i++) {
        // Prevent overly large "LEDs" from slowing down the program
        // These are typically bright windows or ceiling lights etc.
        // Filter out tiny "LEDs" caused by background noise
        if(potential_leds_[i].avg_radius < 10 &&
           potential_leds_[i].avg_radius > 3) {
            float size_mult;
            for(size_mult = CIRCLE_MULT_MIN;
                size_mult <= CIRCLE_MULT_MAX;
                size_mult += CIRCLE_MULT_STEP) {
                
                float r = potential_leds_[i].avg_radius * size_mult;
                
                // Create a circular mask 
                cv::Mat mask(cv::Size(2*r, 2*r), 
                             CV_8U, 
                             cv::Scalar(0));
                // Outer cicle is all 255
                cv::circle(mask,
                           cv::Point2d(r, r),
                           r,
                           cv::Scalar(255),
                           -1,
                           cv::LINE_AA);
                // Inner circle is all 0 as this will be the wrong hue
                // due to the excessive saturation and value
                cv::circle(mask,
                           cv::Point2d(r, r),
                           potential_leds_[i].avg_radius,
                           cv::Scalar(0),
                           -1,
                           cv::LINE_AA);

                // Create a (circular) ROI over this contour
                cv::Rect region(potential_leds_[i].center - cv::Point(r, r),
                                cv::Size(2*r, 2*r));

                cv::Mat roi;

                // Loop through each colour
                for(char col = 0; col < 2; col++) {
                    // Ignore this if the roi leaves the screen
                    try {
                        roi = cv::Mat(hsv_[col], region);
                    }
                    catch(cv::Exception e) {
                        continue;
                    }
                    
                    // Actually make the circular roi now
                    cv::Mat c_roi;
                    cv::bitwise_and(roi, roi, c_roi, mask);

                    // Check what percentage of the correct hue is 
                    // within the c_roi
                    int correct_count = 0;
                    int total_count = 0;
                    for(int y = 0; y < c_roi.rows; y++) {
                        for(int x = 0; x < c_roi.cols; x++) {
                            if(c_roi.at<char>(y, x))
                                correct_count++;
                            total_count++;
                        }
                    }
                    // Update weighting
                    float new_weight = (float)correct_count 
                                     / (float)total_count;
                    if(new_weight > potential_leds_[i].hue_weight[col])
                        potential_leds_[i].hue_weight[col] = new_weight;
                }
            }
            // Delete the potential led if the weighting isn't large enough
            if(potential_leds_[i].hue_weight[0] < MIN_HUE_WEIGHT &&
               potential_leds_[i].hue_weight[1] < MIN_HUE_WEIGHT) {
                // Delete this one and don't skip the next
                potential_leds_.erase(potential_leds_.begin() + i);
                i--;
            }
            else {
                // Copy the blue hue weight to the second blue led
                potential_leds_[i].hue_weight[2] 
                        = potential_leds_[i].hue_weight[1];
            }
        }
        else {
            // Delete this one and don't skip the next
            potential_leds_.erase(potential_leds_.begin() + i);
            i--;
        }
    }
}

bool LedTracker2D::weigh_pos() {
    return false;
    // Check if we have position data yet
    if(prev_led_pos_[0] == cv::Point(0, 0) || 
       prev_led_pos_[1] == cv::Point(0, 0) || 
       prev_led_pos_[2] == cv::Point(0, 0)) {
        return false;
    }

    // Minimum distance from the 3 led points
    cv::Vec3f min_led_distance(999, 999, 999);
    
    // Loop through each potential led and weigh each colour
    for(int p = 0; p < potential_leds_.size(); p++) {
        // Compare the distance between the center 
        // and each previous point
        for(char i = 0; i < 3; i++) {
            float dist = sqrt(pow(prev_led_pos_[i].x 
                                - potential_leds_[p].center.x, 2) +
                              pow(prev_led_pos_[i].y
                                - potential_leds_[p].center.y, 2));
            if(dist < min_led_distance[i])
                min_led_distance[i] = dist;
            potential_leds_[p].pos_weight[i] = dist;
        }
    }
        
    // Loop through and normalise weightings
    for(int p = 0; p < potential_leds_.size(); p++) {
        // Flag to allow us to delete pairs below a certain threshold
        bool above_thresh = false;

        // Loop through each weight
        for(char i = 0; i < 3; i++) {
            // Weight based on position similarity
            // Very far out points will be ignored
            // The closest point will always have a weighting of 1
            potential_leds_[p].pos_weight[i] = 
                    min_led_distance[i] / potential_leds_[p].pos_weight[i];

            if(potential_leds_[p].pos_weight[i] >= MIN_POS_WEIGHT)
                above_thresh = true;
        }

        if(!above_thresh) {
            potential_leds_.erase(potential_leds_.begin() + p);
            p--;
        }
    }
    return true;
}

void LedTracker2D::combine_weights() {
    // Loop through each potential led
    for(int i = 0; i < potential_leds_.size(); i++) {
        // Flag for thresholding
        bool above_thresh = false;

        // Multiply position and colour weights to get total weight
        for(char j = 0; j < 3; j++) {
            potential_leds_[i].total_weight[j] = 
                    potential_leds_[i].hue_weight[j] 
                  * potential_leds_[i].pos_weight[j];
            // Threshold
            if(potential_leds_[i].total_weight[j] >= MIN_TOTAL_WEIGHT)
                above_thresh = true;
        }

        // Delete if below the threshold
        if(!above_thresh) {
            potential_leds_.erase(potential_leds_.begin() + i);
            i--;
        }
    }
}

void LedTracker2D::find_contours() {
    // Create buffer matrix 
    cv::Mat buffer;

    // Compare overlapping saturation and value regions
    cv::bitwise_and(hsv_[2], hsv_[3], buffer);

    // Create temporary contours vector
    std::vector<std::vector<cv::Point>> contours;

    // Find the contours of saturation and value points
    cv::findContours(buffer,
                     contours,
                     CV_RETR_LIST,
                     CV_CHAIN_APPROX_NONE);

    // Clear the potential LEDs vector
    potential_leds_.clear();

    // Loop through the contours and create a potential LED
    for(int i = 0; i < contours.size(); i++) {
        potential_leds_.push_back(PotentialLed());

        // Store the contour
        potential_leds_.back().contour = contours[i];
        
        // Calculate average radius and position
        find_led_pos(&potential_leds_.back());
    }
}

void LedTracker2D::update_rays() {
    // Loop through the potential LEDs and calculate the ray for this LED
    for(int i = 0; i < potential_leds_.size(); i++) {
        potential_leds_[i].ray = rot * cam_.get_ray(potential_leds_[i].center);
    }
}

void LedTracker2D::find_led_pos(PotentialLed * pl) {
    // Use cv::moments to compute the moments of the contour
    auto m = cv::moments(pl->contour);
    
    // The centroid is {x, y} = { m10/m00, m01/m00 }
    pl->center = cv::Point(m.m10/m.m00, m.m01/m.m00);
    
    // Calculate the average radius as the average distance
    // from the centroid
    pl->avg_radius = 0;
    for(int i = 0; i < pl->contour.size(); i++) {
        pl->avg_radius += sqrt(
            pow(pl->contour[i].x - pl->center.x, 2) 
          + pow(pl->contour[i].y - pl->center.y, 2));
    }
    pl->avg_radius /= pl->contour.size();
    if(pl->avg_radius < 1) {
        pl->avg_radius = 1;
    }
}

void LedTracker2D::split_and_threshold_channels() {
    // Create a temporary buffer image
    cv::Mat buffer;

    // Gaussian blur to filter out noise
    cv::GaussianBlur(img_, buffer, cv::Size(11,11), 0);

    // Convert from BGR to HSV
    cv::cvtColor(buffer, buffer, CV_BGR2HSV);

    // Split the channels
    cv::split(buffer, hsv_ + 1);

    // Copy the hue channel from the blue channel to the green channel
    hsv_[1].copyTo(hsv_[0]);

    // Create scalars to loop through
    cv::Scalar min(GREEN_HUE_MIN, BLUE_HUE_MIN, SAT_MIN, VAL_MIN);
    cv::Scalar max(GREEN_HUE_MAX, BLUE_HUE_MAX, SAT_MAX, VAL_MAX);

    // Threshold all channels
    for(char i = 0; i < 4; i++) {
        // Threshold everything above the minimum and store temporarily
        // If the minimum is 0 then set all to 255
        if(min[i] == 0) {
            buffer = cv::Mat(hsv_[i].rows, hsv_[i].cols, CV_8U, 255);
        }
        else {
            cv::threshold(hsv_[i], buffer, min[i], 255, cv::THRESH_BINARY);
        }
        // Threshold everything below the maximum and store temporarily
        // If the maximum is 255 then set all to 255
        if(max[i] == 255) {
            hsv_[i] = cv::Mat(hsv_[i].rows, hsv_[i].cols, CV_8U, 255);
        }
        else {
            cv::threshold(hsv_[i], 
                          hsv_[i], 
                          max[i], 
                          255, 
                          cv::THRESH_BINARY_INV);
        }

        // Compare the two images and keep the overlapping regions
        cv::bitwise_and(buffer, hsv_[i], hsv_[i]);
    }
   /* 
    char name[10];

    memset(name, '\0', 10);
    sprintf(name, "H - %d", camera_num_);
    cv::imshow(name, hsv_[0]);

    memset(name, '\0', 10);
    sprintf(name, "S - %d", camera_num_);
    cv::imshow(name, hsv_[2]);

    memset(name, '\0', 10);
    sprintf(name, "V - %d", camera_num_);
    cv::imshow(name, hsv_[3]);*/
}
