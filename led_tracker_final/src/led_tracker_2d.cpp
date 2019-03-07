#include "led_tracker.hpp"

// Useful vector functions
float dot(cv::Vec3f a, cv::Vec3f b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float mag(cv::Vec3f a) {
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

LedTracker::LedTracker() :thread_(&LedTracker::loop, this), 
                          camera_rot_{LEFT_CAMERA_ROT,
                                      RIGHT_CAMERA_ROT},
                          camera_pos_{LEFT_CAMERA_POS,
                                      RIGHT_CAMERA_POS},
                          camera_{ Camera(LEFT_CAMERA_PARAMS), 
                                   Camera(RIGHT_CAMERA_PARAMS) },
                          running_(true) {
    // Raise error if either camera failed to open
    if(!camera_[0].isOpened()) {
        throw std::runtime_error("Left camera failed to open.\
                                  \nIs it connected?");
    }
    if(!camera_[1].isOpened()) {
        throw std::runtime_error("Right camera failed to open.\
                                  \nIs it connected?");
    }
}

LedTracker::~LedTracker() {
    running_ = false;
    thread_.join();
}

cv::Vec3f LedTracker::get_led_pos(LedNum led) {
    switch(led) {
    case GREEN_LED:
        return led_pos_[GREEN][0];
        break;
    case BLUE_LED_L:
        return led_pos_[BLUE_L][0];
        break;
    case BLUE_LED_R:
        return led_pos_[BLUE_R][0];
        break;
    }
}

void LedTracker::loop() {
    while(running_) {
        // Get the camera images
        update_images();

        // Split and threshold each image
        split_and_threshold_channels(&img_[0], hsv_[0]); 
        split_and_threshold_channels(&img_[1], hsv_[1]);

        // Update the thresholded contour list
        update_contours();

        // Update the contour hue-based weighting
        update_hue_weighting();

        // Update the contour position-based weighting
        update_position_weighting();

        // Pick the most likely candidates for the three LED positions
        calculate_LED_positions();
    }
}

void LedTracker::update_images() {
    // Threadsafe lock 
    std::lock_guard<std::mutex> lock(image_mutex_);
    
    // Get the images
    camera_[0].get_image(img_[0]);
    camera_[1].get_image(img_[1]);
}

void LedTracker::update_contours() {
    // Create buffer matrix for each camera
    cv::Mat buffer;

    // Repeat for each camera
    for(char i = 0; i < 2; i++) {
        // Compare overlapping saturation and value regions
        cv::bitwise_and(hsv_[i][2], hsv_[i][3], buffer);

        // Clear the old contours vector
        contours_[i].clear();

        // Find the contours of saturation and value points
        cv::findContours(buffer,
                         contours_[i],
                         CV_RETR_LIST,
                         CV_CHAIN_APPROX_NONE);
    }
}

void LedTracker::update_hue_weighting() {
    // Loop through each image
    for(char i; i < 2; i++) {
        // Clear storage structures
        hue_weighting_[i].clear();
        potential_leds_[i].clear();

        // Loop though each contour
        for(int c = 0; c < contours_[i].size(); c++) {
            LedPos lp = find_led_pos(contours_[i][c]);
            
            float weighting[2] = {0, 0};
            
            // Prevent overly large "LEDs" from slowing down the program
            // These are typically bright windows or ceiling lights etc.
            // Filter out tiny "LEDs" caused by background noise
            if(lp.average_radius < 20 && lp.average_radius > 3) {
                float size_mult;
                for(size_mult = CIRCLE_MULT_MIN;
                    size_mult <= CIRCLE_MULT_MAX;
                    size_mult += CIRCLE_MULT_STEP) {

                    float r = lp.average_radius * size_mult;
                    
                    // Create a circular mask 
                    cv::Mat mask(cv::Size(2*r, 2*r), 
                                 CV_8U, 
                                 cv::Scalar(0));
                    cv::circle(mask,
                               cv::Point2d(r, r),
                               r,
                               cv::Scalar(255),
                               -1,
                               cv::LINE_AA);

                    // Create a (circular) ROI over this contour
                    cv::Rect region(lp.center - cv::Point(r, r),
                                    cv::Size(2*r, 2*r));

                    cv::Mat roi;

                    // Loop through each colour
                    for(char col = 0; col < 2; col++) {
                        // Ignore this if the roi leaves the screen
                        try {
                            roi = cv::Mat(hsv_[i][col], region);
                        }
                        catch(cv::Exception) {
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
                        if(new_weight > weighting[col])
                            weighting[col] = new_weight;
                    }
                }
            }
            
            // Store the contour if the weighting is large enough
            if(weighting[0] >= MIN_HUE_WEIGHT ||
               weighting[1] >= MIN_HUE_WEIGHT) {
                hue_weighting_[i].push_back(
                    std::pair<float, float>(weighting[0], weighting[1])
                );
                potential_leds_[i].push_back(lp);
            }
            else {
                // Delete the item from the contours as there is no point
                // comparing the position of this contour
                contours_[i].erase(contours_[i].begin() + c);
                c--;
            }
        }
    }
}

void LedTracker::update_position_weighting() {
    // Clear vector
    pair_weighting_.clear();

    // Find average positions of the three leds from previous values
    cv::Vec3f old_position[3];

    for(char i = 0; i < 3; i++) {
        old_position[i][0] = led_pos_[i][LED_POS_MEMORY - 1][0]
                          / (2 << (LED_POS_MEMORY - 1));
        old_position[i][1] = led_pos_[i][LED_POS_MEMORY - 1][1]
                          / (2 << (LED_POS_MEMORY - 1));
        old_position[i][2] = led_pos_[i][LED_POS_MEMORY - 1][2]
                          / (2 << (LED_POS_MEMORY - 1));
        for(int j = LED_POS_MEMORY - 1; j != -1; j--) {
            // Each position is valued at half the next most recent one
            // This reduces noise, but also prevents slow adaptation
            old_position[i][0] = led_pos_[i][j][0] / (2 << j);
            old_position[i][1] = led_pos_[i][j][1] / (2 << j);
            old_position[i][2] = led_pos_[i][j][2] / (2 << j);

            // Also move the led_pos_ along so that the next position 
            // can be written into position 0
            if(j != LED_POS_MEMORY -1)
                led_pos_[i][j+1] = led_pos_[i][j];
        }
    }

    // If the last data point z position is 0 then we do not have 
    // enough data to perform memory-based estimation yet, so return early
    for(char i = 0; i < 3; i++) {
        if(led_pos_[i][LED_POS_MEMORY - 1][2] == 0) {
            position_estimated_ = false;
            return;
        }
    }

    position_estimated_ = true;

    // Minimum distance from the 3 led points
    cv::Vec3f min_led_distance(999, 999, 999);
    
    // Loop through each potential led and weigh each colour
    for(int left = 0; left < potential_leds_[0].size(); left++) {
        for(int right = 0; right < potential_leds_[1].size(); right++) {
            // Find the rays coming out from the point in the camera
            cv::Vec3f ray[2];
            ray[0] = camera_[0].get_ray(potential_leds_[0][left].center);
            ray[1] = camera_[1].get_ray(potential_leds_[1][right].center);

            // Rotate the rays by the camera rotation matrices
            ray[0] = camera_rot_[0] * ray[0];
            ray[1] = camera_rot_[1] * ray[1];
            
            // Find the intersection point of the pair of rays
            cv::Point3f center;
            float ray_distance;

            intersect_rays(ray[0], ray[1], &center, &ray_distance);

            // Compare the distance between the center 
            // and each previous point
            float weightings[3];

            for(char i = 0; i < 3; i++) {
                float dist = mag((cv::Vec3f)center - old_position[i]);
                if(dist < min_led_distance[i])
                    min_led_distance[i] = dist;
                weightings[i] = dist;
            }
            
            // Create a new potential pair
            pair_weighting_.push_back(std::make_tuple(weightings[0],
                                                      weightings[1],
                                                      weightings[2],
                                                      left, right));
        }
    }

    // Loop through and normalise weightings
    for(int i = 0; i < pair_weighting_.size(); i++) {
        // Flag to allow us to delete any pairs below a certain threshold
        bool above_thresh = false;
        float *w[3];
        w[0] = &std::get<0>(pair_weighting_[i]);
        w[1] = &std::get<1>(pair_weighting_[i]);
        w[2] = &std::get<2>(pair_weighting_[i]);
        for(char j = 0; j < 3; j++) {
            // Weight based on position similarity
            // Very far out points will be ignored
            // The closest point will always have a weighting of 1
            *w[j] = min_led_distance[j] / *w[j];

            if(*w[j] > MIN_POS_WEIGHT)
                above_thresh = true;
        }
        if(!above_thresh) {
            pair_weighting_.erase(pair_weighting_.begin() + i);
            i--;
        }
    }
}

void LedTracker::calculate_LED_positions() {
    // If we haven't done any position estimating then just use hue data
    if(!position_estimated_) {
        // Take the two highest blue LED points and the highest green
        // LED points from each image

        // 0 is GREEN, 1 is BLUE_L, 2 is BLUE_R
        float max_hues[2][3] = {{0, 0, 0}, {0, 0, 0}};
        int pos[2][3] = {{0, 0, 0}, {0, 0, 0}};

        // Loop by image
        for(char j = 0; j < 2; j++) {
            // Loop through weights
            for(int i = 0; i < hue_weighting_[j].size(); i++) {
                if(std::get<0>(hue_weighting_[j][i]) > max_hues[j][0]) {
                    // Update best weighting and position
                    max_hues[j][0] = std::get<0>(hue_weighting_[j][i]);
                    pos[j][0] = i;
                }
                if(std::get<1>(hue_weighting_[j][i]) > max_hues[j][1]) {
                    // Store second best weighting and position
                    max_hues[j][2] = max_hues[j][1];
                    pos[j][2] = pos[j][1];

                    // Update best weighting and position
                    max_hues[j][1] = std::get<1>(hue_weighting_[j][i]);
                    pos[j][1] = i;
                }
                else if(std::get<1>(hue_weighting_[j][i]) > max_hues[j][2]) {
                    // Update best weighting and position
                    max_hues[j][2] = std::get<1>(hue_weighting_[j][i]);
                    pos[j][2] = i;
                }
            }
        }

        // Find which pose is the most realistic for a non-rotated hand
        // TODO: Potentially consider up to 3 points for each point and
        // use a cost-based optimisation algorithm? 
        // This could also be used with the position-based weighting
        // algorithm
        return;
    }

    // Store the best 3 weights for green and 6 for blue and their locations
    float best_weights[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    float best_iterators[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    
    // Simple method: Multiply the two weights and pick the best one
    for(int i = 0; i < pair_weighting_.size(); i++) {
        float w[3];
        float w[0] = std::get<0>(pair_weighting_[i]);
        float w[1] = std::get<1>(pair_weighting_[i]);
        float w[2] = std::get<2>(pair_weighting_[i]);
        int l = std::get<3>(pair_weighting_[i]);
        int r = std::get<4>(pair_weighting_[i]);

        // Multiply weights to find the total weight
        w[0] *= std::get<0>(hue_weighting_[l])
             * std::get<0>(hue_weighting_[r]);

        w[1] *= std::get<1>(hue_weighting_[l])
             * std::get<1>(hue_weighting_[r]);

        w[2] *= std::get<1>(hue_weighting_[l])
             * std::get<1>(hue_weighting_[r]);
            
        // Compare to best results and store
        for(char j = 0; j < 3; j++) {
            if(w[j] > best_weights[j][0]) {
                best_weights[j][2] = best_weights[j][1];
                best_weights[j][1] = best_weights[j][0];
                best_weights[j][0] = w[j];
                best_iterators[j][2] = best_iterators[j][1];
                best_iterators[j][1] = best_iterators[j][0];
                best_iterators[j][0] = i;
            }
            else if(w[j] > best_weights[j][1]) {
                best_weights[j][2] = best_weights[j][1];
                best_weights[j][1] = w[j];
                best_iterators[j][2] = best_iterators[j][1];
                best_iterators[j][1] = i;
            }
            else if(w[j] > best_weights[j][2]) {
                best_weights[j][2] = w[j];
                best_iterators[j][2] = i;
            }
        }
    }
    // Find which combination of points has the lowest cost function.
}

LedPos LedTracker::find_led_pos(std::vector<cv::Point> contour) {
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

void LedTracker::split_and_threshold_channels(cv::Mat *src, cv::Mat *dest) {
    // Create a temporary buffer image
    cv::Mat buffer;

    // Gaussian blur to filter out noise
    cv::GaussianBlur(*src, buffer, cv::Size(11,11), 0);

    // Convert from BGR to HSV
    cv::cvtColor(buffer, buffer, CV_BGR2HSV);

    // Split the channels
    cv::split(buffer, dest + 1);

    // Copy the hue channel from the blue channel to the green channel
    dest[1].copyTo(dest[0]);

    // Create scalars to loop through
    cv::Scalar min(GREEN_HUE_MIN, BLUE_HUE_MIN, SAT_MIN, VAL_MIN);
    cv::Scalar max(GREEN_HUE_MAX, BLUE_HUE_MAX, SAT_MAX, VAL_MAX);

    // Threshold all channels
    for(char i = 0; i < 4; i++) {
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

void LedTracker::intersect_rays(cv::Vec3f l, 
                                cv::Vec3f r, 
                                cv::Point3f *center,
                                float *distance) {
    // left starts at camera_pos_[0], right starts at camera_pos_[1]
    cv::Vec3f c = l - r;
    
    // Calculate points on each line where the lines are closest
    cv::Vec3f l_p = camera_pos_[0];
    l_p += ((dot(l, r)*dot(r, c) + dot(l, c)*dot(r, r))
           / (dot(l, l)*dot(r, r) - dot(l, r)*dot(l, r))) * l;
                    
    cv::Vec3f r_p = camera_pos_[1];
    r_p += ((dot(l, r)*dot(l, c) + dot(r, c)*dot(l, l))
           / (dot(l, l)*dot(r, r) - dot(l, r)*dot(l, r))) * r;

    // Find vector between points
    cv::Vec3f v = l_p - r_p;
    
    // Find distance and center point
    *distance = mag(v);
    *center = r_p + 0.5*v;
}
