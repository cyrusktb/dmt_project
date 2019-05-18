#include "glove_position/pose_finder.hpp"

PoseFinder::PoseFinder() :nh_("~") {
    // Get positions of LEDs on glove
    float temp;
    if(!nh_.getParam("green_x", temp))
        ROS_FATAL("Failed to get param 'green_x'");
    led_pos_.A.x = temp;
    if(!nh_.getParam("green_y", temp))
        ROS_FATAL("Failed to get param 'green_y'");
    led_pos_.A.y = temp;
    if(!nh_.getParam("green_z", temp))
        ROS_FATAL("Failed to get param 'green_z'");
    led_pos_.A.z = temp;

    if(!nh_.getParam("blue_1_x", temp))
        ROS_FATAL("Failed to get param 'blue_1_x'");
    led_pos_.A.x = temp;
    if(!nh_.getParam("blue_1_y", temp))
        ROS_FATAL("Failed to get param 'blue_1_y'");
    led_pos_.A.y = temp;
    if(!nh_.getParam("blue_1_z", temp))
        ROS_FATAL("Failed to get param 'blue_1_z'");
    led_pos_.A.z = temp;

    if(!nh_.getParam("blue_2_x", temp))
        ROS_FATAL("Failed to get param 'blue_2_x'");
    led_pos_.A.x = temp;
    if(!nh_.getParam("blue_2_y", temp))
        ROS_FATAL("Failed to get param 'blue_2_y'");
    led_pos_.A.y = temp;
    if(!nh_.getParam("blue_2_z", temp))
        ROS_FATAL("Failed to get param 'blue_2_z'");
    led_pos_.A.z = temp;

    gb1_len_ = mag(led_pos_.A - led_pos_.B);
    gb2_len_ = mag(led_pos_.A - led_pos_.C);
    bb_len_ = mag(led_pos_.B - led_pos_.C);
}

PoseFinder::~PoseFinder() {
    // dtor
}

void PoseFinder::find_pose(std::vector<cv::Point3f> green_points,
                           std::vector<cv::Point3f> blue_points) {
    if(green_points.size() == 0 || blue_points.size() < 2) return;

    // Store the error for each combination
    float error = find_error(green_points[0],
                             blue_points[0],
                             blue_points[1]);
    
    cv::Point3f best_green = green_points[0];
    cv::Point3f best_blue_1 = blue_points[0];
    cv::Point3f best_blue_2 = blue_points[1];

    // Find the best error
    for(int i = 0; i < green_points.size(); i++) {
        for(int j = 0; j < blue_points.size() - 1; j++) {
            for(int k = j+1; k < blue_points.size(); k++) {
                float e = find_error(green_points[i],
                                     blue_points[j],
                                     blue_points[k]);
                if(e < error) {
                    error = e;
                    best_green = green_points[i];
                    best_blue_1 = blue_points[j];
                    best_blue_2 = blue_points[k];
                }
            }
        }
    }

    std::cout << error << std::endl;

    // Find the pose of the glove from the points
    find_glove_pose(best_green, best_blue_1, best_blue_2);
}

float PoseFinder::find_error(cv::Point3f green,
                             cv::Point3f blue_1,
                             cv::Point3f blue_2) {
    // calculate distances between the points
    float gb1, gb2, bb;
    gb1 = mag(green - blue_1);
    gb2 = mag(green - blue_2);
    bb = mag(blue_1 - blue_2);

    // There are two combinations: g->b1->b2 or g->b2->b1
    float err1, err2;
    err1 = sqrt(pow(gb1 - gb1_len_, 2) + 
                pow(gb2 - gb2_len_, 2) + 
                pow(bb - bb_len_, 2));
    err2 = sqrt(pow(gb2 - gb1_len_, 2) + 
                pow(gb1 - gb2_len_, 2) + 
                pow(bb - bb_len_, 2));

    if(err1 > err2)
        return err2;

    return err1;
}

void PoseFinder::find_glove_pose(cv::Point3f green,
                                 cv::Point3f blue_1,
                                 cv::Point3f blue_2) {
    Triangle glove;
    glove.A = green;
    glove.B = blue_1;
    glove.C = blue_2;

    cv::Matx33f R;

    // Find the rotation from the untransformed glove 
    // at 0,0,0 to the real glove
    get_rotation_matrix(R, led_pos_, glove);

    // Now that we have the rotation, calculate the translation
    // Average the translation for each point
    cv::Vec3f trans = (green - R * led_pos_.A +
                       blue_1 - R * led_pos_.B +
                       blue_2 - R * led_pos_.C) / 3;
    
    // Calculate the pose
    tf2::Matrix3x3 M;
    M.setValue(R(0,0), R(0,1), R(0,2),
               R(1,0), R(1,1), R(1,2),
               R(2,0), R(2,1), R(2,2));

    tf2::Quaternion q;
    M.getRotation(q);

    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.child_frame_id = "glove";

    msg.transform.translation.x = trans[0];
    msg.transform.translation.y = trans[1];
    msg.transform.translation.z = trans[2];

    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(msg);

    std::cout << msg << std::endl;
}
