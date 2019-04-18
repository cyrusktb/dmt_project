#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "camera.hpp"

int main(int argc, char **argv) {
    if(argc != 2) {
        std::cout << "Usage: " 
                  << argv[0] 
                  << " <object>"
                  << std::endl;
        return -1;
    }

    // Create camera class instance
    float params[10] = {
        480, 640,
        722.106766, 723.389819,
        344.447625, 271.702332,
        -0.430658, 0.235174,
        0.000098, -0.000494
    };
    Camera cam1(params[0], params[1], params[2], params[3], params[4],
                params[5], params[6], params[7], params[8], params[9], 1);
    // If the camera failed to open, then return
    if(!cam1.isOpened()) {
        std::cout << "Failed to open Camera 1" << std::endl;
        return -1;
    }

    // Mostly copied from the opencv feature homography tutorial

    cv::Mat object = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat scene;

    cv::Ptr<cv::FeatureDetector> brisk = cv::BRISK::create();
    cv::BFMatcher matcher;

    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
    brisk->detect(object, keypoints_object);

    cv::Mat descriptors_object, descriptors_scene;
    brisk->compute(object, keypoints_object, descriptors_object);

    while(true) {
        cam1.get_image(scene);
        cv::cvtColor(scene, scene, CV_BGR2GRAY);

        // Step 1: Detect keypoints
        keypoints_scene.clear();
        brisk->detect(scene, keypoints_scene);

        // Step 2: Calculate descriptors (feature vectors)
        brisk->compute(scene, keypoints_scene, descriptors_scene);

        // Match descriptor vectors
        //cv::FlannBasedMatcher matcher;
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors_object, descriptors_scene, matches);

        // Calculate max and min distances between keypoints
        double max_dist = 0;
        double min_dist = 9999;
        for(int i = 0; i < descriptors_object.rows; i++) {
            double dist = matches[i].distance;
            if(dist < min_dist) min_dist = dist;
            if(dist > max_dist) max_dist = dist;
        }

        // Draw only "good" matches (less than 3*min_dist)
        std::vector<cv::DMatch> good_matches;
        for(int i = 0; i < descriptors_object.rows; i++) {
            if(matches[i].distance < 3 * min_dist)
                good_matches.push_back(matches[i]);
        }

        cv::Mat img_matches;
        cv::drawMatches(object, 
                        keypoints_object,
                        scene, 
                        keypoints_scene,
                        good_matches, 
                        img_matches,
                        cv::Scalar::all(-1), 
                        cv::Scalar::all(-1),
                        std::vector<char>(), 
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // Localise the object
        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> sc;

        for(int i = 0; i < good_matches.size(); i++) {
            obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
            sc.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
        }

        cv::Mat H = findHomography(obj, sc, CV_RANSAC);

        // Get the corners from the template
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cv::Point(0, 0);
        obj_corners[1] = cv::Point(object.cols, 0);
        obj_corners[2] = cv::Point(object.cols, object.rows);
        obj_corners[3] = cv::Point(0, object.rows);
        
        std::vector<cv::Point2f> scene_corners(4);

        cv::perspectiveTransform(obj_corners, scene_corners, H);

        // Draw lines between the corners from the mapped object to the scene
        cv::line(img_matches, 
                 scene_corners[0] + cv::Point2f(object.cols, 0),
                 scene_corners[1] + cv::Point2f(object.cols, 0),
                 cv::Scalar(0, 255, 0),
                 4);
        cv::line(img_matches, 
                 scene_corners[1] + cv::Point2f(object.cols, 0),
                 scene_corners[2] + cv::Point2f(object.cols, 0),
                 cv::Scalar(0, 255, 0),
                 4);
        cv::line(img_matches, 
                 scene_corners[2] + cv::Point2f(object.cols, 0),
                 scene_corners[3] + cv::Point2f(object.cols, 0),
                 cv::Scalar(0, 255, 0),
                 4);
        cv::line(img_matches, 
                 scene_corners[3] + cv::Point2f(object.cols, 0),
                 scene_corners[0] + cv::Point2f(object.cols, 0),
                 cv::Scalar(0, 255, 0),
                 4);

        cv::imshow("Matches", img_matches);

        if(cv::waitKey(30) == 27) break;
    }

    return 0;
}
