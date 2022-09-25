#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;

int main() {
    // We use FAST for keypoint detection and BRIEF for description
    // Combined in ORB for orientation component
    cv::Ptr<cv::ORB> detector = cv::ORB::create();

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

    std::vector<cv::KeyPoint> lastFrameKeypoints1, lastFrameKeypoints2;

    cv::Mat lastFrameDescriptors1, lastFrameDescriptors2;

    std::vector<cv::DMatch > matches;

    int movementDirection = 0;
    std::string image_path1 = cv::samples::findFile("Hill1.jpg");
    std::string image_path2 = cv::samples::findFile("Hill2.jpg");

    cv::Mat image1 = cv::imread(image_path1);
    cv::Mat image2 = cv::imread(image_path2);

    // Find keypoints and their descriptor using ORB
    detector->detectAndCompute(image1, cv::noArray(), lastFrameKeypoints1, lastFrameDescriptors1);
    detector->detectAndCompute(image2, cv::noArray(), lastFrameKeypoints2, lastFrameDescriptors2);

    // Match the descriptor between two images, put them in matches vector
    matcher->match(lastFrameDescriptors1, lastFrameDescriptors2, matches);

    std::vector<cv::Point2d> good_point1, good_point2;
    good_point1.reserve(matches.size());
    good_point2.reserve(matches.size());

    // Calculate max and min distance between keypoints
    // distance = similarity between descriptors, less distance -> more similar
    double max_dist = 0;
    double min_dist = 100;

    for (const auto& m : matches) {
        double dist = m.distance;

        if (dist < min_dist) {
            min_dist = dist;
        }

        if (dist > max_dist) {
            max_dist = dist;
        }
    }

    // Get all valid matches
    // If distane is <= min_dist * 1.5 its valid
    // Value can be changed, higher value means more keypoints 

    for (const auto& m : matches) {
        if (m.distance <= 1.5 * min_dist) {

            // Matches variable holds the index values of x-y positions of the keypoints in both images.
            // queryIdx gives key poinst index which has a match and trainIdx gives its corresponding matched key
            // These index values can then be used to find the key points in the key points arrays.

            good_point1.push_back(lastFrameKeypoints1.at(m.queryIdx).pt);
            good_point2.push_back(lastFrameKeypoints2.at(m.trainIdx).pt);
        }
    }

    cv::Rect croppImg1(0, 0, image1.cols, image1.rows);
    cv::Rect croppImg2(0, 0, image2.cols, image2.rows);

    // find minimum horizontal value for image 1 to crop
    //e.g. img1 size = 200 first keypoint having match found at position 100 crop img1 to 0-100
    // crop image2 to from corresponding x value to the width. 
    //e.g. img2 width 200 point found at 50 crop image  50-200

    // movementDirection tells us if the two images are aligned or not
    // if not, adjust them accordingly
    int imgWidth = image1.cols;
    for (int i = 0; i < good_point1.size(); ++i)
    {
        if (good_point1[i].x < imgWidth)
        {
            croppImg1.width = good_point1.at(i).x;
            croppImg2.x = good_point2[i].x;
            croppImg2.width = image2.cols - croppImg2.x;
            movementDirection = good_point1[i].y - good_point2[i].y;
            imgWidth = good_point1[i].x;
        }
    }
    

}