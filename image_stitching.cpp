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


    

}