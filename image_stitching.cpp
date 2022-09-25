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

  

}