#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;

int main() {
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
}