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

    cv::Ptr<cv::DescriptorMatcher> descriptor_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);

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
    descriptor_matcher->match(lastFrameDescriptors1, lastFrameDescriptors2, matches);

    std::vector<cv::Point2d> first_good_point, second_good_point;
    first_good_point.reserve(matches.size());
    second_good_point.reserve(matches.size());

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

    for (const auto& current_match : matches) {
        if (current_match.distance <= 1.5 * min_dist) {

            // Matches variable holds the index values of x-y positions of the keypoints in both images.
            // queryIdx gives key poinst index which has a match and trainIdx gives its corresponding matched key
            // These index values can then be used to find the key points in the key points arrays.

            first_good_point.push_back(lastFrameKeypoints1.at(current_match.queryIdx).pt);
            second_good_point.push_back(lastFrameKeypoints2.at(current_match.trainIdx).pt);
        }
    }

     // Stitching
    // To stitch we are cropping the images with the unwanted or repeating regions

    cv::Rect cropped_image1(0, 0, image1.cols, image1.rows);
    cv::Rect cropped_image2(0, 0, image2.cols, image2.rows);

    // find minimum horizontal value for image 1 to crop
    //e.g. img1 size = 200 first keypoint having match found at position 100 crop img1 to pixel 0 to 100, i.e. crop everything above
    // crop image2 to from corresponding x value to the width. Same procedure
    //e.g. img2 width 200 point found at 50 crop image  50-200, so cut everything to the left

    // movementDirection tells us if the two images are aligned or not
    // if not, adjust them accordingly
    int imgWidth = image1.cols;
    for (int i = 0; i < first_good_point.size(); ++i)
    {
        if (first_good_point[i].x < imgWidth)
        {
            // Crop image at minimum horizontal point where matched keypoint is detected
            cropped_image1.width = first_good_point.at(i).x;
            cropped_image2.x = second_good_point[i].x;
            cropped_image2.width = image2.cols - cropped_image2.x;
            movementDirection = first_good_point[i].y - second_good_point[i].y;
            imgWidth = first_good_point[i].x;
        }
    }

    image1 = image1(cropped_image1);
    image2 = image2(cropped_image2);
    
    int maxHeight;
    int maxWidth;

    if (image1.rows > image2.rows) {
        maxHeight = image1.rows;
    } else {
        maxHeight = image2.rows;
    }

    int maxWidth = image1.cols + image2.cols;

    cv::Mat result = cv::Mat::zeros(cv::Size(maxWidth, maxHeight + abs(movementDirection)), CV_8UC3);
    if (movementDirection > 0)
    {
        cv::Mat half1(result, cv::Rect(0, 0, image1.cols, image1.rows));
        image1.copyTo(half1);
        cv::Mat half2(result, cv::Rect(image1.cols, abs(movementDirection),image2.cols, image2.rows));
        image2.copyTo(half2);
    }
    else
    {
        cv::Mat half1(result, cv::Rect(0, abs(movementDirection), image1.cols, image1.rows));
        image1.copyTo(half1);
        cv::Mat half2(result, cv::Rect(image1.cols,0 ,image2.cols, image2.rows));
        image2.copyTo(half2);
    }
    cv::imshow("Stitched Image", result);

    int k = cv::waitKey(0);
    if (k == 's') {
        imwrite("StitchedImage.png", result);
    }
    return 0;
}