#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

// Functions declaration
void draw_epilines(Mat img1, Mat img2, cv::Vec3f line, Point2f point1, Point2f point2);

int main()
{
    // STEP 1 --> COMPUTE THE KEYPOINTS

    // Read the image
    Mat leftImg  = imread("../../../Dataset/Epipolar/left1.jpg");
    Mat rightImg = imread("../../../Dataset/Epipolar/right1.jpg");
    // Check
    if( leftImg.empty() || rightImg.empty() ){
        printf("Could not open or find the images\n");
        return -1;
    }
    
    // Keypoints and descriptor variables
    std::vector<KeyPoint> key_left, key_right;
    Mat desc_left, desc_right;

    // Initialize ORB detector
    Ptr<ORB> orb_detector = ORB::create();

    // Find the keypoints and descriptors with ORB
    orb_detector->detectAndCompute(leftImg, noArray(), key_left, desc_left);
    orb_detector->detectAndCompute(rightImg, noArray(), key_right, desc_right);

    // Output image with keypoints
    Mat outImg = leftImg.clone();
    drawKeypoints(leftImg, key_left, outImg);
    imshow("Original Img", leftImg);
    imshow("Img with keypoints", outImg);
    waitKey(0);
    destroyAllWindows();

    // STEP 2 --> COMPUTE MATCHING

    // Initialize the matcher (Brute Force)
    BFMatcher matcher(NORM_HAMMING);
    std::vector< DMatch > matches;

    // Find the matches btw keypoints of the two images
    matcher.match(desc_left, desc_right, matches);
    
    // Consider only the best matching
    std::vector< DMatch > good_matches;
    for(int i = 0; i < desc_left.rows; i++){
        if(matches[i].distance < 40){
            good_matches.push_back(matches[i]);
        }
    }

    //for(int j = 0; j < good_matches.size(); j++){
    //    printf("%f\n", good_matches[j].distance);
    //}

    printf("Number of good matches: %lu\n", good_matches.size());

    // Visualize the matching
    Mat matchesImg;
    drawMatches(leftImg, key_left, rightImg, key_right, good_matches, matchesImg);
    imshow("Matching btw images", matchesImg);
    waitKey(0);
    destroyAllWindows();

    // STEP 3 --> FIND THE FUNDAMENTAL MATRIX

    // Get the keypoints from the good matching
    std::vector<Point2f> pts1, pts2;
    for(int j = 0; j < good_matches.size(); j++){
        pts1.push_back(key_left[good_matches[j].queryIdx].pt);
        pts2.push_back(key_right[good_matches[j].trainIdx].pt);
    }

    // Compute the fundamental matrix
    Mat F = findFundamentalMat(pts1, pts2, FM_RANSAC);
    printf("Dimensions of the fundamental matrix: %d x %d\n", F.rows, F.cols);  // check
    // Print of the matrix
    printf("Fundamental matrix F:\n");
    for(int i = 0; i < F.rows; i++){
        for(int j = 0; j < F.cols; j++){
            printf("%f\t", F.at<double>(i,j));
        }
        printf("\n");
    }

    // Compute epipolar line
    std::vector< cv::Vec3f > lines1, lines2;
    computeCorrespondEpilines(pts1, 1, F, lines1);
    computeCorrespondEpilines(pts2, 2, F, lines2);

    // Draw the epipolar lines
    for(int i = 0; i < lines1.size(); i++){
        draw_epilines(leftImg, rightImg, lines1[i], pts1[i], pts2[i]);
        draw_epilines(rightImg, leftImg, lines2[i], pts2[i], pts1[i]);
    }
    imshow("Epilines on left image", leftImg);
    imshow("Epilines on right image", rightImg);
    waitKey(0);

    return 0;
}

// Function for drawing the epipolar lines
void draw_epilines(Mat img1, Mat img2, cv::Vec3f line, Point2f point1, Point2f point2){
    Scalar color(rand() % 256, rand() % 256, rand() % 256);

    // Draw the line in the other image
    cv::line(img2, cv::Point(0, -line[2]/line[0]), cv::Point(img1.cols, -(line[2]+line[1]*img1.cols)/line[1]), color);
    cv::circle(img1, point1, 3, color, -1);
    cv::circle(img2, point2, 3, color, -1);
}