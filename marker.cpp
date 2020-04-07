#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>    // for the ArUco Markers

#include "./person.hpp"

#include <ctime>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cfloat>
#include <cmath>

using namespace cv;
using namespace std;


int main()
{

    // Marker creation 
    Mat markerImg1, markerImg2, markerImg3;
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
    aruco::drawMarker(dict, 25, 200, markerImg1, 1);
    aruco::drawMarker(dict, 30, 200, markerImg2, 1);
    aruco::drawMarker(dict, 35, 200, markerImg3, 1);
    imwrite("mark1.png", markerImg1);
    imwrite("mark2.png", markerImg2);
    imwrite("mark3.png", markerImg3);

    // Show the input image
    Mat inImg = imread("../../../Dataset/Markers/img14.jpg");
    namedWindow("Original Img", WINDOW_NORMAL);
    resizeWindow("Original Img", 800, 600);
    Mat inImg = imread("../marker_example.png");
    imshow("Original Img", inImg); 

    // Marker detection
    vector<int> markerIDs;
    vector< vector<Point2f> > markerCorn, rejCandidates;
    Ptr<aruco::DetectorParameters> param = aruco::DetectorParameters::create();
    // ** here we can modify the dafault parameters **
    aruco::detectMarkers(inImg, dict, markerCorn, markerIDs, param, rejCandidates);

    // Show the result
    Mat outImg = inImg.clone();
    aruco::drawDetectedMarkers(outImg, markerCorn, markerIDs);

    namedWindow("Final Img", WINDOW_NORMAL);
    resizeWindow("Final Img", 800, 600);
    imshow("Final Img", outImg);

    waitKey(0);
}