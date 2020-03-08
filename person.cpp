#include "person.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace cv;
using namespace std;


person::person()
{
    //boundingBox = ROI;
    //point2D = (ROI.br() + ROI.tl()) * 0.5;
    //position2D = vector<int>{point2D.x, point2D.y};
    // position3D = vector<int>{point2D.x, point2D.y, profondità};
}

person::~person()
{
}

void person::update(cv::Rect new_ROI)
{
    /* update story + update ROI and positions*/
    update_story();
    boundingBox = new_ROI;
    point2D = (new_ROI.br() + new_ROI.tl()) * 0.5;
    position2D = vector<int>{point2D.x, point2D.y};
    // TO DO: update also the hist
}

void person::update_story()
{
    pos2D_story.push_back(position2D);
    pos3D_story.push_back(position3D);
}

void person::get_proof() // passare pcl, eventuale transform matrix
{
    /* prelevare profondità da pcl */

    /* salvare profondità su vettore 3D */
}

void person::calc_hist(cv::Mat *frame)
{
    int histSize = 256;
    float range[] = {0, 256}; //the upper boundary is exclusive
    const float *histRange = {range};
    bool uniform = true, accumulate = false;
    calcHist(&frame[0], 1, 0, cv::Mat(), *b_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&frame[1], 1, 0, cv::Mat(), *g_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&frame[2], 1, 0, cv::Mat(), *r_hist, 1, &histSize, &histRange, uniform, accumulate);
}

void detection_on_frame(cv::Mat *frame, vector<Rect> *peoples)
{
    /* code */
    /* do detection of the peoples on the frame */
}

// Take into consideration only the ROI near the center of the camera
void person::remove_ROIs(cv::Point center, std::vector<cv::Rect> ROIs, double thr)
{
    std::vector<double> dist;
    for(int i = 0; i < ROIs.size(); i++){
        dist.push_back(eucledian_norm(center, (ROIs[i].br() + ROIs[i].tl()) * 0.5));
    }
    int min = min_element(dist.begin(), dist.end()) - dist.begin();
    if(dist[min] < thr)
        boundingBox = ROIs[min];
    else
        printf("No ROI satisfy the threshold\n");
}

// Function for computing the distance between two points on a plane
float person::eucledian_norm(cv::Point p1, cv::Point p2)
{
    return sqrt( (float)(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)) );
}

// Find the target with the QR code
void person::QR_code()
{
    
}