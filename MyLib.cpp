#include "MyLib.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace cv;
using namespace std;


person::person(cv::Rect ROI)
{
    boundingBox = ROI;
    point2D = (ROI.br() + ROI.tl()) * 0.5;
    position2D = vector<int>{point2D.x, point2D.y};
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
