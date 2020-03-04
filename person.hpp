#ifdef PERSONLIBRARY
#define PERSONLIBRARY
#else
#define PERSONLIBRARY
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>


// LIBRARY FUNCTIONS

class person
{
private:

public:
    /* data */
    cv::Rect boundingBox;

    cv::Point2i point2D;
    cv::Point3i point3D;
    std::vector<int> position2D;
    std::vector<int> position3D;

    cv::Rect ir_boundingBox;
    cv::Point2i ir_point2D; // TO DO: da ricavare con la trasformazione vista stereo
    std::vector<int> ir_position2D;
    std::vector<int> ir_position3D;

    int proof;
    std::vector< std::vector<int>> pos2D_story;
    std::vector< std::vector<int>> pos3D_story;

    // for histogram
    cv::Mat *b_hist;
    cv::Mat *g_hist;
    cv::Mat *r_hist;

    /* functions */
    person(cv::Rect ROI);
    void update(cv::Rect new_ROI);
    void update_story();

    void get_ir_pos();     // trasforma 2d rgb in 2d ir
    void get_proof();      // legge pcl

    void calc_hist(cv::Mat *ROI);

    ~person();
};

void detection_on_frame(cv::Mat *frame, std::vector<cv::Rect> *peoples);