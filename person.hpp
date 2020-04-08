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
    person();
    void update(cv::Rect new_ROI);
    void update_story();

    void get_ir_pos();     // trasforma 2d rgb in 2d ir
    void get_proof();      // legge pcl

    void calc_hist(cv::Mat *ROI);

    void remove_ROIs(cv::Point center, std::vector<cv::Rect> ROIs, double thr);  // tiene solo la persona nel centro
    float eucledian_norm(cv::Point p1, cv::Point p2);

    void QR_code(cv::Mat frame);  // detect and decode the QR code
    void display_QR(cv::Mat frame, cv::Mat QR_bbox);  // display a bbox on QR code
    void verify_user(cv::Mat QR_bbox, std::string data);  // check the correct user

    ~person();
};

void detection_on_frame(cv::Mat *frame, std::vector<cv::Rect> *peoples);

float eucledian_norm(cv::Point p1, cv::Point p2);

cv::MatND evaluate_hist(cv::Mat frame, cv:: Rect2d ROI);

double comparison_hist(cv::Mat frame, cv::MatND hist_1, cv::Rect ROI2);

// void target_update(Target *target, Mat frame, cv::MultiTracker trackers);


// struct Target
// {
//     cv::Rect boundingBox;
//     cv::Point2i point2D;
//     cv::Point3i point3D;
//     std::vector<int> position2D;
//     std::vector<int> position3D;
//     cv::Rect ir_boundingBox;
//     cv::Point2i ir_point2D; // TO DO: da ricavare con la trasformazione vista stereo
//     std::vector<int> ir_position2D;
//     std::vector<int> ir_position3D;
    

//     int depth;
//     std::vector<cv::Point2i> pos2D_story;
//     //std::vector<std::vector<int>> pos2D_story;
//     // std::vector<std::vector<int>> pos3D_story;

//     // for histogram
//     cv::MatND histogram;
// };