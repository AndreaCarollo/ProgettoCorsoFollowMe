#include "./person.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>



#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace cv;
using namespace std;

class person
{
private:
    /* data */
    cv::Rect boundingBox;

    Point2i point2D;
    Point3i point3D;
    vector<int> position2D;
    vector<int> position3D;

    cv::Rect ir_boundingBox;
    Point2i ir_point2D; // da ricavare con la trasformazione vista stereo
    vector<int> ir_position2D;
    vector<int> ir_position3D;

    int profondità;
    vector<vector<int>> pos2D_story;
    vector<vector<int>> pos3D_story;

    // for histogram
    Mat * b_hist;
    Mat * g_hist;
    Mat * r_hist;

public:
    person(cv::Rect ROI);
    void update(Rect new_ROI);
    void update_story();

    void get_ir_pos();     // trasforma 2d rgb in 2d ir
    void get_profondità(); // legge pcl

    void calc_hist(Mat *frame);

    ~person();
};

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

void person::update(Rect new_ROI)
{
    /* update story + update ROI and positions*/
    update_story();
    boundingBox = new_ROI;
    point2D = (new_ROI.br() + new_ROI.tl()) * 0.5;
    position2D = vector<int>{point2D.x, point2D.y};
}

void person::update_story()
{
    pos2D_story.push_back(position2D);
    pos3D_story.push_back(position3D);
}

void person::get_profondità() // passare pcl, eventuale transform matrix
{
    /* prelevare profondità da pcl */

    /* salvare profondità su vettore 3D */
}

void person::calc_hist(Mat *frame)
{
    int histSize = 256;
    float range[] = {0, 256}; //the upper boundary is exclusive
    const float *histRange = {range};
    bool uniform = true, accumulate = false;
    calcHist(&frame[0], 1, 0, Mat(), *b_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&frame[1], 1, 0, Mat(), *g_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&frame[2], 1, 0, Mat(), *r_hist, 1, &histSize, &histRange, uniform, accumulate);

}

void detection_on_frame(Mat *frame, vector<Rect> *peoples)
{
    /* code */
    /* do detection of the peoples on the frame */
    
}
