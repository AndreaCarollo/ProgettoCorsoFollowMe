#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

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
    // TO DO: convert into a realsense video streaming
    // VideoCapture cap("../../../Dataset/Our_Video/test4.mp4");
    VideoCapture cap(0);

    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    Mat frame, frame_gray;
    vector<Rect> ROIs;
    Rect2d ROI;

    person user;
    Point frame_center;

    /*** Detector initialization ***/
    // String haar_fullbody = "/home/andrea/opencv/sources/opencv/data/haarcascades/haarcascade_fullbody.xml";
    String haar_pedestrian = "../trained/haarcascade_pedestrian.xml";
    String haar_upbody = "../trained/haarcascade_upperbody.xml";

    // Create classifiers
    CascadeClassifier people_cascade;
    people_cascade.load(haar_pedestrian);

    CascadeClassifier upbody_cascade;
    upbody_cascade.load(haar_upbody);

    while (true)
    {
        int i = 0;
        i++;

        if (i == 1)
            frame_center = Point (frame.cols*0.5, frame.rows*0.5);
        
        cap >> frame;

        people_cascade.detectMultiScale(frame, ROIs, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(150, 200));
        vector<Rect> ROIs_up;
        upbody_cascade.detectMultiScale(frame, ROIs_up, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(150, 200));

        for( int j = 0; j < ROIs_up.size(); j++)
            ROIs.push_back(ROIs_up[j]);

        // TO DO: add a function to manage the two detection in order to understand if the upperbody dect is into the fulbody dect
        
        
        if (ROIs.size() > 0)
        {
            user.remove_ROIs(frame_center, ROIs, 100.0);
        }

        /*
        for (int j = 0; j < ROIs.size(); j++)
        {
            rectangle(frame, ROIs[j], Scalar(0, 255, 0), 3, 8, 0);
        }
        */

        rectangle(frame, user.boundingBox, Scalar(0, 255, 0), 3, 8, 0);

        imshow("Video", frame);
        if (waitKey(1) == 27)
        {
            return 0;
        }
       
    }
    return 0;
}