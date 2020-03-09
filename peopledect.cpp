#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "./MyLib.hpp"

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
    // VideoCapture cap("/home/andrea/Documents/Robotics/Dataset/Our_Video/test1.mp4");
    VideoCapture cap("../../../Dataset/Our_Video/test1.mp4");
    // VideoCapture cap(0);

    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    Mat frame, frame_gray;
    vector<Rect> ROIs;
    Rect2d ROI;

    /*** Detector initialization ***/
    // String haar_fullbody = "/home/andrea/opencv/sources/opencv/data/haarcascades/haarcascade_fullbody.xml";
    String haar_pedestrian = "../trained/haarcascade_pedestrian.xml";
    String haar_upbody = "../trained/haarcascade_upperbody.xml";

    // Create classifiers
    CascadeClassifier people_cascade;
    people_cascade.load(haar_pedestrian);

    CascadeClassifier upbody_cascade;
    upbody_cascade.load(haar_upbody);

    /*** Tracker initialization ***/
    // List of tracker types in OpenCV 3.4.1
    string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD",
                              "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    string trackerType = trackerTypes[0];
    MultiTracker trackers;
    Ptr<Tracker> tracker;

    // creation of the tracker selected
    if (trackerType == "BOOSTING")
        tracker = TrackerBoosting::create();
    if (trackerType == "MIL")
        tracker = TrackerMIL::create();
    if (trackerType == "KCF")
        tracker = TrackerKCF::create();
    if (trackerType == "TLD")
        tracker = TrackerTLD::create();
    if (trackerType == "MEDIANFLOW")
        tracker = TrackerMedianFlow::create();
    if (trackerType == "GOTURN")
        tracker = TrackerGOTURN::create();
    if (trackerType == "MOSSE")
        tracker = TrackerMOSSE::create();
    if (trackerType == "CSRT")
        tracker = TrackerCSRT::create();

    while (true)
    {
        int i = 0;
        i++;

        cap >> frame;
        if (i == 0 || ROIs.size() == 0)
        {
            vector<Rect> ROIs_full;
            vector<Rect> ROIs_up;

            people_cascade.detectMultiScale(frame, ROIs_full, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

            if (ROIs_full.empty())
            {
                upbody_cascade.detectMultiScale(frame, ROIs_up, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            }

            // TO DO: add a function to manage the two detection in order to understand if the upperbody dect is into the fulbody dect
            /* code */
            // for (int j = 0; j < ROIs_up.size(); j++)
            //     ROIs.push_back(ROIs_up[j]);

            bool flag_no_ROI = false;
            if (ROIs_full.empty() & !(ROIs_up.empty()))
            {
                ROIs = ROIs_up;
            }
            else if (!(ROIs_full.empty()) & ROIs_up.empty())
            {
                ROIs = ROIs_full;
            }
            else if (!(ROIs_full.empty()) & !(ROIs_up.empty()))
            {
                // TO DO: compare each rectangle..
                ROIs = compare_rect(ROIs_up, ROIs_full);
            }
            else
            {
                flag_no_ROI = true;
            }

            // if( !(flag_no_ROI) ){ }
            bool go_on = false;

            if (flag_no_ROI == false)
            {
                // TO DO: understand which rectangle is my target
                /* code */
                go_on = true;
            }

            if (go_on == true)
            {
                ROI = ROIs[0];
                // person target = person(ROI);
                trackers.clear();
                trackers.add(tracker, frame, ROI);
                trackers.update(frame);
                for (int j = 0; j < ROIs.size(); j++)
                {
                    cv::rectangle(frame, ROIs[j], Scalar(0, 255, 0), 3, 8, 0);
                }
            }

            cv::imshow("Video", frame);
            if (waitKey(1) == 27)
            {
                return 0;
            }
        }
        else
        {

            // apply calssifier to each frame
            vector<Rect> ROIs_full;
            vector<Rect> ROIs_up;

            /* face detection [method in the class] + scale handling
            * if use fullbody                1.05, 5
            * if use haarcascade pedestrian: 1.5 , 30
            * both min Size(50,50) 
            * */
            // people_cascade.detectMultiScale(frame_gray, ROIs_full, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            people_cascade.detectMultiScale(frame, ROIs_full, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

            if (ROIs_full.empty())
            {
                upbody_cascade.detectMultiScale(frame, ROIs_up, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            }

            for (int j = 0; j < ROIs_up.size(); j++)
                ROIs_full.push_back(ROIs_up[j]);

            // display the results of detection
            for (int j = 0; j < ROIs_full.size(); j++)
            {
                cv::rectangle(frame, ROIs_full[j], Scalar(0, 255, 0), 3, 8, 0);
            }

            // Update Tracker
            trackers.update(frame);

            // TO DO: Update Target
            // target.update(trackers.getObjects()[0]);
            // rectangle(frame, target.boundingBox, Scalar(0, 0, 255), 3, 8, 0);

            // TO DO: Update Robot Status

            cv::rectangle(frame, trackers.getObjects()[0], Scalar(255, 0, 0), 3, 8, 0);

            cv::imshow("Video", frame);
            if (waitKey(1) == 27)
            {
                return 0;
            }
        }
    }
    return 0;
}