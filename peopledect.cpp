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
    VideoCapture cap("../../../Dataset/Our_Video/test2.mp4");
    // VideoCapture cap(0);
    namedWindow("Video", WINDOW_KEEPRATIO);
    resizeWindow("Video", 960, 540);

    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    Mat frame, frame_gray;
    vector<Rect> ROIs;
    Rect2d ROI;

    /*** Detector initialization ***/
    String haar_fullbody = "../trained//haarcascade_fullbody.xml";
    String haar_pedestrian = "../trained/haarcascade_pedestrian.xml";
    String haar_upbody = "../trained/haarcascade_upperbody.xml";

    // Create classifiers
    CascadeClassifier fullbody_cascade;
    fullbody_cascade.load(haar_fullbody);

    CascadeClassifier pedestrian_cascade;
    pedestrian_cascade.load(haar_pedestrian);

    CascadeClassifier upbody_cascade;
    upbody_cascade.load(haar_upbody);

    // Classifier Types & Flags
    string classifierTypes[3] = {"FULBODY", "PEDESTRIAN", "UPPERBODY"};
    string classifierType = classifierTypes[0];
    int classifier_counter = 0;
    int max_frame_try = 5;

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
            vector<Rect> ROIs;

            if (classifierType == classifierTypes[0])
            {
                fullbody_cascade.detectMultiScale(frame, ROIs, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
                if (ROIs.empty() & classifier_counter == max_frame_try)
                {
                    classifierType = classifierTypes[1];
                    classifier_counter = 0;
                }
                else
                {
                    classifier_counter++;
                }
            }
            else if (classifierType == classifierTypes[1])
            {
                pedestrian_cascade.detectMultiScale(frame, ROIs, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
                if (ROIs.empty() & classifier_counter == max_frame_try)
                {
                    classifierType = classifierTypes[2];
                    classifier_counter = 0;
                }
                else
                {
                    classifier_counter++;
                }
            }
            else if (classifierType == classifierTypes[2])
            {
                upbody_cascade.detectMultiScale(frame, ROIs, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
                if (ROIs.empty() & classifier_counter == max_frame_try)
                {
                    classifierType = classifierTypes[0];
                    classifier_counter = 0;
                }
                else
                {
                    classifier_counter++;
                }
            }

            // TO DO: function that given the ROIs, select the bounding-box to track
            /* code */

            // for the moment take the first ROI ...
            ROI = ROIs[0];
            // person target = person(ROI);
            trackers.clear();
            trackers.add(tracker, frame, ROI);
            trackers.update(frame);
            for (int j = 0; j < ROIs.size(); j++)
            {
                cv::rectangle(frame, ROIs[j], Scalar(0, 255, 0), 3, 8, 0);
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

            if (classifierType == classifierTypes[0])
            {
                fullbody_cascade.detectMultiScale(frame, ROIs, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            }
            else if (classifierType == classifierTypes[1])
            {
                pedestrian_cascade.detectMultiScale(frame, ROIs, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            }
            else if (classifierType == classifierTypes[2])
            {
                upbody_cascade.detectMultiScale(frame, ROIs, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            }
            // people_cascade.detectMultiScale(frame_gray, ROIs_full, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            // people_cascade.detectMultiScale(frame, ROIs_full, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            // upbody_cascade.detectMultiScale(frame, ROIs_up, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

            // display the results of detection
            for (int j = 0; j < ROIs.size(); j++)
            {
                cv::rectangle(frame, ROIs[j], Scalar(0, 255, 0), 3, 8, 0);
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