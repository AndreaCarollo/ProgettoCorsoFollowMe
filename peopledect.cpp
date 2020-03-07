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
    VideoCapture sequence("/home/andrea/Documents/Robotics_FollowMe/Dataset/Our_Video/test4.mp4");
    // VideoCapture cap("/home/andrea/Documents/Robotics_FollowMe/Dataset/Our_Video/test1.mp4");

    if (!sequence.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    Mat frame, frame_gray;
    vector<Rect> ROIs;
    Rect2d ROI;

    // sequence >> frame;

    /*** Detector initialization ***/
    // String haar_fullbody = "/home/andrea/opencv/sources/opencv/data/haarcascades/haarcascade_fullbody.xml";
    String haar_pedestrian = "../trained/haarcascade_pedestrian.xml";
    CascadeClassifier people_cascade; // Class for the cascade classifier
    people_cascade.load(haar_pedestrian);

    /*** Tracker initialization ***/
    // List of tracker types in OpenCV 3.4.1
    string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD",
                              "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    string trackerType = trackerTypes[7];
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

    for (int i = 0; i < 1000; i++)
    {
        sequence >> frame;
        if (i == 0 || ROIs.size() == 0)
        {
            cvtColor(frame, frame_gray, COLOR_RGB2GRAY);
            equalizeHist(frame_gray, frame_gray);
            people_cascade.detectMultiScale(frame_gray, ROIs, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

            if (ROIs.size() > 1)
            {
                /* need to select only the biggest and closest to the center */
                // ROI = ROIs[0];
            }
            else if (ROIs.size() == 1)
            {
                ROI = ROIs[0];
            }

            // person target = person(ROI);
            trackers.clear();
            trackers.add(tracker, frame, ROI);
            trackers.update(frame);

            imshow("Video", frame);
            if (waitKey(1) == 27)
            {
                return 0;
            }
        }
        else
        {

            // apply calssifier to each frame
            vector<Rect> peoples;
            cvtColor(frame, frame_gray, COLOR_RGB2GRAY);
            // image equalization to increse the performances
            equalizeHist(frame_gray, frame_gray);

            // face detection [method in the class] + scale handling
            // if use fullbody                1.05, 5
            // if use haarcascade pedestrian: 1.5 , 30
            // both min Size(50,50)
            // people_cascade.detectMultiScale(frame_gray, peoples, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            people_cascade.detectMultiScale(frame_gray, peoples, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

            // display the results
            for (int j = 0; j < peoples.size(); j++)
            {
                rectangle(frame, peoples[j], Scalar(0, 255, 0), 3, 8, 0);
                // ellipse(frame, center, Size(peoples[j].width * 0.5, peoples[j].height * 0.5), 0, 0, 360, Scalar(255, 0, 0), 4, 8, 0);
            }
            trackers.update(frame);
            // target.update(trackers.getObjects()[0]);
            // rectangle(frame, target.boundingBox, Scalar(0, 0, 255), 3, 8, 0);

            rectangle(frame, trackers.getObjects()[0], Scalar(255, 0, 0), 3, 8, 0);

            imshow("Video", frame);
            if (waitKey(1) == 27)
            {
                return 0;
            }
        }
    }
    return 0;
}