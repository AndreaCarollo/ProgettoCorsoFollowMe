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

enum StateMachine
{
    DETECT,        // continue detecting if find peoples on the frame
                   // + policy of changing detector
    SELECT_TARGET, // given the target on the detect, look for them if one of them are the target to follow (smile/QR/ect)
    TRACK,         // provide tracking of the target
    LOST_TRACK     // get here if lost the tracking or if need to re-initialize tracker

};

enum DetectMachine
{
    FULLBODY,
    PEDESTRIAN,
    UPPERBODY
};

struct Target
{
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
    std::vector<std::vector<int>> pos2D_story;
    std::vector<std::vector<int>> pos3D_story;

    // for histogram
    cv::Mat *b_hist;
    cv::Mat *g_hist;
    cv::Mat *r_hist;
};

// ----------------------------------------------------------------------------
// Main
//

int main()
{
    // ---- Import Video ----
    //TO DO: convert into a realsense video streaming
    // VideoCapture cap(0);
    VideoCapture cap("../../../Dataset/Our_Video/test3.mp4");
    // VideoCapture cap("../../../Dataset/Our_Video/QR_test3.mp4");

    // VideoCapture cap(selected_cap);

    namedWindow("Video", WINDOW_KEEPRATIO);
    resizeWindow("Video", 960, 540);
    //Control open video
    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    // ---- Frames & parameters ----
    Mat frame;
    vector<Rect> ROIs;
    Rect2d ROI;

    // ---- Detector initialization ---
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
    string classifierTypes[3] = {"PEDESTRIAN", "FULBODY", "UPPERBODY"};
    string classifierType = classifierTypes[0];
    int classifier_counter = 0;
    int max_frame_try = 5;

    // ---- Tracker Initialization ----

    //TO DO: la selezione del tracker può esser messa in una funzione in lib

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

    // Initial State is DETECT, stay here until find someone to track.
    StateMachine currentState = DETECT;

    Rect target_bbox;
    bool flag_init_track = true;

    while (true)
    {
        cap >> frame;
        switch (currentState)
        {
        case DETECT:
            /* code */
            // cadscedes uses for tuning values
            // pedestrian_cascade.detectMultiScale(frame, ROIs, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            // fullbody_cascade.detectMultiScale(frame, ROIs, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            // upbody_cascade.detectMultiScale(frame, ROIs, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

            //TO DO: for the next big IF maybe to put inside a function in lib
            if (classifierType == classifierTypes[0])
            {
                pedestrian_cascade.detectMultiScale(frame, ROIs, 1.4, 28, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
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
                fullbody_cascade.detectMultiScale(frame, ROIs, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
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
                upbody_cascade.detectMultiScale(frame, ROIs, 1.3, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
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

            if (ROIs.empty())
            {
                //Keep State
                currentState = DETECT;
            }
            else
            {
                // ---- Select Target ----
                /* code */
                Point2d center;
                center.x = frame.cols / 2;
                center.y = frame.rows / 2;
                int thr_euclidean = 50;
                std::vector<double> dist;
                for (int i = 0; i < ROIs.size(); i++)
                {
                    dist.push_back(eucledian_norm(center, (ROIs[i].br() + ROIs[i].tl()) * 0.5));
                }
                int min = min_element(dist.begin(), dist.end()) - dist.begin();
                if (dist[min] < thr_euclidean)
                {
                    target_bbox = ROIs[min];
                    flag_init_track = true;
                    currentState = TRACK;
                }
                else
                {
                    currentState = DETECT;
                }
            }
            break;
        case TRACK:
            /* code */
            // bool flag_lost = false;
            if (flag_init_track == true)
            {
                trackers.clear();
                trackers.add(tracker, frame, target_bbox);
                trackers.update(frame);

                target_bbox = trackers.getObjects()[0];
                
            }
            else
            {
                trackers.update(frame);
                target_bbox = trackers.getObjects()[0];
            }

            // TO DO: check if lost target
            /* code */
            currentState = TRACK;
            flag_init_track = false;
            // if (flag_lost == false)
            // {
            //     currentState = TRACK;
            //     flag_init_track = false;
            // }
            // else
            // {
            //     currentState = DETECT;
            //     flag_init_track = true;
            // }
            break;
        case LOST_TRACK:
            // TO DO: need to find back the target person on the frame
            /* code */
            break;
        default:
            currentState = DETECT;
            break;
        }

        // --- Show Stuff
        /* code */
        if (!(ROIs.empty()))
        {
            for (int j = 0; j < ROIs.size(); j++)
            {
                cv::rectangle(frame, ROIs[j], Scalar(255, 0, 0), 3, 8, 0);
            }
            cv::rectangle(frame, target_bbox, Scalar(0, 0, 255), 1, 8, 0);
        }
        cv::imshow("Video", frame);
        if (waitKey(1) == 27)
        {
            return 0;
        }
    }
}

// -------------------------------------------------------------------
// Functions
//
