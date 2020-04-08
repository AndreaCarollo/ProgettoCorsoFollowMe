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

    // ir camera
    // VideoCapture cap("../../../Dataset/IR_renamed/test01_04b/%3d.png");

    // VideoCapture cap(selected_cap);

    namedWindow("Video", WINDOW_KEEPRATIO);
    // resizeWindow("Video", 960, 540);
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
    String haar_fullbody   = "../trained//haarcascade_fullbody.xml";
    String haar_pedestrian = "../trained/haarcascade_pedestrian.xml";
    String haar_upbody     = "../trained/haarcascade_upperbody.xml";

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
    int max_frame_try = 1;

    // --------------------------------
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

    // some initialization of parameters for state machine
    Rect target_bbox;
    bool flag_lost = false;
    bool flag_init_track = true;

    Point2d centre_bbox;
    int h_ROI, w_ROI;
    int h_TAR, w_TAR;
    float th_ROI_min = 0.6;
    float th_ROI_max = 1.3;

    while (true)
    {
        cap >> frame;
        switch (currentState)
        {
        case DETECT:
            // cout << "DETECT \n"
            //      << endl;
            /* code */
            // cadscedes uses for tuning values
            // pedestrian_cascade.detectMultiScale(frame, ROIs, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            // fullbody_cascade.detectMultiScale(frame, ROIs, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            // upbody_cascade.detectMultiScale(frame, ROIs, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

            //TO DO: for the next big IF maybe to put inside a function in lib
            // Detection on frame
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
                    w_ROI = ROIs[min].width;
                    h_ROI = ROIs[min].height;

                    // initialize the tracker
                    trackers.clear();
                    trackers.add(tracker, frame, target_bbox);
                    trackers.update(frame);

                    // target_bbox = trackers.getObjects()[0];
                    // go to TRACK
                    currentState = TRACK;
                }
                else
                {
                    // keep DETECT
                    currentState = DETECT;
                }
            }
            break;
        case TRACK:
            // cout << "TRACK \n" << endl;

            /* code */
            // set to false the lost flag
            flag_lost = false;

            // Update tracker
            trackers.update(frame);
            target_bbox = trackers.getObjects()[0];

            // TO DO: check if lost target
            /* code */
            // flag_lost = person::islost();
            // centre_bbox = (target_bbox.br() - target_bbox.tl()) / 2;
            w_TAR = target_bbox.width;
            h_TAR = target_bbox.height;

            // if ( w_TAR < th_ROI_min * w_ROI or w_TAR > th_ROI_max * w_ROI or
            //      h_TAR < th_ROI_min * h_ROI or h_TAR > th_ROI_max * h_ROI  )
            if ( (w_TAR <= 0.5* w_ROI) * (w_TAR >= 1.5 * w_ROI) *
                 (h_TAR <= 0.5* h_ROI) * (h_TAR >= 1.5 * h_ROI) )
            {
                flag_lost = true;
            }

            if (flag_lost == false)
            {
                currentState = TRACK;
                // cout << "stay on TRACK" << endl;

                // --> state machine: robot moving + avoid ostacles
                // ...
            }
            else
            {
                // cout << "go to LOST_TRACK" << endl;
                currentState = LOST_TRACK;
            }

            break;

        case LOST_TRACK:
            // cout << "LOST_TRACK \n" << endl;
            // TO DO: need to find back the target person on the frame
            /* code */
            // use last working detector, and do detection
            // compare new detection hist with previous target hist

            // if fallisco il rematching -> go to DETECT
            currentState = DETECT;
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
        if (waitKey(1000) == 27)
        {
            return 0;
        }
    }
}

// -------------------------------------------------------------------
// Functions
//
