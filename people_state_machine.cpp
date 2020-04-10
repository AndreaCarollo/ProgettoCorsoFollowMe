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

// ---------------------------------
// ---- STRUTTURA TARGET -----------

struct Target
{
    cv::Rect starting_BBOx;
    cv::Rect boundingBox;
    cv::Point2i point2D;
    cv::Point3i point3D;
    std::vector<int> position2D;
    std::vector<int> position3D;
    cv::Rect ir_boundingBox;
    cv::Point2i ir_point2D; // TO DO: da ricavare con la trasformazione vista stereo
    std::vector<int> ir_position2D;
    std::vector<int> ir_position3D;

    int depth;
    std::vector<cv::Point2i> pos2D_story;
    //std::vector<std::vector<int>> pos2D_story;
    // std::vector<std::vector<int>> pos3D_story;

    // for histogram
    cv::MatND histogram;

    void target_update(Mat frame, cv::MultiTracker *trackers, int flag_hist)
    {
        boundingBox = trackers->getObjects()[0];
        if (flag_hist == 1)
            histogram = evaluate_hist(frame, boundingBox);
        point2D = (boundingBox.tl() + boundingBox.br()) / 2;
        pos2D_story.push_back(point2D);

        // TODO: add calculation of depth
        // depth = evaluatedepth();
    }
};

// --------------------------------------
// ---- STATE MACHINE DEFINITION --------

enum StateMachine
{
    DETECT,    // continue detecting if find peoples on the frame
               // + policy of changing detector
               // + select target
               // + initialization of tracker
    TRACK,     // provide tracking of the target
    LOST_TRACK // if lost use policy of re-id:
               // - histogram
               // - near location

};

enum DetectMachine
{
    FULLBODY,
    PEDESTRIAN,
    UPPERBODY
};

// ----------------------------------------------------------------------------------------------
// ---- MAIN ---- MAIN ---- MAIN ---- MAIN ---- MAIN ---- MAIN ---- MAIN ---- MAIN ---- MAIN ----
// ----------------------------------------------------------------------------------------------

int main()
{
    // ---- Import Video ----    //// TO DO: convert into a realsense video streaming

    /* from video */
    // VideoCapture cap(0);
    VideoCapture cap("../../../Dataset/Markers/vid2.mp4");
    // VideoCapture cap("../../../Dataset/Our_Video/QR_test3.mp4");

    /* from ir camera */
    // VideoCapture cap("../../../Dataset/IR_renamed/test01_04b/%3d.png");

    namedWindow("Video", WINDOW_KEEPRATIO);
    resizeWindow("Video", 960, 540);
    // Control open video
    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    // ---- Frames & parameters ----
    Mat frame;
    Mat portion_frame;
    vector<Rect> ROIs;
    Rect2d ROI;

    // ---- Create TARGET ----
    Target target;

    // ---- Detector initialization ---
    String haar_fullbody = "../trained//haarcascade_fullbody.xml";
    String haar_pedestrian = "../trained/haarcascade_pedestrian.xml";
    String haar_upbody = "../trained/haarcascade_upperbody.xml";

    // ---- Create classifiers ----
    CascadeClassifier fullbody_cascade;
    fullbody_cascade.load(haar_fullbody);

    CascadeClassifier pedestrian_cascade;
    pedestrian_cascade.load(haar_pedestrian);

    CascadeClassifier upbody_cascade;
    upbody_cascade.load(haar_upbody);

    // ---- Classifier Types & Flags ----
    string classifierTypes[3] = {"PEDESTRIAN", "FULBODY", "UPPERBODY"};
    string classifierType = classifierTypes[0];
    int classifier_counter = 0;
    int max_frame_try = 5;

    // ArUco marker dictionary and paramters
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
    Ptr<aruco::DetectorParameters> param = aruco::DetectorParameters::create();
    // *** these parameters can be tuned ***

    // Define the user ID marker (the one in the videos is 25)
    int user_ID = 25;

    // ---- Tracker Initialization ---- TO DO: can be put in a function

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

    // some initialization of parameters for state machine
    bool *flag_find = new bool;
    bool *flag_mark = new bool;
    bool flag_lost = false;
    bool flag_init_track = true;

    int tracker_counter = 0;
    int max_frame_lost = 25;

    Point2d centre_bbox;
    int h_SBBox, w_SBBox;
    int h_TAR, w_TAR;
    float th_SBBox_min = 0.7;
    float th_SBBox_max = 1.2;

    int count_hist = 0;
    int refresh_hist = 4;

    // Initial State is DETECT, stay here until find someone to track.
    StateMachine currentState = DETECT;

    // -----------------------------
    // ---- START STATE MACHINE ----
    // -----------------------------
    while (true)
    {
        cap >> frame;
        switch (currentState)
        {
        case DETECT:
            cout << "DETECT" << endl;
            /* code */
            // cascades used for tuning values
            // pedestrian_cascade.detectMultiScale(frame, ROIs, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            // fullbody_cascade.detectMultiScale(frame, ROIs, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
            // upbody_cascade.detectMultiScale(frame, ROIs, 1.5, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

            // TO DO: for the next big IF maybe put inside a function in lib
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
                *flag_find = false;
                int thr_euclidean = 50;
                int min = remove_ROIs(frame, ROIs, thr_euclidean, flag_find);

                // Check if it is the user through ArUco marker
                detect_aruco(frame, dict, param, ROIs[min], flag_mark);

                if (*flag_find == true && *flag_mark == true)
                {
                    cout << "fuond target" << endl;
                    target.starting_BBOx = ROIs[min];

                    // initialize the tracker
                    trackers.clear();
                    trackers.add(tracker, frame, target.starting_BBOx);
                    trackers.update(frame);

                    // save data into target
                    target.target_update(frame, &trackers, 1);
                    count_hist = 0;

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
            cout << "TRACK" << endl;

            /* code */
            // set to false the lost flag
            flag_lost = false;

            // Update tracker
            trackers.update(frame);
            if (count_hist == refresh_hist && tracker_counter != max_frame_lost)
            {
                target.target_update(frame, &trackers, 1);
                count_hist = 0;
            }
            else
            {
                target.target_update(frame, &trackers, 0);
            }

            // TO DO: check if lost target
            /* code */
            // centre_bbox = (target_bbox.br() - target_bbox.tl()) / 2;
            w_TAR = target.boundingBox.width;
            h_TAR = target.boundingBox.height;
            w_SBBox = target.starting_BBOx.width;
            h_SBBox = target.starting_BBOx.height;

            // ---- Check dimension of tracked box ----
            if (tracker_counter == max_frame_lost)
            {
                if ((w_TAR <= th_SBBox_min * w_SBBox) || (w_TAR >= th_SBBox_max * w_SBBox) ||
                    (h_TAR <= th_SBBox_min * h_SBBox) || (h_TAR >= th_SBBox_max * h_SBBox))
                {
                    flag_lost = true;
                }
                tracker_counter = 0;
            }

            if (flag_lost == false)
            {
                tracker_counter++;
                count_hist++;

                currentState = TRACK;
                cout << ">> stay on TRACK" << endl;

                // --> state machine: robot moving + avoid ostacles
                // ...
            }
            else
            {
                cout << ">> go to LOST_TRACK" << endl;
                tracker_counter = 0;
                currentState = LOST_TRACK;
            }

            break;

        case LOST_TRACK:
            cout << "LOST_TRACK" << endl;
            // TO DO: need to find back the target person on the frame
            /* IDEA:
            - use last working detector
            - detection aroud the area where lost -> take centroid of 5 frame before lost flag on
            - compare new detection hist with previous target hist
            */

            //--------------------------------------
            //---- Extract the portion of frame ----

            /* code */
            // TODO:
            // Mat portion_frame = crop the frame ()

            //---- Detection ------------
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

            if (!(ROIs.empty()))
            {
                // -- do comparison hist
                vector<double> compare_hist;
                for (int i = 0; i < ROIs.size(); i++)
                {
                    compare_hist.push_back(comparison_hist(frame, target.histogram, ROIs[i]));
                }
                int maxElementIndex = std::max_element(compare_hist.begin(), compare_hist.end()) - compare_hist.begin();
                int minElementIndex = std::min_element(compare_hist.begin(), compare_hist.end()) - compare_hist.begin();
                Rect2d New_ROI = ROIs[maxElementIndex];

                // -- update tracker & target, go to TRACK
                target.starting_BBOx = New_ROI;
                trackers.clear();
                trackers.add(tracker, frame, target.starting_BBOx);
                trackers.update(frame);
                target.target_update(frame, &trackers, 1);
                count_hist = 0;
                currentState = TRACK;
            }
            else
            {
                // if fallisco il rematching -> go to DETECT
                currentState = DETECT;
            }

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
            cv::rectangle(frame, target.boundingBox, Scalar(0, 0, 255), 1, 8, 0);
        }
        cv::imshow("Video", frame);
        if (waitKey(1) == 27)
        {
            return 0;
        }
    }
}

// --------------------------------------
// ----------- Functions ----------------
//
