#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "./person.hpp"

#include <chrono>
#include <ctime>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cfloat>
#include <cmath>
#include <numeric>

using namespace cv;
using namespace std;

// ---------------------------------
// ---- STRUTTURA TARGET -----------
// ---------------------------------

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
// --------------------------------------

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

// -----------------------------------
// ---- MAIN ---- MAIN ---- MAIN ----
// -----------------------------------

int main()
{
    // ---- Import Video ----    //// TO DO: convert into a realsense video streaming

    // acquisition video
    // VideoCapture cap(0);
    // VideoCapture cap("../../../Dataset/Markers/vid2.mp4");
    VideoCapture cap("../../../Dataset/Our_Video/test5_2.mp4"); //test3.mp4"); //

    namedWindow("Video", WINDOW_KEEPRATIO);
    resizeWindow("Video", 960, 540);
    // Control open video
    if (!cap.isOpened())
    {
        std::cout << "Could not read video file" << endl;
        return 1;
    }

    // ---- Frames & parameters ----
    Mat frame;
    Mat portion_frame;
    vector<Rect> ROIs, TMP_ROIs;
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
    string classifierType = classifierTypes[1];
    int classifier_counter = 0;
    int max_frame_try = 600000;

    // ArUco marker dictionary and paramters
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
    Ptr<aruco::DetectorParameters> param = aruco::DetectorParameters::create();
    // *** these parameters can be tuned ***

    // Define the user ID marker (the one in the videos is 25)
    int user_ID = 25;

    // ---- Tracker Initialization ---- TO DO: can be put in a function
    // string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD",
                            //   "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    // string trackerType = trackerTypes[7];
    MultiTracker trackers;
    Ptr<Tracker> tracker = TrackerCSRT::create();

    // creation of the tracker selected
    // if (trackerType == "BOOSTING")
    //     tracker = TrackerBoosting::create();
    // if (trackerType == "MIL")
    //     tracker = TrackerMIL::create();
    // if (trackerType == "KCF")
    //     tracker = TrackerKCF::create();
    // if (trackerType == "TLD")
    //     tracker = TrackerTLD::create();
    // if (trackerType == "MEDIANFLOW")
    //     tracker = TrackerMedianFlow::create();
    // if (trackerType == "GOTURN")
    //     tracker = TrackerGOTURN::create();
    // if (trackerType == "MOSSE")
    //     tracker = TrackerMOSSE::create();
    // if (trackerType == "CSRT")
    //     tracker = TrackerCSRT::create();

    // Some initialization of parameters for state machine
    bool *flag_find = new bool;
    bool *flag_mark = new bool;
    bool flag_lost = false;
    bool flag_init_track = true;

    int tracker_counter = 0;
    int counter_lost = 0;
    int max_frame_lost = 25;

    Point2d centre_bbox;
    int h_SBBox, w_SBBox;
    int h_TAR, w_TAR;
    float th_SBBox_min = 0.5;
    float th_SBBox_max = 1.5;

    int count_hist = 0;
    int refresh_hist = 30;

    Point2d tmp_delta;
    int delta = 0;
    int it_story;

    vector<float> overlap_areas;
    int index_pos;
    float av_pos_x = 0;
    float av_pos_y = 0;

    // Initial State is DETECT, stay here until find someone to track.
    StateMachine currentState = DETECT;
    StateMachine prevState = DETECT;

    // Other for Time Analysis
    std::vector<float> ts_det, ts_track, ts_visu;
    auto start = chrono::high_resolution_clock::now();
    auto end = chrono::high_resolution_clock::now();
    double time_taken = 0;

    // -----------------------------
    // ---- START STATE MACHINE ----
    // -----------------------------
    while ((cap >> frame).grab())
    {

        switch (currentState)
        {
        case DETECT:
            // std::cout << "DETECT" << endl;
            cv::putText(frame, "DETECT", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 2);

            // start chrono
            start = chrono::high_resolution_clock::now();

            // Detection on frame                                  // TODO: COnvert into a Switch
            if (classifierType == classifierTypes[0])
            {
                pedestrian_cascade.detectMultiScale(frame, ROIs, 1.4, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
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

            // end chrono
            end = chrono::high_resolution_clock::now();
            // eval time taken
            time_taken = chrono::duration_cast<chrono::microseconds>(end - start).count();
            ts_det.push_back((float)time_taken / 1000.0);

            if (ROIs.empty())
            {
                //Keep State
                currentState = DETECT;
                prevState = DETECT;
            }
            else
            {
                // ---- Select Target ----
                *flag_find = false;
                // usign euclidean distance from center
                int thr_euclidean = 100;

                for (int j = 0; j < ROIs.size(); j++)
                {
                    cv::rectangle(frame, ROIs[j], Scalar(255, 0, 0), 3, 8, 0);
                }

                int min = remove_ROIs(frame, ROIs, thr_euclidean, flag_find);

                // Check if it is the user through ArUco marker
                detect_aruco(frame, dict, param, ROIs[min], flag_mark);

                // into the if set && if use the marker
                if (*flag_find == true || *flag_mark == true)
                {
                    cout << "found target" << endl;
                    target.starting_BBOx = ROIs[min];

                    // initialize the tracker
                    tracker = TrackerCSRT::create();
                    trackers.clear();
                    trackers.add(tracker, frame, target.starting_BBOx);
                    trackers.update(frame);

                    // save data into target
                    target.target_update(frame, &trackers, 1);
                    count_hist = 0;

                    // go to TRACK
                    currentState = TRACK;
                    prevState = DETECT;
                }
                else
                {
                    // keep DETECT
                    currentState = DETECT;
                    prevState = DETECT;
                }
            }
            break;
        case TRACK:
            // cout << "TRACK" << endl;
            cv::putText(frame, "TRACK", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 2);

            // set to false the lost flag
            flag_lost = false;

            // start chrono
            start = chrono::high_resolution_clock::now();

            // Update tracker
            trackers.update(frame);

            // end chrono
            end = chrono::high_resolution_clock::now();
            // eval time taken
            time_taken = chrono::duration_cast<chrono::microseconds>(end - start).count();
            ts_track.push_back((float)time_taken / 1000.0);

            // Check if the tracker falls by jumping somewhere
            it_story = target.pos2D_story.size() - 1;
            if (it_story >= 3)
            {
                tmp_delta = (Point2d)(target.pos2D_story[it_story].x - target.pos2D_story[it_story - 1].x, target.pos2D_story[it_story].y - target.pos2D_story[it_story - 1].y);
                delta = std::sqrt(tmp_delta.x * tmp_delta.x + tmp_delta.y * tmp_delta.y);
                std::cout << " Delta: " << delta << endl;
                if (delta < 69)
                {

                    pedestrian_cascade.detectMultiScale(frame, TMP_ROIs, 1.4, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

                    if (!(TMP_ROIs.empty()))
                    {
                        for (int i = 0; i < TMP_ROIs.size(); i++)
                        {
                            overlap_areas.push_back((target.boundingBox & TMP_ROIs[i]).area());
                        }

                        if (*max_element(overlap_areas.begin(), overlap_areas.end()) < target.boundingBox.area() / 4.0)
                        {
                            flag_lost = true;
                            std::cout << " --- lost da jump 1" << endl;
                        }
                    }
                    overlap_areas.clear();

                    // else if (TMP_ROIs.empty() && target.pos2D_story.size() > 10)
                    // {
                    //     index_pos = 10;

                    //     av_pos_x = 0;
                    //     av_pos_y = 0;

                    //     for (int j = 0; j < index_pos; j++)
                    //     {
                    //         // compute sum position x y of story
                    //         av_pos_x += target.pos2D_story[target.pos2D_story.size()-j].x;
                    //         av_pos_y += target.pos2D_story[target.pos2D_story.size()-j].y;
                    //     }
                    //     // compute average pos
                    //     av_pos_x *= 0.1;
                    //     av_pos_y *= 0.1;

                    //     // compare average position with gap
                    //     if ((av_pos_x < target.pos2D_story[-1].x + 640 * 0.1 || av_pos_x > target.pos2D_story[-1].x - 640 * 0.1) &&
                    //         (av_pos_y < target.pos2D_story[-1].y + 480 * 0.1 || av_pos_y > target.pos2D_story[-1].y - 480 * 0.1))
                    //     {
                    //         std::cout << " --- lost da const pos:" << av_pos_x << " , " << av_pos_y << endl;
                    //         flag_lost = true;
                    //     }

                    // }
                }
                else if (delta > 70)
                {
                    std::cout << " --- lost da jump 2, Delta :" << delta << endl;
                    flag_lost = true;
                }
                else
                {
                    flag_lost = false;
                }
            }
            // Update the target class & it's histogram every "refresh_hist" frames
            if (count_hist == refresh_hist && tracker_counter != max_frame_lost && flag_lost == false)
            {
                cv::MatND old_hist = target.histogram;
                target.target_update(frame, &trackers, 1);
                count_hist = 0;
                // compare new hist with previous, if different very much -> LOST_TRACK
                double comparison = cv::compareHist(target.histogram, old_hist, 1);
                std::cout << comparison << endl;
                if (comparison < 2)
                {
                    std::cout << " --- lost da hist comp:" << comparison << endl;
                    flag_lost = true;
                }
            }
            else
            {
                target.target_update(frame, &trackers, 0);
            }

            // TO DO: check if lost target
            // centre_bbox = (target_bbox.br() - target_bbox.tl()) / 2;
            w_TAR = target.boundingBox.width;
            h_TAR = target.boundingBox.height;
            w_SBBox = target.starting_BBOx.width;
            h_SBBox = target.starting_BBOx.height;

            // ---- Check dimension of tracked box ----
            if (tracker_counter == max_frame_lost & flag_lost == false)
            {
                if ((w_TAR <= th_SBBox_min * w_SBBox) || (w_TAR >= th_SBBox_max * w_SBBox) ||
                    (h_TAR <= th_SBBox_min * h_SBBox) || (h_TAR >= th_SBBox_max * h_SBBox))
                {
                    std::cout << " --- lost da change dimension from starting box" << endl;
                    flag_lost = true;
                }
                tracker_counter = 0;
            }

            if (flag_lost == false)
            {
                tracker_counter++;
                count_hist++;

                currentState = TRACK;
                prevState = TRACK;

                // --> state machine: robot moving + avoid ostacles
                // ...
            }
            else
            {
                std::cout << ">> go to LOST_TRACK" << endl;
                tracker_counter = 0;
                currentState = LOST_TRACK;
                prevState = TRACK;
            }

            break;

        case LOST_TRACK:
            // cout << "LOST_TRACK" << endl;
            cv::putText(frame, "LOST-TRACK", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 2);
            // TO DO: need to find back the target person on the frame
            /* IDEA:
            ok - use last working detector
            -- - detection aroud the area where lost -> take centroid of 5 frame before lost flag on
            ?  - compare new detection hist with previous target hist
            */

            //--------------------------------------
            //---- Extract the portion of frame ----

            /* code */
            // TODO:
            // Mat portion_frame = crop the frame () --> meglio di no

            //---- Detection ------------
            if (classifierType == classifierTypes[0])
            {
                pedestrian_cascade.detectMultiScale(frame, ROIs, 1.4, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
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

            if (counter_lost != 10 && !(ROIs.empty()))
            {
                // -- do comparison hist
                vector<double> compare_hist;
                for (int i = 0; i < ROIs.size(); i++)
                {
                    compare_hist.push_back(comparison_hist(frame, target.histogram, ROIs[i]));
                }
                int maxElementIndex = std::max_element(compare_hist.begin(), compare_hist.end()) - compare_hist.begin();
                // int minElementIndex = std::min_element(compare_hist.begin(), compare_hist.end()) - compare_hist.begin();

                if (compare_hist[maxElementIndex] > 5)
                {
                    Rect2d New_ROI = ROIs[maxElementIndex];

                    // -- update tracker & target, go to TRACK
                    target.starting_BBOx = New_ROI;
                    tracker = TrackerCSRT::create();
                    trackers.clear();
                    trackers.add(tracker, frame, target.starting_BBOx);
                    trackers.update(frame);
                    target.target_update(frame, &trackers, 1);
                    count_hist = 0;
                    counter_lost = 0;
                    currentState = TRACK;
                    prevState = LOST_TRACK;
                }
                else
                {
                    counter_lost++;
                }
            }
            else if (counter_lost == 10)
            {
                // if fallisco il rematching -> go to DETECT
                cout << "lost counter " << counter_lost << endl;
                trackers.clear();
                tracker = TrackerCSRT::create();
                counter_lost = 0;
                currentState = DETECT;
                prevState = LOST_TRACK;
            }
            else
            {
                cout << "lost counter " << counter_lost << endl;
                counter_lost++;
                currentState = LOST_TRACK;
                prevState = LOST_TRACK;
            }

            break;
            // default:
            //     currentState = DETECT;
            //     break;
        }

        // --- Show Stuff
        // start chrono
        start = chrono::high_resolution_clock::now();

        // print rectangles on the frame
        if (!(ROIs.empty()) && currentState != TRACK)
        {
            for (int j = 0; j < ROIs.size(); j++)
            {
                cv::rectangle(frame, ROIs[j], Scalar(255, 0, 0), 3, 8, 0);
            }
            cv::rectangle(frame, target.boundingBox, Scalar(0, 255, 255), 1, 8, 0);
        }
        else if (currentState == TRACK)
        {
            cv::rectangle(frame, target.boundingBox, Scalar(0, 0, 255), 1, 8, 0);
        }

        ROIs.clear();

        // plot di debug
        if (!(TMP_ROIs.empty()))
        {
            for (int j = 0; j < TMP_ROIs.size(); j++)
            {
                cv::rectangle(frame, TMP_ROIs[j], Scalar(255, 255, 0), 3, 8, 0);
            }
            TMP_ROIs.clear();
        }

        cv::imshow("Video", frame);

        // stop chrono
        end = chrono::high_resolution_clock::now();
        // eval time taken
        time_taken = chrono::duration_cast<chrono::microseconds>(end - start).count();
        ts_visu.push_back((float)time_taken / 1000.0);

        if (waitKey(1) == 27) //1000 / 30
        {
            return 0;
        }
    }

    float t_det = std::accumulate(ts_det.begin(), ts_det.end(), 0.0) / ts_det.size();
    float t_track = std::accumulate(ts_track.begin(), ts_track.end(), 0.0) / ts_track.size();
    float t_visu = std::accumulate(ts_visu.begin(), ts_visu.end(), 0.0) / ts_visu.size();

    std::cout << "Timing :" << endl;
    std::cout << "   Detection time     :  " << t_det << "\t[ms]" << endl;
    std::cout << "   Tracking  time     :  " << t_track << "\t[ms]" << endl;
    std::cout << endl;
    std::cout << "   Visualization time :  " << t_visu << "\t[ms]" << endl;
}
