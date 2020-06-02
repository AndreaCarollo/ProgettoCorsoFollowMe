#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include "./lib/followme.h"
#include "./lib/person.h"

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

//--------------------------------------//
//                                      //
//  THIS IS A DEMO OF PEOPLE DETECTION  //
//                                      //
//--------------------------------------//

int main(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("** Error. Usage: ./Demo_dect <video_path>\n");
        return -1;
    }

    // VideoCapture cap("../../../Dataset/Our_Video/test8.mp4");
    VideoCapture cap(argv[1]);

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
    vector<Rect> ROIs_ped, ROIs_ful, ROIs_up;
    Rect2d ROI;

    // ---- Detector initialization ---
    String haar_fullbody = "../src/trained//haarcascade_fullbody.xml";
    String haar_pedestrian = "../src/trained/haarcascade_pedestrian.xml";
    String haar_upbody = "../src/trained/haarcascade_upperbody.xml";

    // ---- Create classifiers ----
    CascadeClassifier fullbody_cascade;
    fullbody_cascade.load(haar_fullbody);

    CascadeClassifier pedestrian_cascade;
    pedestrian_cascade.load(haar_pedestrian);

    CascadeClassifier upbody_cascade;
    upbody_cascade.load(haar_upbody);

    // Other for Time Analysis
    std::vector<float> ts_det, ts_track, ts_visu;
    auto start = chrono::high_resolution_clock::now();
    auto end = chrono::high_resolution_clock::now();
    double time_taken = 0;

    int index = 0;

    cap >> frame;

    // save video
    int frame_width  = frame.cols;
    int frame_height = frame.rows;
    VideoWriter video("../Demo_detection.avi", cv::VideoWriter::fourcc('M','J','P','G'),10, Size(frame_width,frame_height),true);


    while ((cap >> frame).grab())
    {

        // std::cout << "DETECT" << endl;
        // cv::putText(frame, "DETECT", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 2);

        // start chrono

        // Detection on frame
        pedestrian_cascade.detectMultiScale(frame, ROIs_ped, 1.14, 24, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
        fullbody_cascade.detectMultiScale(frame, ROIs_ful, 1.05, 6, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
        upbody_cascade.detectMultiScale(frame, ROIs_up, 1.05, 6, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

        if (!(ROIs_ped.empty()))
        {
            for (int j = 0; j < ROIs_ped.size(); j++)
            {
                cv::rectangle(frame, ROIs_ped[j], Scalar(255, 0, 0), 3, 8, 0);
            }
        }
        if (!(ROIs_ful.empty()))
        {
            for (int j = 0; j < ROIs_ful.size(); j++)
            {
                cv::rectangle(frame, ROIs_ful[j], Scalar(0, 255, 0), 3, 8, 0);
            }
        }

        if (!(ROIs_up.empty()))
        {
            for (int j = 0; j < ROIs_up.size(); j++)
            {
                cv::rectangle(frame, ROIs_up[j], Scalar(0, 0, 255), 3, 8, 0);
            }
        }

        // Write info in frame
        cv::putText(frame, "Detection Demo", cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1.5);
        cv::putText(frame, "frame: " + to_string(index), cv::Point(10, 40), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1.5);
        // legend
        cv::putText(frame, "Pedestrian", cv::Point(530, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 1.8);
        cv::putText(frame, "Full Body",  cv::Point(530, 40), cv::FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 0), 1.8);
        cv::putText(frame, "Upper Body", cv::Point(530, 60), cv::FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 255), 1.8);

        cv::imshow("Video", frame);
        if (waitKey(1) == 27) //1000 / 30
        {
            return 0;
        }

        // if(index == 127 || index == 204||index == 217||index == 237 ){
        string save_string = "../DemoImg/det_" + to_string(index) + ".jpg";
        imwrite(save_string, frame);
        //}

        index++;
        
        video.write(frame);
    }
}
