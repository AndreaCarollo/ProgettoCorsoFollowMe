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
    // VideoCapture cap(0);
    VideoCapture cap("../../../Dataset/Markers/vid1.mp4");

    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    // Steps to write the output video
    int frame_width = cap.get(CAP_PROP_FRAME_WIDTH); 
    int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);

    // VideoWriter object
    VideoWriter video("out_video.avi",VideoWriter::fourcc('M','J','P','G'),10, Size(frame_width, frame_height));

    Mat frame, frame_gray;
    vector<Rect> ROIs;
    vector<Rect2d> ROIs_track;
    Rect2d ROI;
    Point center;
    bool flag_det;  // flag that identifies when detection occurs

    /*** Detector initialization ***/
    // String haar_fullbody = "/home/andrea/opencv/sources/opencv/data/haarcascades/haarcascade_fullbody.xml";
    String haar_pedestrian = "../trained/haarcascade_pedestrian.xml";
    CascadeClassifier people_cascade; // Class for the cascade classifier
    people_cascade.load(haar_pedestrian);

    /*** Tracker initialization ***/
    MultiTracker trackers;
    std::vector< Ptr<Tracker> > algorithms;

    // Initialization of the class
    person user;

    while(true)
    {
        int i = 0;
        i++;

        cap >> frame;

        // Compute the center of the frame
        if(i == 1)
            center = Point (frame.cols*0.5, frame.rows*0.5);

        // DETECTION
        cvtColor(frame, frame_gray, COLOR_RGB2GRAY);
        equalizeHist(frame_gray, frame_gray);
        people_cascade.detectMultiScale(frame_gray, ROIs, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

        // TRACKING
        if(ROIs.size() > 0 && flag_det){
            for(int j = 0; j < ROIs.size(); j++){
                algorithms.push_back(TrackerCSRT::create());
                ROIs_track.push_back((Rect2d)ROIs[j]);
            }

            trackers.add(algorithms, frame, ROIs_track);

            for(int j = 0; j < ROIs_track.size(); j++){
                ROIs.push_back(trackers.getObjects()[j]);
            }
            flag_det = false;
        }
        else{
            trackers.update(frame);
            if(trackers.getObjects().size() > 0){
                for(int j = 0; j < ROIs_track.size(); j++){
                    ROIs.push_back(trackers.getObjects()[j]);
                }
            }
            else
                flag_det = true;        
        }

        // Remove ROIs in order to get only the user one
        if(ROIs.size() > 1){
            //user.remove_ROIs(center, ROIs, 25);
        }

        // Find the QR code
        user.QR_code(frame);

        // VIDEO RESULT
        for(int k = 0; k < ROIs.size(); k++){
            rectangle(frame, ROIs[k], Scalar(0, 255, 0), 3, 8, 0);
        }

        // Bbox of the user
        rectangle(frame, user.boundingBox, Scalar(255, 0 , 0), 3, 8, 0);

        // "Write" the video
        // video.write(frame);

        imshow("Video", frame);
        if (waitKey(1) == 27){
            video.release();
            return 0;
        }
    }   
    return 0;
}