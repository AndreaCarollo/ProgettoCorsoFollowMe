#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/aruco.hpp>    // for the ArUco Markers

#include <ctime>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cfloat>
#include <cmath>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    VideoCapture cap(argv[1]);

    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    Mat frame, frame_copy;
    int i = 0;

    // Steps to write the output video
    int frame_width = cap.get(CAP_PROP_FRAME_WIDTH); 
    int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);

    // VideoWriter object
    //VideoWriter video("out_video.avi",VideoWriter::fourcc('M','J','P','G'),10, Size(frame_width,frame_height));

    // Define the dictionary and parameters of the ArUco markers
    Ptr<aruco::Dictionary> dict = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);
    Ptr<aruco::DetectorParameters> param = aruco::DetectorParameters::create();



    while((cap >> frame).grab()){
        i++;

        frame.copyTo(frame_copy);

        // Marker detection
        vector<int> markerIDs;                      // IDs of markers
        vector< vector<Point2f> > markerCorn;       // contains the corners of the markers
        aruco::detectMarkers(frame_copy, dict, markerCorn, markerIDs, param);

        // Draw the results on the image
        if(markerIDs.size() > 0){                   // at least 1 marker detected
            aruco::drawDetectedMarkers(frame_copy, markerCorn, markerIDs);
            for(int i = 0; i < markerIDs.size(); i++){
                printf("ID --> %d\n", markerIDs[i]);
            }
        }

        // Write the frame in the output video
        //video.write(frame_copy);

        putText(frame_copy, "frame: " + to_string(i), cv::Point(10, 20), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1);
        imshow("Markers detection", frame_copy);
        waitKey(30);

        // Write the images of the frames
        //string save_string = "../DemoMarker/frame_" + to_string(i) + ".jpg";
        //imwrite(save_string, frame_copy);
    }

    destroyAllWindows();
    return 0;
}