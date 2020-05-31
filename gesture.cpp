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
#include <math.h>

using namespace cv;
using namespace std;


int main()
{
    // Open the webcam or the video
    //VideoCapture cap(0);
    VideoCapture cap("../in_video.mp4");

    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    int frame_width = cap.get(CAP_PROP_FRAME_WIDTH); 
    int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);

    // VideoWriter object
    VideoWriter video1("in_video.avi", VideoWriter::fourcc('M','J','P','G'),10, Size(frame_width,frame_height));
    VideoWriter video2("out_video.avi", VideoWriter::fourcc('M','J','P','G'),10, Size(frame_width,frame_height));

    int i = 0;
    Mat frame, frame_org, roi, frameHSV, handMask;
    std::vector< std::vector<Point> > contours;

    // Range values for the skin color
    Scalar lowerSkin = Scalar(0, 20, 70);  
    Scalar upperSkin = Scalar(20, 255, 255);

    // Definition of the colors (BGR)
    Scalar blue = Scalar(255, 0, 0);
    Scalar green = Scalar(0, 255, 0);
    Scalar red = Scalar(0, 0, 255);

    // Font for the text on the output image
    HersheyFonts font = FONT_HERSHEY_COMPLEX;

    while((cap >> frame).grab())
    {
        i++;
        // cap >> frame;
        // flip(frame, frame, 1);
        // frame_org = frame.clone();

        // ***** STEP 1 --> BINARY IMAGE FOR THE HAND *****

        // Select the region of interest and show it
        roi = frame(Rect(300, 100, 200, 200));
        rectangle(frame, Point(300, 100), Point(500, 300), green, 3);

        // Convert BGR --> HSV, consider the skin range
        cvtColor(roi, frameHSV, COLOR_BGR2HSV);
        inRange(frameHSV, lowerSkin, upperSkin, handMask);

        // Dilation
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilate(handMask, handMask, kernel, Point2f(-1, 1), 1);

        // Blur the mask (remove noise) then apply it
        GaussianBlur(handMask, handMask, Size(3, 3), 0);

        // ***** STEP 2 --> CONTOUR AND CONVEX HULL *****

        // Generate the contours, take only the largest
        findContours(handMask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

        try
        {
            // Find the contour with the max area (hand)
            std::vector<double> areas;
            for(int j = 0; j < contours.size(); j++)
            {
                areas.push_back(contourArea(contours[j]));
            }
            int max = max_element(areas.begin(), areas.end()) - areas.begin();
            std::vector<Point> handContour = contours[max];

            // Make an approximation of the contour
            double eps = 0.01*arcLength(handContour, true);
            std::vector<Point> approxCnt;
            approxPolyDP(handContour, approxCnt, eps, true);

            // Define the convex hull around the hand
            std::vector<Point> hull;
            convexHull(handContour, hull);

            // Find hand and hull areas, define the ratio
            double areaHull = contourArea(hull);
            double areaCnt = contourArea(handContour);
            double areaRatio = ((areaHull - areaCnt)/areaCnt)*100;

            // Find the defects in convex hull with respet to the hand
            convexHull(approxCnt, hull, false, false);
            std::vector<int> hull_I;
            convexHull(approxCnt, hull_I, false, false);
            std::vector<Vec4i> defects;
            convexityDefects(approxCnt, hull_I, defects);
            drawContours(roi, contours, -1, blue, 2);

            // ***** STEP 3 --> ANALYZE THE DEFECTS DUE TO FINGERS *****

            int l = 0;          // number of defects

            for(int j = 0; j < defects.size(); j++)
            {
                // Extract indexes
                int s = defects[j][0];
                int e = defects[j][1];
                int f = defects[j][2];

                // Find correspinding points in the approximated contour
                Point start = approxCnt[s];
                Point end = approxCnt[e];
                Point far = approxCnt[f];

                // Lenght of all the sides of the triangle
                float a = eucledian_norm(start, end);
                float b = eucledian_norm(start, far);
                float c = eucledian_norm(far, end);
                float semi = (a+b+c)/2;                         // semi-perimeter
                float area_tr = sqrt( s*(s-a)*(s-b)*(s-c));     // Erone's formula

                // Distance btw point and convex hull
                float dist = (2*area_tr)/a;

                // Find the angle btw b and c
                float angle = acos((pow(b, 2) + pow(c, 2) - pow(a, 2))/(2*b*c));

                // Ignore angles > 90 deg and points close to convex hull
                if(angle <= M_PI/2 && dist > 30)
                {   
                    l += 1;
                    circle(roi, far, 3, blue, -1);
                }

                // Draw lines around the hand
                line(roi, start, end, green, 2);
            }

            l += 1;

            // ***** STEP 4 --> IDENTIFY GESTURES *****
            if(l == 1)
            {
                if(areaCnt < 2000)
                    putText(frame, "Put hand in the box", Point(0, 50), font, 1, red, 3);
                else
                {
                    if(areaRatio < 12)
                        putText(frame, "0", Point(0, 50), font, 1, red, 3);   
                    else if(areaRatio < 17.5)
                        putText(frame, "Best of luck", Point(0, 50), font, 1, red, 3);
                    else
                       putText(frame, "1", Point(0, 50), font, 1, red, 3);
                }
            }
            else if(l == 2)
            {
                putText(frame, "2", Point(0, 50), font, 1, red, 3);
            }
            else if(l == 3)
            {
                if(areaRatio < 27)
                    putText(frame, "3", Point(0, 50), font, 1, red, 3);
                else
                    putText(frame, "ok", Point(0, 50), font, 1, red, 3);
            }
            else if(l == 4)
            {   
                putText(frame, "4", Point(0, 50), font, 1, red, 3);
            }
            else if(l == 5)
            {
                putText(frame, "5", Point(0, 50), font, 1, red, 3);
            }
            else 
            {
                putText(frame, "Reposition", Point(0, 50), font, 1, red, 3);
            }
        }

        // if segmentation fault occurs, go to next frame without doing any computations
        catch(const std::exception& e)
        {
            ;
        }

        putText(frame, "frame: " + to_string(i), cv::Point(10, 450), cv::FONT_HERSHEY_DUPLEX, 1, red, 1);

        // Save frames
        // string save_string1 = "../hand/hand_" + to_string(i) + ".jpg";
        // imwrite(save_string1, roi);
        // string save_string2 = "../BH/BH_" + to_string(i) + ".jpg";
        // imwrite(save_string2, handMask);

        // Streams
        imshow("Frame", frame);              // full frame
        imshow("Binary hand", handMask);     // binary hand
        imshow("Only hand", roi);            // hand

        // Write the videos
        // video1.write(frame_org);
        // video2.write(frame);

        if (waitKey(1) == 27)
        {
            video1.release();
            video2.release();
            return 0;
        }
        waitKey(30);
    }
    video1.release();
    video2.release();
    return 0;
}