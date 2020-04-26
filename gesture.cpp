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
    // Open the video stream
    VideoCapture cap(0);
    // VideoCapture cap("../../../Dataset/Our_Video/test4.mp4");

    if (!cap.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    int i = 0;
    Mat frame, roi, frameHSV, handMask;
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

    while(true)
    {
        i++;
        cap >> frame;
        flip(frame, frame, 1);

        // ***** STEP 1 --> BINARY IMAGE FOR THE HAND *****

        // Select the region of interest and show it
        roi = frame(Rect(100, 100, 200, 200));
        rectangle(frame, Point(100, 100), Point(300, 300), green, 3);

        // Convert BGR --> HSV, consider the skin range
        cvtColor(roi, frameHSV, COLOR_BGR2HSV);
        inRange(frameHSV, lowerSkin, upperSkin, handMask);

        // Dilation
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilate(handMask, handMask, kernel, Point2f(-1, 1), 2);

        // Blur the mask (remove noise) then apply it
        GaussianBlur(handMask, handMask, Size(3, 3), 0);

        // ***** STEP 2 --> CONTOUR AND CONVEX HULL *****

        // Generate the contours, take only the largest
        findContours(handMask, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
        
        // Find the contour with the max area (hand)
        std::vector<double> areas;
        for(int j = 0; j < contours.size(); j++)
        {
            areas.push_back(contourArea(contours[j]));
        }
        int min = max_element(areas.begin(), areas.end()) - areas.begin();
        std::vector<Point> handContour = contours[min];

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
                putText(frame, "Put hand in the box", Point(0, 50), font, 2, red, 3);
            else
            {
                if(areaRatio < 12)
                    putText(frame, "0", Point(0, 50), font, 2, red, 3);   
                else if(areaRatio < 17.5)
                    putText(frame, "Best of luck", Point(0, 50), font, 2, red, 3);
                else
                    putText(frame, "1", Point(0, 50), font, 2, red, 3);
            }
        }
        else if(l == 2)
        {
            putText(frame, "2", Point(0, 50), font, 2, red, 3);
        }
        else if(l == 3)
        {
            if(areaRatio < 27)
                putText(frame, "3", Point(0, 50), font, 2, red, 3);
            else
                putText(frame, "ok", Point(0, 50), font, 2, red, 3);
        }
        else if(l == 4)
        {
            putText(frame, "4", Point(0, 50), font, 2, red, 3);
        }
        else if(l == 5)
        {
            putText(frame, "5", Point(0, 50), font, 2, red, 3);
        }
        else 
        {
            putText(frame, "Reposition", Point(0, 50), font, 2, red, 3);
        }


        imshow("Hand", frame);
        imshow("Binary hand", handMask);
        if (waitKey(1) == 27)
        {
            return 0;
        }
    }

    return 0;
}