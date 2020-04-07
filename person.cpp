#include "person.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>    // for the ArUco Markers

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace cv;
using namespace std;


person::person()
{
    //boundingBox = ROI;
    //point2D = (ROI.br() + ROI.tl()) * 0.5;
    //position2D = vector<int>{point2D.x, point2D.y};
    // position3D = vector<int>{point2D.x, point2D.y, profondità};
}

person::~person()
{
}

void person::update(cv::Rect new_ROI)
{
    /* update story + update ROI and positions*/
    update_story();
    boundingBox = new_ROI;
    point2D = (new_ROI.br() + new_ROI.tl()) * 0.5;
    position2D = vector<int>{point2D.x, point2D.y};
    // TO DO: update also the hist
}

void person::update_story()
{
    pos2D_story.push_back(position2D);
    pos3D_story.push_back(position3D);
}

void person::get_proof() // passare pcl, eventuale transform matrix
{
    /* prelevare profondità da pcl */

    /* salvare profondità su vettore 3D */
}

void person::calc_hist(cv::Mat *frame)
{
    int histSize = 256;
    float range[] = {0, 256}; //the upper boundary is exclusive
    const float *histRange = {range};
    bool uniform = true, accumulate = false;
    calcHist(&frame[0], 1, 0, cv::Mat(), *b_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&frame[1], 1, 0, cv::Mat(), *g_hist, 1, &histSize, &histRange, uniform, accumulate);
    calcHist(&frame[2], 1, 0, cv::Mat(), *r_hist, 1, &histSize, &histRange, uniform, accumulate);
}

void detection_on_frame(cv::Mat *frame, vector<Rect> *peoples)
{
    /* code */
    /* do detection of the peoples on the frame */
}

// *** TAKES THESE FUNCTIONS ***

// Take into consideration only the ROI near the center of the camera
void person::remove_ROIs(cv::Point center, std::vector<cv::Rect> ROIs, double thr)
{
    std::vector<double> dist;
    for(int i = 0; i < ROIs.size(); i++){
        dist.push_back(eucledian_norm(center, (ROIs[i].br() + ROIs[i].tl()) * 0.5));
    }
    int min = min_element(dist.begin(), dist.end()) - dist.begin();
    if(dist[min] < thr)
        boundingBox = ROIs[min];
    else{
        Rect boundingBox;
        printf("No ROI satisfy the threshold\n");
    }
}

// Function for computing the distance between two points on a plane
float person::eucledian_norm(cv::Point p1, cv::Point p2)
{
    return sqrt( (float)(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)) );
}


// *** IGNORE QR FUNCTIONS **************************************************
// Find the target with the QR code
void person::QR_code(Mat frame)
{
    QRCodeDetector QRdetector;
    Mat QR_bbox;
    string data = QRdetector.detectAndDecode(frame, QR_bbox);
    if(data.length() > 0)
    {
        cout << "Decoded Data : " << data << endl;
        display_QR(frame, QR_bbox);
        // verify_user(QR_bbox, data);
    }
}

// Display QR code
void person::display_QR(cv::Mat frame, cv::Mat QR_bbox)
{
    line(frame, Point2f(QR_bbox.at<float>(0,0), QR_bbox.at<float>(0,1)),
                Point2f(QR_bbox.at<float>(0,2), QR_bbox.at<float>(0,3)),
                Scalar(255, 0, 0), 3);
    line(frame, Point2f(QR_bbox.at<float>(0,2), QR_bbox.at<float>(0,3)),
                Point2f(QR_bbox.at<float>(0,4), QR_bbox.at<float>(0,5)),
                Scalar(255, 0, 0), 3);
    line(frame, Point2f(QR_bbox.at<float>(0,4), QR_bbox.at<float>(0,5)),
                Point2f(QR_bbox.at<float>(0,6), QR_bbox.at<float>(0,7)),
                Scalar(255, 0, 0), 3);
    line(frame, Point2f(QR_bbox.at<float>(0,6), QR_bbox.at<float>(0,7)),
                Point2f(QR_bbox.at<float>(0,0), QR_bbox.at<float>(0,1)),
                Scalar(255, 0, 0), 3);
}

// Check if the user has the correct QR code
void person::verify_user(cv::Mat QR_bbox, std::string data){
    if (boundingBox.tl().x < QR_bbox.at<float>(0,0) &&
        boundingBox.tl().y < QR_bbox.at<float>(0,1) &&
        boundingBox.br().x > QR_bbox.at<float>(0,4) &&
        boundingBox.br().y > QR_bbox.at<float>(0,5) &&
        data == "TRUE")
        printf("The bounding box corresponds to the user!\n");
    else
        Rect boundingBox;  // l'idea è di annullare il contenuto
}                          

// *******************************************************************************

// **** FUNCTIONS FOR ARUCO MARKERS *****************

// Function for detecting the markers in the environment
void person::detect_aruco(Mat frame, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> param, bool flag)
{
    flag = false;                               // initialize always to false
    // Marker detection
    vector<int> markerIDs;                      // IDs of markers
    vector< vector<Point2f> > markerCorn;       // contains the corners of the mrkers
    aruco::detectMarkers(frame, dict, markerCorn, markerIDs, param);
    // at least 1 marker detected --> draw the markers
    if(markerIDs.size() > 0)
    {
        aruco::drawDetectedMarkers(frame, markerCorn, markerIDs);
        // check the correct marker
        for(int i = 0; i < markerIDs.size(); i++){
            // flag = check_marker(markerIDs[i], markerCorn[i]);            // ATTENZIONE --> IL COMMENTO VA TOLTO
            if(flag == true)
                break;
        }
    }              
}

// Function for checking the user's marker
bool person::check_marker(int ID, std::vector<cv::Point2f> corners)
{
    // check if the bbox contains the marker
    if( boundingBox.contains(corners[0]) ||
        boundingBox.contains(corners[1]) ||
        boundingBox.contains(corners[2]) ||
        boundingBox.contains(corners[3])   )
    {
        // if it contains the marker, then verify the ID's user (for this marker is 25)
        if(ID == userID)
            return true;
        else
            return false;
    }
    return false;
}

// IDEA: se il flag restituito da "detect_aruco" è TRUE:
//       1) cambiare stato DETECT --> TRACK
//       2) associare al tracker "boundingBox"