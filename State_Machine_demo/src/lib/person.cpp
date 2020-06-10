#include "person.h"

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

// Take into consideration only the ROI near the center of the camera
int remove_ROIs(cv::Mat frame, std::vector<cv::Rect> ROIs, double thr, bool *flag)
{
    Point2d center;
    center.x = frame.cols / 2;
    center.y = frame.rows / 2;
    std::vector<double> dist;
    for(int i = 0; i < ROIs.size(); i++){
        dist.push_back(euclidean_norm(center, (ROIs[i].br() + ROIs[i].tl()) * 0.5));
    }
    int min = min_element(dist.begin(), dist.end()) - dist.begin();
    if(dist[min] < thr)
    {
        *flag = true;
    }
    return min;
}

// Function for computing the distance between two points on a plane
float euclidean_norm(cv::Point p1, cv::Point p2)
{
    return sqrt((float)(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)));
}

// Find the target with the QR code
void person::QR_code(Mat frame)
{
    QRCodeDetector QRdetector;
    Mat QR_bbox;
    string data = QRdetector.detectAndDecode(frame, QR_bbox);
    if (data.length() > 0)
    {
        cout << "Decoded Data : " << data << endl;
        display_QR(frame, QR_bbox);
        // verify_user(QR_bbox, data);
    }
}

// Display QR code
void person::display_QR(cv::Mat frame, cv::Mat QR_bbox)
{
    line(frame, Point2f(QR_bbox.at<float>(0, 0), QR_bbox.at<float>(0, 1)),
         Point2f(QR_bbox.at<float>(0, 2), QR_bbox.at<float>(0, 3)),
         Scalar(255, 0, 0), 3);
    line(frame, Point2f(QR_bbox.at<float>(0, 2), QR_bbox.at<float>(0, 3)),
         Point2f(QR_bbox.at<float>(0, 4), QR_bbox.at<float>(0, 5)),
         Scalar(255, 0, 0), 3);
    line(frame, Point2f(QR_bbox.at<float>(0, 4), QR_bbox.at<float>(0, 5)),
         Point2f(QR_bbox.at<float>(0, 6), QR_bbox.at<float>(0, 7)),
         Scalar(255, 0, 0), 3);
    line(frame, Point2f(QR_bbox.at<float>(0, 6), QR_bbox.at<float>(0, 7)),
         Point2f(QR_bbox.at<float>(0, 0), QR_bbox.at<float>(0, 1)),
         Scalar(255, 0, 0), 3);
}

// Check if the user has the correct QR code
void person::verify_user(cv::Mat QR_bbox, std::string data)
{
    if (boundingBox.tl().x < QR_bbox.at<float>(0, 0) &&
        boundingBox.tl().y < QR_bbox.at<float>(0, 1) &&
        boundingBox.br().x > QR_bbox.at<float>(0, 4) &&
        boundingBox.br().y > QR_bbox.at<float>(0, 5) &&
        data == "TRUE")
        printf("The bounding box corresponds to the user!\n");
    else
        Rect boundingBox; // l'idea è di annullare il contenuto
}

// COMMENTI
// Probabilmente si può migliorare l'if con la funzione contains() del boundingBox.
// Forse bisogna usare una variabile globale (nella classe person) per identificare l'user
// così quando si fa la detection di "ostacoli" si sa che uno di questi è proprio l'utente

cv::MatND evaluate_hist(cv::Mat frame, cv:: Rect2d ROI)
{
    Point2d TL = ROI.tl();
    Point2d BR = ROI.br();
    int WBBOX = ROI.width;
    int HBBOX = ROI.height;
    Mat cropframe(frame, Rect(TL.x, TL.y, WBBOX, HBBOX));
    cv::Mat hsv;
    /// Convert to HSV
    cv::cvtColor(frame, hsv, COLOR_BGR2HSV);
    /// Using 50 bins for hue and 60 for saturation
    int h_bins = 50;
    int s_bins = 60;
    int histSize[] = {h_bins, s_bins};
    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = {0, 180};
    float s_ranges[] = {0, 256};
    const float *ranges[] = {h_ranges, s_ranges};

    // Use the o-th and 1-st channels
    int channels[] = {0, 1};
    /// Histograms
    cv::MatND hist_base;
    /// Calculate the histograms for the HSV images
    cv::calcHist(&hsv, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false);
    cv::normalize(hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat());
    return hist_base;
}

double comparison_hist(cv::Mat frame, cv::MatND hist_1, cv::Rect ROI2)
{
    cv::MatND hist_2 = evaluate_hist(frame,ROI2);
    int compare_method = 1;
    double comparison = cv::compareHist(hist_1, hist_2, 1); // compare_method);
    return comparison;
}

void detect_aruco(Mat frame, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> param, Rect ROI, bool *flag)
{
    *flag = false;
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
            bool flagId = check_marker(ROI, markerIDs[i], markerCorn[i]);            
            if(flagId == true)
            {
                *flag = true;
                break;
            }
        }
    }              
}

bool check_marker(Rect ROI, int ID, std::vector<cv::Point2f> corners)
{
    // check if the bbox contains the marker
    if( ROI.contains(corners[0]) ||
        ROI.contains(corners[1]) ||
        ROI.contains(corners[2]) ||
        ROI.contains(corners[3])   )
    {
        // if it contains the marker, then verify the ID's user (for this marker is 25)
        if(ID == 25)
            return true;
        else
            return false;
    }
    return false;
}

