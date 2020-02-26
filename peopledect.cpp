#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
    // String haar_fullbody = "/home/andrea/opencv/sources/opencv/data/haarcascades/haarcascade_fullbody.xml";
    String haar_pedestrian = "../trained/haarcascade_pedestrian.xml";
    CascadeClassifier people_cascade; // Class for the cascade classifier

    // VideoCapture cap("vtest.avi");
    VideoCapture sequence("/home/andrea/Documents/Robotics/Dataset/Our_Video/test1.mp4"); // convert into a realsense video streaming
    Mat frame, frame_gray;
    // people_cascade.load(haar_fullbody);
    people_cascade.load(haar_pedestrian);

    for (int i = 0; i < 1000; i++)
    {
        sequence >> frame;

        // apply calssifier to each frame
        vector<Rect> peoples;
        cvtColor(frame, frame_gray, COLOR_RGB2GRAY);
        // image equalization to increse the performances
        equalizeHist(frame_gray, frame_gray);

        // face detection [method in the class] + scale handling
        // if use fullbody                1.05, 5
        // if use haarcascade pedestrian: 1.5 , 30
        // both min Size(50,50)
        // people_cascade.detectMultiScale(frame_gray, peoples, 1.05, 5, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));
        people_cascade.detectMultiScale(frame_gray, peoples, 1.5, 30, 0 | CASCADE_DO_CANNY_PRUNING, Size(50, 50));

        // display the results
        for (int j = 0; j < peoples.size(); j++)
        {
            Point center(peoples[j].x + peoples[j].width / 2.0, peoples[j].y + peoples[j].height / 2.0);
            rectangle(frame, peoples[j], Scalar(255, 0, 0), 3, 8, 0);
            // ellipse(frame, center, Size(peoples[j].width * 0.5, peoples[j].height * 0.5), 0, 0, 360, Scalar(255, 0, 0), 4, 8, 0);
        }



        imshow("Video", frame);
        if (waitKey(10) == 27)
        {
            return 0;
        }
    }
    return 0;
}