#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp>

using namespace cv;
using namespace std;


int main()
{

    // GLOBAL PARAMETERS


    // create matrix for frames
    cv::Mat frame;
    vector<Rect> detected;

    std::string path_to_img = "../../Dataset/Ply+jpeg/test01_01/a/Renamed/ir_%03d.png";
    std::string path_to_vid = "../../Dataset/Our_Video/VID_20200217_184502.mp4";

    // frame = imread(path_to_img, 0);
    cv::VideoCapture sequence(path_to_img);
    cv::VideoCapture cap(path_to_vid);
    // Detector set-up
    HOGDescriptor hog(Size(48, 96), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    hog.setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
    namedWindow("image",WINDOW_NORMAL);
    resizeWindow("image", 500,500);
    for (int i = 0; i < 10000; i++)
    {
        /* code */
        sequence >> frame;

        /* detection */
        hog.detectMultiScale(frame, detected, 0, Size(50, 100), Size(400, 800), 1.05, 2, false);
        vector<float> descriptorValues;
        hog.compute(frame, descriptorValues);

        for (int j = 0; j < detected.size(); j++)
        {
            rectangle(frame, detected[j], Scalar(0, 255, 0), 2, 1);
        }
        cout << detected.size() << endl;

        cv::imshow("image", frame);
        waitKey(100);
    }
}
