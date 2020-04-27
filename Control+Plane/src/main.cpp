#include "./lib/followme.h"
#include "./lib/utils.h"
#include "./lib/segmentation.h"
#include "./lib/configurator.h"
#include "./lib/rs_stream.h"
#include "./lib/control.h"


using namespace std;



// --------------------------------------------
// ------------------ main --------------------
// --------------------------------------------
int main (int argc, char** argv)
{
    // ------------------ Configuration file ----------------- //
    
    // Create the configurator object and parse conf.ini file
    ConfigReader *p = ConfigReader::getInstance();
    p->parseFile((char *) argv[1]);


    // SOME INITIALIZATIONS

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    rs2::frameset frames;

    // Create a Pipeline for the data acquisition from the camera
    rs2::pipeline pipe;

    // initialize the viewer for the point cloud
    PntCldV::Ptr viewer(new PntCldV ("3D Viewer"));

    // The camera of the viewer is "initialized"
    viewer->initCameraParameters();

    // Others - OPENCV
    int x_cv, y_cv;
    float x_rel, y_rel;
    cv::Point target_point;

    // Others - CONTROL
    Control ctrl = Control(p);      // INITIALIZATION OF THE CONTROL CLASS

    // Others
    PntCld::Ptr cloud_tmp;
    std::vector<float> durations_control, durations_rgb_acq;


    // START THE STREAM

    // Add desired streams to configuration
    cfg.enable_device_from_file((char *) argv[2]);    // Enable stream from a recordered device (.bag file)
    // Configure and start the pipeline
    pipe.start(cfg);

    //Call the class Stream
    std::string stream_name = "Realsense stream";
    frames = pipe.wait_for_frames();                   // The first frame is used to initialize the class stream only
    Stream stream(stream_name, &frames, p);


    // LOOP
    // while(cv::waitKey(1) != 27)
    while(!viewer->wasStopped()) // If the ESC button is pressed, the cycle is stopped and the program finishes
    {
        
        frames = pipe.wait_for_frames();
        

        // ------------------ OpenCV Part ----------------------- //

        auto start_rgb_acq = std::chrono::high_resolution_clock::now();

        stream.update(&frames);
        // Load the images from the camera and convert it in cv::Mat
        stream.RGB_acq();

        auto stop_rgb_acq = std::chrono::high_resolution_clock::now();
        auto duration_rgb_acq = std::chrono::duration_cast<std::chrono::microseconds>(stop_rgb_acq - start_rgb_acq);

        durations_rgb_acq.push_back((float) duration_rgb_acq.count()/1000);

        // Target point choosing
        y_rel = 0.5;                                    x_rel = 0.5;
        y_cv = stream.color_frame.rows * y_rel;         x_cv = stream.color_frame.cols * x_rel;

        // A rectangle is put on the image as a marker
        target_point = cv::Point(x_cv, y_cv);
        cv::rectangle(stream.color_frame,cv::Point(x_cv-5,y_cv-5),cv::Point(x_cv+5,y_cv+5),cv::Scalar(0,0,255),5);


        // ---------------- Control Part ------------------------ //

        auto start_control = std::chrono::high_resolution_clock::now();

        // Update the control -> all other operation are called inside the update
        ctrl.update(&target_point, &stream);

        auto stop_control = std::chrono::high_resolution_clock::now();
        auto duration_control = std::chrono::duration_cast<std::chrono::microseconds>(stop_control - start_control);

        // durations_control.push_back((float) duration_control.count()/1000 - (float) ctrl.duration.count()/1000);
        durations_control.push_back((float) duration_control.count()/1000);

        // NOTE: Since the .bag file that we consider does not have a plane, we must add a plane every frame.
        //       The time emploied to add the plane is removed from the control algorithm time.
        //       When we want to use the control in a real application we must remove some initializations in 
        //       "control.h" and the plane adding in "control.cpp -> update(cv::Point* , Stream* )"


        // ---------------- Visualization Part ------------------ //

        // Does not work
        // Apply transformation mtx to plane and pcl
        // pcl::transformPointCloud(*stream.cloud, *cloud_tmp, ctrl.plane->transf_mtx);
        // stream.cloud.swap (cloud_tmp);

        // pcl::transformPointCloud(*ctrl.plane->plane_cloud, *cloud_tmp, ctrl.plane->transf_mtx);
        // ctrl.plane->plane_cloud.swap (cloud_tmp);
        

        // Add a cube to the visualizer that works as a marker
        viewer->addCube(ctrl.refPnt.x-0.030,ctrl.refPnt.x+0.030,
                        ctrl.refPnt.y-0.030,ctrl.refPnt.y+0.030,
                        ctrl.refPnt.z-0.030,ctrl.refPnt.z+0.030,
                        1.0,0.0,0.0);

        PCViewer(stream.cloud, viewer);
        viewer->addPointCloud(ctrl.plane->plane_cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(stream.cloud, 0, 255, 0),"Plane");
        viewer->addCoordinateSystem(1, "RF_cam");
        viewer->addCoordinateSystem(1, ctrl.plane->transf_mtx.inverse(), "RF_plane");


        cv::imshow("Image",stream.color_frame);
        cv::imshow("Control",ctrl.interface->interface);
        viewer->spinOnce (100); // wait for some microseconds, makes the viewer interactive

        cv::waitKey(1);

        // Remove the point cloud from the viewer
        viewer -> removePointCloud("sample cloud");
        viewer -> removePointCloud("Plane");         // this gives segmentation when no plane is found! anyway is only for debug
        viewer -> removeShape("cube");
        viewer -> removeAllCoordinateSystems();

    }

    float t_rgb_acq = accumulate( durations_rgb_acq.begin(), durations_rgb_acq.end(), 0.0) / durations_rgb_acq.size();
    float t_control = accumulate( durations_control.begin(), durations_control.end(), 0.0) / durations_control.size();

    cout << "Timing :"                                              << endl;
    cout << "   RGB acquisition time   :  " << t_rgb_acq << "\t[ms]" << endl;
    cout << "   Control algorithm time :  " << t_control << "\t[ms]" << endl;
    

    return (0);
}
