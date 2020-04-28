#include "./lib/followme.h"
#include "./lib/utils.h"
#include "./lib/segmentation.h"
#include "./lib/configurator.h"
#include "./lib/rs_stream.h"
#include "./lib/control.h"


using namespace std;



// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~ main ~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main (int argc, char** argv)
{
    // ~~~~~~~~~~~~~~~~~~~ CONFIGURATION FILE ~~~~~~~~~~~~~~~~~~~ //
    
    // Create the configurator object and parse conf.ini file
    ConfigReader *p = ConfigReader::getInstance();
    p->parseFile((char *) argv[1]);                     // Use the file "d435i_walk_arround.bag"


    // ~~~~~~~~~~~~~~~~~~~~ INITIALIZATIONS ~~~~~~~~~~~~~~~~~~~~~ //

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Create a Pipeline and the frameset for the data acquisition
    rs2::pipeline pipe;
    rs2::frameset frames;

    // Others - OPENCV
    int x_cv, y_cv;
    float x_rel, y_rel;
    cv::Point target_point;

    // Others - visualization and timing
    PntCld::Ptr cloud_tmp;
    std::vector<float> durations_control, durations_rgb_acq, durations_visu;
    PntCldV::Ptr viewer(new PntCldV ("3D Viewer"));
    viewer->initCameraParameters();


    // ~~~~~~~~~~~~~~~~~~~~ START THE STREAM ~~~~~~~~~~~~~~~~~~~~ //

    // Add desired streams to configuration
    // camSettings(&cfg, p);                               // Enable stream from the camera
    cfg.enable_device_from_file((char *) argv[2]);      // Enable stream from a recordered device (.bag file)
                                                        // Use the file "d435i_walk_arround.bag" to perfrom good results
                                                        // If will you use that file, use the "config_chess.ini" initialization file
    
    // Start the pipeline
    pipe.start(cfg);

    // Initialize the class stream: the first frame from the camera is used for this purpose
    std::string stream_name = "Realsense stream";
    frames = pipe.wait_for_frames();
    Stream stream(stream_name, &frames, p);


    // Initialize the control stream
    Control ctrl = Control(p);


    // LOOP
    while(!viewer->wasStopped()) // The visualizer stops, stop the code
    {
        
        frames = pipe.wait_for_frames();
        
        // ~~~~~~~~~~~~~~~~~~ OpenCV Part ~~~~~~~~~~~~~~~~~~~~~~~ //

        auto start_rgb_acq = std::chrono::high_resolution_clock::now();

        // Load the images from the camera (update the straem) and convert the rgb frame in cv::Mat
        stream.update(&frames);
        stream.RGB_acq();

        auto stop_rgb_acq = std::chrono::high_resolution_clock::now();
        auto duration_rgb_acq = std::chrono::duration_cast<std::chrono::microseconds>(stop_rgb_acq - start_rgb_acq);

        durations_rgb_acq.push_back((float) duration_rgb_acq.count()/1000);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
        // ~~~~~~~~~~~~~~~~~~ STATE MACHINE ~~~~~~~~~~~~~~~~~~~~~ //
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

        // Target point choosing (since the state machine there aren't)
        y_rel = 0.5;                                    x_rel = 0.5;
        y_cv = stream.color_frame.rows * y_rel;         x_cv = stream.color_frame.cols * x_rel;
        target_point = cv::Point(x_cv, y_cv);

        // A rectangle is put on the image as a marker
        cv::rectangle(stream.color_frame,cv::Point(x_cv-5,y_cv-5),cv::Point(x_cv+5,y_cv+5),cv::Scalar(0,0,255),5);


        // ~~~~~~~~~~~~~~~~~~ Control Part ~~~~~~~~~~~~~~~~~~~~~~~ //

        auto start_control = std::chrono::high_resolution_clock::now();

        // Update the control -> all others operations are done inside the update function
        ctrl.update(&target_point, &stream);

        auto stop_control = std::chrono::high_resolution_clock::now();
        auto duration_control = std::chrono::duration_cast<std::chrono::microseconds>(stop_control - start_control);

        durations_control.push_back((float) duration_control.count()/1000);

        
        // ~~~~~~~~~~~~~~~ Visualization Part ~~~~~~~~~~~~~~~~~~~~ //

        // Does not work
        // Apply transformation mtx to plane and pcl
        // pcl::transformPointCloud(*stream.cloud, *cloud_tmp, ctrl.plane->transf_mtx);
        // stream.cloud.swap (cloud_tmp);

        // pcl::transformPointCloud(*ctrl.plane->plane_cloud, *cloud_tmp, ctrl.plane->transf_mtx);
        // ctrl.plane->plane_cloud.swap (cloud_tmp);
        
        auto start_visu = std::chrono::high_resolution_clock::now();

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

        auto stop_visu = std::chrono::high_resolution_clock::now();
        auto duration_visu = std::chrono::duration_cast<std::chrono::microseconds>(stop_visu - start_visu);

        durations_visu.push_back((float) duration_visu.count()/1000);

    }

    float t_rgb_acq = accumulate( durations_rgb_acq.begin(), durations_rgb_acq.end(), 0.0) / durations_rgb_acq.size();
    float t_control = accumulate( durations_control.begin(), durations_control.end(), 0.0) / durations_control.size();
    float t_visu    = accumulate( durations_visu.begin()   , durations_visu.end()   , 0.0) / durations_visu.size();

    cout << "Timing :"                                              << endl;
    cout << "   RGB acquisition time   :  " << t_rgb_acq << "\t[ms]" << endl;
    cout << "   Control algorithm time :  " << t_control << "\t[ms]" << endl;
    cout << endl;
    cout << "   Visualization time     :  " << t_visu    << "\t[ms]" << endl;

    

    return (0);
}
