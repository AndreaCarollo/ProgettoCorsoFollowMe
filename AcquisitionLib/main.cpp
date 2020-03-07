// Include the acquisition library
#include "./acq_lib.h"


int main() try
{

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    //Add desired streams to configuration
    camSettings(&cfg);

    // Create a Pipeline for the data acquisition from the camera
    rs2::pipeline p;

    // Initialize the OpenCV frames
    cv::Mat color_frame, infrared_frame, depth_frame;

    // Initialize the point cloud 
    PntCld cloud;

    // initialize the viewer for the point cloud
    PntCldV viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));


    bool counter = true;

    // Configure and start the pipeline
    p.start(cfg);

    while(true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Load the images from the camera and convert it in cv::Mat
        RGB_acq(&color_frame, frames);
        IR_acq(&infrared_frame, frames);
        DEPTH_acq(&depth_frame, frames);

        // Load the depth image and transform it in a point cloud (pcl)
        cloud = PC_acq(frames);

        // The camera of the viewer is "initialized"
        viewer->initCameraParameters();

        // Rappresentation of the poit cloud
        PCViewer(cloud, viewer);

        // Rappresentation of the images from the camera
        cv::imshow("color",    color_frame);
        cv::imshow("infrared", infrared_frame);
        cv::imshow("depth",    depth_frame);

        // If the ESC button is pressed, the cycle is stopped and the program finishes
        if (cv::waitKey(1) == 27)
        {
            p.stop();
        }

        // Remove the point cloud from the viewer
        viewer -> removePointCloud("sample cloud");

    }
    
    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}