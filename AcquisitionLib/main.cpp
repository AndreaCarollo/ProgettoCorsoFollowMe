// Include the acquisition library
#include "./lib/followme.h"
#include "./lib/rs_stream.h"
#include "./lib/utils.h"


int main(int argc, char const *argv[]) try
{

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    
    // Add desired streams to configuration
    // camSettings_rec(&cfg);                      // enable stream from the camera
    cfg.enable_device_from_file((char *) argv[1]);    // Enable stream from a recordered device (.bag file)

    // If we use a recordered device, we must be sure that the stream contains the frames that we will use
    // ( for example the "file1.bag" does not have the infrared stream )

    // Create a Pipeline for the data acquisition from the camera
    rs2::pipeline p;

    // initialize the viewer for the point cloud
    PntCldV::Ptr viewer(new PntCldV ("3D Viewer"));

    // Configure and start the pipeline
    p.start(cfg);

    // The camera of the viewer is "initialized"
    viewer->initCameraParameters();

    //Call the class Stream
    std::string stream_name = "Realsense stream";
    Stream stream(stream_name);


    while(true)
    {
        // Start chrono time
        auto start = std::chrono::high_resolution_clock::now();

        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Update the stream object
        stream.update(&frames);

        // Load the images from the camera and convert it in cv::Mat
        stream.RGB_acq();
        // stream.IR_acq();
        stream.PC_acq(false);        // If the argument is true, also a cv::Mat is generated for the depth (default = false)

        // Start chrono time
        auto stop_acq = std::chrono::high_resolution_clock::now();

        auto duration_acq = std::chrono::duration_cast<std::chrono::milliseconds>(stop_acq - start);

        // Start chrono time
        auto start_comp = std::chrono::high_resolution_clock::now();

        // Select the target point (now the center of the image)
        cv::Point pointFromDetection = cv::Point(stream.color_frame.cols*0.5, stream.color_frame.rows*0.5);
        // Transform the target point in (RGB reference system) into DEPTH reference system
        stream.project_RGB2DEPTH(&pointFromDetection);

        // Start chrono time
        auto stop_comp = std::chrono::high_resolution_clock::now();

        auto duration_comp = std::chrono::duration_cast<std::chrono::microseconds>(stop_comp - start_comp);

        // Start chrono time
        auto start_visu = std::chrono::high_resolution_clock::now();

        // Rappresentation of the poit cloud
        PCViewer(stream.cloud, viewer);

        // A rectangle is put on the image as a marker
        cv::rectangle(stream.color_frame,
                      cv::Point(pointFromDetection.x-5,pointFromDetection.y-5),
                      cv::Point(pointFromDetection.x+5,pointFromDetection.y+5),
                      cv::Scalar(0,0,255),5);

        viewer->addCube(stream.refPnt.x-0.01,stream.refPnt.x+0.01,
                        stream.refPnt.y-0.01,stream.refPnt.y+0.01,
                        stream.refPnt.z-0.01,stream.refPnt.z+0.01,
                        1.0,0.0,0.0,"cube");

        // Rappresentation of the images from the camera
        cv::imshow("color", stream.color_frame);
        // cv::imshow("infrared", stream.infrared_frame);
        // cv::imshow("depth", stream.depth_frame);

        // Show the viewer
        viewer -> spinOnce();

        // If the ESC button is pressed, the cycle is stopped and the program finishes
        if (cv::waitKey(1) == 27)
        {
            p.stop();
            break;
        }

        // Remove the point cloud from the viewer
        viewer -> removePointCloud("sample cloud");
        viewer -> removeShape("cube");

        // Start chrono time
        auto stop = std::chrono::high_resolution_clock::now();

        auto duration_visu = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_visu);

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "Duration time : \t\t" << duration.count() << "\tms" << std::endl;
        std::cout << "\tAcquisition time   : \t" << duration_acq.count() << "\tms" << std::endl;
        std::cout << "\tComputation time   : \t" << duration_comp.count() << "\tus" << std::endl;
        std::cout << "\tVisualization time : \t" << duration_visu.count() << "\tms" << std::endl << std::endl;

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