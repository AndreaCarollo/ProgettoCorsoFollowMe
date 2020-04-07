// Include the acquisition library
#include "./acq_lib.h"


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

    // Initialize the OpenCV frames
    // cv::Mat infrared_frame;
    cv::Mat color_frame, depth_frame;

    // Initialize the point cloud
    PntCld cloud;

    // initialize the viewer for the point cloud
    PntCldV viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));


    bool counter = true;

    // Configure and start the pipeline
    p.start(cfg);

    // The camera of the viewer is "initialized"
    viewer->initCameraParameters();



    while(true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Load the images from the camera and convert it in cv::Mat
        RGB_acq(&color_frame, frames);
        // IR_acq(&infrared_frame, frames);
        // DEPTH_acq(&depth_frame, frames);

        // Load the depth image and transform it in a point cloud (pcl)
        PC_acq(&cloud, frames);

        // Start chrono time
        auto start = std::chrono::high_resolution_clock::now();

        // rs2_deproject_pixel_to_point(rgb_points, &intrin_rgb, rgb_pixels, rgb_aligned_distance);
        rs2::frame depth;
        rs2::stream_profile depth_profile;
        rs2::stream_profile color_profile;

        depth = frames.get_depth_frame();
        depth_profile = depth.get_profile();
        color_profile = frames.get_color_frame().get_profile();

        // Search along a projected beam from 0.5m to 10 meter
        
        auto depth_intrin = depth_profile.as<rs2::video_stream_profile>().get_intrinsics();
        auto color_intrin = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
        auto depth_extrin_to_color = depth_profile.as<rs2::video_stream_profile>().get_extrinsics_to(color_profile);
        auto color_extrin_to_depth = color_profile.as<rs2::video_stream_profile>().get_extrinsics_to(depth_profile);

        float depth_scale = 0.001;        // Device dipendend
        float from_pixel[2] = {(float) color_frame.cols/2, (float) color_frame.rows/2};  // x and y coordinates of the pixel that we want to transform

        float to_pixel[2];


        rs2_project_color_pixel_to_depth_pixel(to_pixel, reinterpret_cast<const uint16_t*>(depth.get_data()), depth_scale, 0.5, 10,
                                               &depth_intrin, &color_intrin,
                                               &color_extrin_to_depth, &depth_extrin_to_color, from_pixel);



        int ref_x = (int) to_pixel[0], ref_y = (int) to_pixel[1];
        pcl::PointXYZ refPnt = cloud->at(ref_x, ref_y);

        // Start chrono time
        auto stop = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << "Duration time : \t" << duration.count() << " us" << std::endl;
        
        
        // Rappresentation of the poit cloud
        PCViewer(cloud, viewer);

        // A rectangle is put on the image as a marker
        cv::rectangle(color_frame,cv::Point(color_frame.cols/2-5,color_frame.rows/2-5),
                      cv::Point(color_frame.cols/2+5,color_frame.rows/2+5),cv::Scalar(0,0,255),5);

        viewer->addCube(refPnt.x-0.01,refPnt.x+0.01,
                        refPnt.y-0.01,refPnt.y+0.01,
                        refPnt.z-0.01,refPnt.z+0.01,
                        1.0,0.0,0.0,"cube");

        // Rappresentation of the images from the camera
        cv::imshow("color",    color_frame);
        // cv::imshow("infrared", infrared_frame);
        // cv::imshow("depth",    depth_frame);

        // Show the viewer
        viewer -> spinOnce();

        // If the ESC button is pressed, the cycle is stopped and the program finishes
        if (cv::waitKey(1) == 27)
        {
            p.stop();
        }

        // Remove the point cloud from the viewer
        viewer -> removePointCloud("sample cloud");
        viewer -> removeShape("cube");

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