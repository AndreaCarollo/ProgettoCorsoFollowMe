// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <iostream> // for cout

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char *argv[]) try
{
    cv::Mat framebgr,frame_i;
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::video_frame infrared = frames.get_infrared_frame();
        rs2::video_frame color = frames.get_color_frame();

        // Get the depth frame's dimensions
        float w_i = infrared.get_width();
        float h_i = infrared.get_height();

        // Get the depth frame's dimensions
        int w_c = color.get_width();
        int h_c = color.get_height();

        cv::Mat color_frame(cv::Size(w_c, h_c), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat infrared_frame(cv::Size(w_i, h_i), CV_8UC3, (void *)infrared.get_data(), cv::Mat::AUTO_STEP);

        cv::cvtColor(color_frame, framebgr, cv::COLOR_RGB2BGR);
        cv::cvtColor(infrared_frame, frame_i, cv::COLOR_RGB2BGR);

        cv::imshow("color", framebgr);
        cv::imshow("infrared", frame_i);
        cv::waitKey(1);
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