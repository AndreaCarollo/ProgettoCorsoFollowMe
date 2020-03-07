
#ifndef ACQUISITION_LIB_H_
#define ACQUISITION_LIB_H_


// Common used library
#include <iostream>
#include <thread>

// OpenCV Library
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// Realsense Library
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_frame.hpp>

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


// Definition of a new type variable for the point cloud
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PntCld;

// Definition of a new type variable for the point cloud viewer
typedef pcl::visualization::PCLVisualizer::Ptr PntCldV;



// --------------- FUNCTIONS --------------- //

/* Set the camera settings */
void camSettings(rs2::config *cfg);


/* Get the color frame from the camera and transform it in a OpenCV Mat */
void RGB_acq(cv::Mat *color, rs2::frameset frames);


/* Get the infrared frame from the camera and transform it in a OpenCV Mat */
void IR_acq(cv::Mat *infrared, rs2::frameset frames);


/* Get the depth frame from the camera and transform it in a OpenCV Mat */
void DEPTH_acq(cv::Mat *depth, rs2::frameset frames);


/* Transform an object point in a point cloud */
PntCld points_to_pcl(const rs2::points& points);


/* Get the depth image from the camera and transform it in a point cloud */
PntCld PC_acq( rs2::frameset frames);


/* Point cloud visualizer */
void PCViewer(PntCld cloud, PntCldV viewer);


#endif




