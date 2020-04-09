// Double inclusion guard
#ifndef UTILS_H
#define UTILS_H

#include "followme.h"
#include "./rs_stream.h"

// Common used library
#include <iostream>
#include <thread>

// OpenCV Library
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::visualization::PCLVisualizer Visualizer;

// --------------------------------------------
// ----------------- Functions ----------------
// --------------------------------------------

/* Set the camera settings */
void camSettings(rs2::config *cfg);

void camSettings(rs2::config *cfg, int image_width, int image_heigth, int fps);

void PCViewer(PntCld::Ptr cloud, PntCldV::Ptr viewer);

Visualizer::Ptr simpleVis (PntCld::ConstPtr cloud);

Visualizer::Ptr customColourVis (PntCld::ConstPtr cloud);

void downsampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, float* leaf);

void down_sampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, int n);

void interfaceBuilding (cv::Mat *output_matrix, cv::Point targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize, 
                        float max_dist = 6000.0f, int obstacle_resolution = 54, float low_threshold = 10.0f, float up_threshold = 1500.0f );

void interfaceBuilding (cv::Mat *output_matrix, cv::Point* targetPoint2D, Stream* stream, 
                        float max_dist = 6000.0f, int obstacle_resolution = 54, float low_threshold = 10.0f, float up_threshold = 1500.0f );


#endif