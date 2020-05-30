//  ______    _____    __       __        _____   ___          ___  __      __   ______
// |   ___|  /  _  \  |  |     |  |      /  _  \  \  \        /  / |   \  /   | |  ____| 
// |  |__   |  | |  | |  |     |  |     |  | |  |  \  \  /\  /  /  | |\ \/ /| | | |___
// |   __|  |  | |  | |  |     |  |     |  | |  |   \  \/  \/  /   | | \__/ | | |  ___|
// |  |     |  |_|  | |  |___  |  |___  |  |_|  |    \   /\   /    | |      | | | |____
// |__|      \_____/  |______| |______|  \_____/      \_/  \_/     |_|      |_| |______|
//
// This is the common project header file
// Declare and define here types and structures that are of common usage


// Double inclusion guard
#ifndef FOLLOWME_H
#define FOLLOWME_H

// System headers
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <vector>
#include <queue>
#include <chrono> // for time measurement

// Point Cloud headerseu
// #include <pcl/io/ply_io.h>
// #include <pcl/point_types.h>

// OpenCV Library
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// Realsense Library
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_frame.hpp>

// PCL
// #include <pcl/common/common_headers.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/console/parse.h>
// #include <pcl/filters/voxel_grid.h>

// Homogeneous transformation
// #include <pcl/common/transforms.h>

// SAC plane segmentation
// #include <pcl/ModelCoefficients.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/extract_indices.h>


// Data types
typedef uint32_t index_t; // for indexes
// typedef pcl::PointCloud<pcl::PointXYZ> PntCld;
// typedef pcl::visualization::PCLVisualizer PntCldV;


#endif