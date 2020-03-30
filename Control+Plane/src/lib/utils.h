// Double inclusion guard
#ifndef UTILS_H
#define UTILS_H

#include "followme.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono> 

// New include
// #include <pcl/conversions.h>

typedef pcl::visualization::PCLVisualizer Visualizer;

// --------------------------------------------
// -------------3D Viewer Creators-------------
// --------------------------------------------
Visualizer::Ptr simpleVis (PntCld::ConstPtr cloud);

Visualizer::Ptr customColourVis (PntCld::ConstPtr cloud);

void downsampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, float* leaf);


#endif