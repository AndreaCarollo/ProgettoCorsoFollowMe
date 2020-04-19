// Double inclusion guard
#ifndef UTILS_H
#define UTILS_H

#include "followme.h"
#include "rs_stream.h"
#include "configurator.h"


typedef pcl::visualization::PCLVisualizer Visualizer;


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~ UTIL FUNCTIONS ~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void camSettings(rs2::config *cfg);

void camSettings(rs2::config *cfg, int image_width, int image_heigth, int fps);

void PCViewer(PntCld::Ptr cloud, PntCldV::Ptr viewer);

Visualizer::Ptr simpleVis (PntCld::ConstPtr cloud);

Visualizer::Ptr customColourVis (PntCld::ConstPtr cloud);

void downsampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, float* leaf);

void down_sampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, int n);



#endif