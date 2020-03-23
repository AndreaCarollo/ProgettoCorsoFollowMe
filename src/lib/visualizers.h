#include "followme.h"
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::visualization::PCLVisualizer Visualizer;

// --------------------------------------------
// -------------3D Viewer Creators-------------
// --------------------------------------------
Visualizer::Ptr simpleVis (PntCld::ConstPtr cloud);

Visualizer::Ptr customColourVis (PntCld::ConstPtr cloud);