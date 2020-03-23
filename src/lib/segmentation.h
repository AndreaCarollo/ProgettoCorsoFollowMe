// Double inclusion guard
#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "followme.h"
// for homogeneous transformation
#include <pcl/common/transforms.h>
// for SAC plane segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>



// --------------------------------------------
// ---------Point Cloud Transformation---------
// --------------------------------------------
PntCld::Ptr transform(PntCld::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients);


#endif