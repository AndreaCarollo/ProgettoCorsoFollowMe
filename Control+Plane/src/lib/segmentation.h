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
#include <pcl/filters/extract_indices.h>


// --------------------------------------------
// -------------Class declarations-------------
// --------------------------------------------
class Plane{
    public:
        pcl::ModelCoefficients::Ptr coefficients;
        Eigen::Affine3f transf_mtx;
        Plane(Eigen::Vector3f* normal, float threshold, ushort angle);
        void update(PntCld::Ptr cloud_in);  // loopable
    private:
        int tries;
        Eigen::Vector3f* normal;
        float threshold;
        ushort angle;
        void setTransfMtx();
};


#endif