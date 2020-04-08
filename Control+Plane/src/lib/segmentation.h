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
        PntCld::Ptr plane_cloud;
        pcl::ModelCoefficients::Ptr coefficients;
        Eigen::Affine3f transf_mtx;
        Plane(Eigen::Vector3f* normal, float threshold, ushort angle, uint tries);
        // locate(Persona)
        // {
        //     prendi centroide persona
        //     trasforma con this.transf_mtx
        //     assegna a persona.punto3d
        // }
        void update(PntCld::Ptr cloud_in);  // put it into a loop of the main
    private:
        int tries;
        Eigen::Vector3f* normal;
        float threshold;
        ushort angle;
        void setTransfMtx();
};


#endif