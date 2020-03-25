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
        Eigen::Matrix4f transf_mtx;

        Plane(PntCld::Ptr cloud_in, int16_t threshold,
                Eigen::Vector3f normal, int16_t angle);
        // locate(Persona)
        // {
        //     prendi centroide persona
        //     trasforma con this.transf_mtx
        //     assegna a persona.punto3d
        // }
    private:
        void setTransfMtx();
};


#endif