// Double inclusion guard
#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "followme.h"
#include "configurator.h"
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
        // only for debug use
        PntCld::Ptr plane_cloud;
        PntCld::Ptr easy_cloud;     // containing the cloud without the plane

        pcl::ModelCoefficients::Ptr coefficients;
        Eigen::Affine3f transf_mtx;
        Plane(ConfigReader *p);
        // locate(Persona)
        // {
        //     prendi centroide persona
        //     trasforma con this.transf_mtx
        //     assegna a persona.punto3d
        // }
        void update(PntCld::Ptr cloud_in);  // put it into a loop of the main
    private:
        int tries;
        Eigen::Vector3f normal;
        float threshold;
        ushort angle;
        ushort leaf;
        float look_down;
        void setTransfMtx();
        void downsample(PntCld::Ptr cloud_in);
};


#endif