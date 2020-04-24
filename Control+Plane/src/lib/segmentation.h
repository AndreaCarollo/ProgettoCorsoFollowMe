// Double inclusion guard
#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "followme.h"
#include "configurator.h"


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~ Class declarations ~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class Plane{
    friend class Control;       // specify who can access protected elements of this class

    public:
        static Plane* segInstance;
        // only for debug use
        PntCld::Ptr plane_cloud;
        PntCld::Ptr easy_cloud;     // containing the cloud without the plane

        pcl::PointIndices::Ptr inliers;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr coefficients;
        Eigen::Affine3f transf_mtx;
        
        static Plane* getInstance(ConfigReader *p);
        void update(PntCld::Ptr cloud_in);  // put it into a loop of the main

    private:
        int tries, max_tries, min_tries;
        Eigen::Vector3f normal;
        float threshold, incrementFactor;
        ushort angle;
        ushort leaf;
        float look_down;

        Plane(ConfigReader *p);
        Plane(const Plane& obj){}
        void setTransfMtx();
        void downsample(PntCld::Ptr cloud_in);
};


#endif