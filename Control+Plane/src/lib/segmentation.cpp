#include "segmentation.h"

// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------

Plane* Plane::segInstance = NULL;

Plane::Plane(ConfigReader *p)
{
    plane_cloud = PntCld::Ptr (new PntCld);
    easy_cloud = PntCld::Ptr (new PntCld);
    
    inliers = pcl::PointIndices::Ptr (new pcl::PointIndices());
    coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients());
    transf_mtx = Eigen::Affine3f::Identity();

    // Internal parameters from the configurator
    p->getValue("RANSAC_MAX_ITER", tries);
    p->getValue("PLANE_NORMAL", normal);
    p->getValue("PLANE_THRESHOLD", threshold);
    p->getValue("PLANE_ANGLE", (int&) angle);
    p->getValue("PLANE_LEAF", (int&) leaf);
    p->getValue("PLANE_LOOK_DOWN", look_down);

    // Set the segmentation object
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (tries);
    seg.setDistanceThreshold (threshold);
    seg.setAxis(normal);
    seg.setEpsAngle(M_PI/180*angle);
    seg.setOptimizeCoefficients(true);  // optional

}

Plane* Plane::getInstance(ConfigReader *parser)
{
    // No need to use double re-check lock mechanism here
    // because this getInstance() will call at the time of
    // initialization only and mostly, at the time of
    // initialization, there will be only one thread.
    if(segInstance == NULL)
        segInstance = new Plane(parser);

    return segInstance;
}

void Plane::setTransfMtx()
{
    float n1  = -coefficients->values[0];
    float n2  = coefficients->values[1];
    float n3  = -coefficients->values[2];
    float den = sqrt(pow(n2,2)+pow(n3,2));

    // Set the transformation object
    transf_mtx (0,0) = den;
    transf_mtx (1,0) = -n2*n1/den;
    transf_mtx (2,0) = -n3*n1/den;
    transf_mtx (0,1) = -n1;
    transf_mtx (1,1) = -n2;
    transf_mtx (2,1) = -n3;
    transf_mtx (1,2) = n3/den;   
    transf_mtx (1,3) = -coefficients->values[3];
}

void Plane::downsample(PntCld::Ptr cloud_in)
{
    easy_cloud->clear();
    size_t i = look_down*cloud_in->size();
    do{
        easy_cloud->points.push_back(cloud_in->at(i));
        i +=  leaf;
    }while( i < cloud_in->size());
}

void Plane::update(PntCld::Ptr cloud_in)
{
    // PntCld::Ptr cloud_tmp (new PntCld);
    downsample(cloud_in);   // generates easy_cloud
    
    // Run the segmentation
    seg.setInputCloud (easy_cloud);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() == 0)            // break the code if plane is not found
        return;
    
    setTransfMtx();
    
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;     // still thr bcs is for debug only

    // Extract the inliers (plane)
    extract.setInputCloud (easy_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane_cloud);

}