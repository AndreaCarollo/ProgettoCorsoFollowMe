#include "segmentation.h"

// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------
Plane::Plane(Eigen::Vector3f* normal, float threshold, ushort angle)
{
    // Internal parameters
    tries = 5000;
    this->normal = normal;
    this->threshold = threshold;
    this->angle = angle;
    coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients());
    plane_cloud = PntCld::Ptr (new PntCld);
}

void Plane::setTransfMtx()
{
    float n1  = -coefficients->values[0];
    float n2  = coefficients->values[1];
    float n3  = -coefficients->values[2];
    float den = sqrt(pow(n2,2)+pow(n3,2));

    // Set the transformation object
    transf_mtx = Eigen::Affine3f::Identity();
    transf_mtx (0,0) = den;
    transf_mtx (1,0) = -n2*n1/den;
    transf_mtx (2,0) = -n3*n1/den;
    transf_mtx (0,1) = -n1;
    transf_mtx (1,1) = -n2;
    transf_mtx (2,1) = -n3;
    transf_mtx (1,2) = n3/den;   
    transf_mtx (1,3) = -coefficients->values[3];
}

void Plane::update(PntCld::Ptr cloud_in)
{
    PntCld::Ptr cloud_tmp (new PntCld);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

    // Create and set the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (tries);               // max iterations for plane search (internal parameter)
    seg.setDistanceThreshold (threshold);       // points within +- threshold are inliers
    seg.setAxis(*normal);
    seg.setEpsAngle(M_PI/180*angle);            // plane can deviate of +- 20deg in the other two axis

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    // Run the segmentation
    seg.setInputCloud (cloud_in);
    seg.segment (*inliers, *coefficients);
    if(inliers->indices.size() == 0)            // break the code if plane is not found
        return;
    
    setTransfMtx();

    // Extract the inliers/plane
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane_cloud);

    // std::cerr << "PointCloud representing the planar component: " 
    //           << plane_cloud->width * plane_cloud->height << " data points." << std::endl;
    
    // Remove the plane from the input pcl
    extract.setNegative (true);
    extract.filter (*cloud_tmp);
    cloud_in.swap (cloud_tmp);
}