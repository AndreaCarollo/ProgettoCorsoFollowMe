#include "segmentation.h"

// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------
Plane::Plane(PntCld::Ptr cloud_in, int16_t threshold, Eigen::Vector3f normal, int16_t angle)
{
    PntCld::Ptr cloud_tmp (new PntCld);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients());
    plane_cloud = PntCld::Ptr (new PntCld);
    // Create and set the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (5000); // this is the key parameter to estimate the proper plane
    seg.setDistanceThreshold (threshold);
    seg.setAxis(normal);
    seg.setEpsAngle(M_PI/180*angle); // plane can be within +- 20 deg X-Z

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    // Run the segmentation
    seg.setInputCloud (cloud_in);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
    }
    
    // Extract the inliers
    extract.setInputCloud (cloud_in);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane_cloud);
    
    // Remove the plane from the input pcl
    extract.setNegative (true);
    extract.filter (*cloud_tmp);
    cloud_in.swap (cloud_tmp);

    // Set transformation mtx
    this->setTransfMtx();

}


void Plane::setTransfMtx()
{
    float n1  = -coefficients->values[0];
    float n2  = coefficients->values[1];
    float n3  = -coefficients->values[2];
    float den = sqrt(pow(n2,2)+pow(n3,2));

    // Set the transformation object
    transf_mtx = Eigen::Matrix4f::Identity();
    transf_mtx (0,0) = den;
    transf_mtx (1,0) = -n2*n1/den;
    transf_mtx (2,0) = -n3*n1/den;
    transf_mtx (0,1) = -n1;
    transf_mtx (1,1) = -n2;
    transf_mtx (2,1) = -n3;
    transf_mtx (1,2) = n3/den;
}


PntCld::Ptr transform(PntCld::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients)
{
    PntCld::Ptr cloud_out (new PntCld);
    float n1  = -(*coefficients).values[0];
    float n2  = (*coefficients).values[1];
    float n3  = -(*coefficients).values[2];
    float den = sqrt(pow(n2,2)+pow(n3,2));

    // Build the transformation object
    Eigen::Matrix4f double_rotation = Eigen::Matrix4f::Identity();
    double_rotation (0,0) = den;
    double_rotation (1,0) = -n2*n1/den;
    double_rotation (2,0) = -n3*n1/den;
    double_rotation (0,1) = -n1;
    double_rotation (1,1) = -n2;
    double_rotation (2,1) = -n3;
    double_rotation (1,2) = n3/den;

    // Execute the transformation
    pcl::transformPointCloud(*cloud_in, *cloud_out, double_rotation);

    return cloud_out;
}
