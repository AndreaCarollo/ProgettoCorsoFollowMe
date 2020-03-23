#include "segmentation.h"

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
    PntCld::Ptr transformed_cloud (new PntCld());
    pcl::transformPointCloud(*cloud_in, *cloud_out, double_rotation);

    return cloud_out;
}
