#include <iostream>
#include <chrono>
#include <assert.h>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> CloudT;
using namespace std;

// --------------------------------------------
// -------------3D Viewer Creators-------------
// --------------------------------------------
pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (0.001);
    viewer->initCameraParameters ();
    return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}


// --------------------------------------------
// ---------Point Cloud Transformation---------
// --------------------------------------------
CloudT::Ptr transform(CloudT::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients)
{
    CloudT::Ptr cloud_out (new CloudT);
    float n1 = -(*coefficients).values[0];
    float n2 = (*coefficients).values[1];
    float n3 = -(*coefficients).values[2];
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*cloud_in, *cloud_out, double_rotation);

    return cloud_out;
}


// --------------------------------------------
// ------------------ main --------------------
// --------------------------------------------
int main (int argc, char** argv)
{
    assert(argc>1);
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    CloudT::Ptr cloud_filtered (new CloudT), cloud_p (new CloudT), cloud_f (new CloudT);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_filtered, 0, 255, 0);

    // Fill in the cloud data
    pcl::PLYReader reader;
    
    reader.read (argv[1], *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 100 x 100 x 10 mm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (100, 100, 10);
    sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
    // viewer = simpleVis(cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // Write the downsampled version to disk
    pcl::PLYWriter writer;
    std::stringstream ss;
    ss << argv[1] << "_downsampled.ply";
    writer.write<pcl::PointXYZ> (ss.str(), *cloud_filtered, false);
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (5000); // this is the key parameter to estimate the proper plane
    seg.setDistanceThreshold (50); // points within +- 50mm from plane model are inliers
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0); // search for plane normal to Y axis
    seg.setAxis(axis);
    seg.setEpsAngle(M_PI/180*20); // plane can be within 10 degrees X-Z

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int i = 0, nr_points = (int) cloud_filtered->points.size();
    // While 30% of the original cloud is still there
    // while (cloud_filtered->points.size() > 0.5 * nr_points)
    while(i<1)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        std::cerr << "Plane " << i << " coefficients: " << endl
                  << (*coefficients).values[0] << endl
                  << (*coefficients).values[1] << endl
                  << (*coefficients).values[2] << endl
                  << (*coefficients).values[3] << endl;

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        // Save the inliers on disk
        std::stringstream ss;
        ss << "plane_" << i << ".ply";
        writer.write<pcl::PointXYZ> (ss.str(), *cloud_p);

        // Modify extraction attributes to keep the remaining part
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
        i++;
    }
    
    auto start = chrono::high_resolution_clock::now(); 
  
    // unsync the I/O of C and C++. 
    ios_base::sync_with_stdio(false); 
  
    CloudT::Ptr cloud_transformed = transform(cloud_filtered, coefficients);
  
    auto end = chrono::high_resolution_clock::now(); 
  
    // Calculating total time taken by the program. 
    double time_taken =  
      chrono::duration_cast<chrono::nanoseconds>(end - start).count(); 
  
    time_taken *= 1e-9; 
  
    cout << "Time taken by program is : " << fixed  
         << time_taken << setprecision(9); 
    cout << " sec" << endl; 

    viewer = simpleVis(cloud_transformed);
    
    int n = i;
    for (i=0; i<n; i++)
    {
        std::stringstream ss;
        ss << "plane_" << i << ".ply";
        reader.read (ss.str(), *cloud_filtered);
        cloud_transformed = transform(cloud_filtered, coefficients);
        viewer->addPointCloud(cloud_transformed, single_color);
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100); // wait for some microseconds, makes the viewer interactive
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}