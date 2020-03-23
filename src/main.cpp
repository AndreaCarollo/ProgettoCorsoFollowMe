#include "./lib/followme.h"
#include "./lib/visualizers.h"
#include "./lib/segmentation.h"


using namespace std;



// --------------------------------------------
// ------------------ main --------------------
// --------------------------------------------
int main (int argc, char** argv)
{
    assert(argc>1);
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    PntCld::Ptr cloud_filtered (new PntCld), cloud_p (new PntCld), cloud_f (new PntCld);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    Visualizer::Ptr viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_filtered, 0, 255, 0);

    // Fill in the cloud data
    pcl::PLYReader reader;
    
    reader.read (argv[1], *cloud_blob);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height
                << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 100 x 100 x 10 mm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_blob);
    sor.setLeafSize (100, 100, 10);
    sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
                << " data points." << std::endl;

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
    seg.setEpsAngle(M_PI/180*20); // plane can be within +- 20 deg X-Z

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return (-1);
    }

    std::cerr << "Plane coefficients: " << endl
                << (*coefficients).values[0] << endl
                << (*coefficients).values[1] << endl
                << (*coefficients).values[2] << endl
                << (*coefficients).values[3] << endl;

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " 
                << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Save the inliers on disk
    writer.write<pcl::PointXYZ> ("../plane.ply", *cloud_p);

    // Modify extraction object to keep the pcl without the plane
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);

    // Execute transformation
    PntCld::Ptr cloud_transformed = transform(cloud_filtered, coefficients);

    // Visualize
    viewer = simpleVis(cloud_transformed);
    cloud_transformed = transform(cloud_p, coefficients);
    viewer->addPointCloud(cloud_transformed, color_handler);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100); // wait for some microseconds, makes the viewer interactive
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}