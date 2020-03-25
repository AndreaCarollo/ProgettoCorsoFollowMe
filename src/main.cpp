#include "./lib/followme.h"
#include "./lib/utils.h"
#include "./lib/segmentation.h"


using namespace std;



// --------------------------------------------
// ------------------ main --------------------
// --------------------------------------------
int main (int argc, char** argv)
{
    assert(argc>1);
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2);
    PntCld::Ptr cloud_filtered (new PntCld), cloud_tmp (new PntCld);
    Visualizer::Ptr viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_filtered, 0, 255, 0);
    pcl::PLYReader reader;
    pcl::PLYWriter writer;
    std::stringstream ss;
    Eigen::Vector3f normal = Eigen::Vector3f(0.0, 1.0, 0.0);
    int16_t threshold = 50, angle = 20;

    // Fill in the cloud data
    reader.read (argv[1], *cloud_blob);

    // Downsample the original cloud, requires a filtering object
    float leaf[3];                      // with same scale of pcl
    leaf[0] = 100; leaf[1] = 100; leaf[2] = 10;
    downsampling(cloud_blob, cloud_filtered, leaf);

    // Write the downsampled version to disk
    ss << argv[1] << "_downsampled.ply";
    writer.write<pcl::PointXYZ> (ss.str(), *cloud_filtered, false);
    
    // Extract plane creating plane object
    Plane plane(cloud_filtered, 50, normal, 20);

    std::cerr << "PointCloud representing the planar component: " 
                << plane.plane_cloud->width * plane.plane_cloud->height << " data points." << std::endl;

    std::cerr << "Plane coefficients: " << endl
                << plane.coefficients->values[0] << endl
                << plane.coefficients->values[1] << endl
                << plane.coefficients->values[2] << endl
                << plane.coefficients->values[3] << endl;

    // Save the inliers on disk
    writer.write<pcl::PointXYZ> ("../plane.ply", *plane.plane_cloud);

    

    // Apply transformation mtx to plane and pcl
    pcl::transformPointCloud(*cloud_filtered, *cloud_tmp, plane.transf_mtx);
    cloud_filtered.swap (cloud_tmp);
    pcl::transformPointCloud(*plane.plane_cloud, *cloud_tmp, plane.transf_mtx);
    plane.plane_cloud.swap (cloud_tmp);

    // Visualize
    viewer = simpleVis(cloud_filtered);
    viewer->addPointCloud(plane.plane_cloud, color_handler);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100); // wait for some microseconds, makes the viewer interactive
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return (0);
}