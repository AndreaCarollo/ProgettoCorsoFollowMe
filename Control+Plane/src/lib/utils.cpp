#include "utils.h"


// --------------------------------------------
// ---------------PCL Visualizers--------------
// --------------------------------------------
Visualizer::Ptr simpleVis (PntCld::ConstPtr cloud)
{
    Visualizer::Ptr viewer (new Visualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (0.001);
    viewer->initCameraParameters ();
    return (viewer);
}

Visualizer::Ptr customColourVis (PntCld::ConstPtr cloud)
{
    Visualizer::Ptr viewer (new Visualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

void downsampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, float* leaf)
{

    // Start chrono time
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud_in, *cloud2);

    // Stop chrono time
    auto stop = std::chrono::high_resolution_clock::now();
    
    // Duration
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    cout << endl << "Cloud conversion time : " << duration.count() << endl;

    // std::cerr << "PointCloud before filtering: " << cloud_in->width * cloud_in->height
    //             << " data points." << std::endl;
    pcl::PCLPointCloud2 cloud_tmp;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud2);
    sor.setLeafSize (leaf[0], leaf[1], leaf[2]);
    sor.filter (cloud_tmp);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(cloud_tmp, *cloud_out);

    // std::cerr << "PointCloud after filtering: " << cloud_out->width * cloud_out->height
    //             << " data points." << std::endl;

}