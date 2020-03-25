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

void downsampling(pcl::PCLPointCloud2::Ptr cloud_in, PntCld::Ptr cloud_out, float* leaf)
{
    std::cerr << "PointCloud before filtering: " << cloud_in->width * cloud_in->height
                << " data points." << std::endl;
    pcl::PCLPointCloud2 cloud_tmp;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_in);
    sor.setLeafSize (leaf[0], leaf[1], leaf[2]);
    sor.filter (cloud_tmp);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (cloud_tmp, *cloud_out);

    std::cerr << "PointCloud after filtering: " << cloud_out->width * cloud_out->height
                << " data points." << std::endl;
}