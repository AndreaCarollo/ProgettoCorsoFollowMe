#include "utils.h"


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~ UTIL FUNCTIONS ~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void camSettings(rs2::config *cfg, ConfigReader* p){

    int rgb_width, rgb_heigth, ir_width, ir_heigth, fps;
    p->getValue("RGB_WIDTH", rgb_width);
    p->getValue("RGB_HEIGTH", rgb_heigth);
    p->getValue("IR_WIDTH", ir_width);
    p->getValue("IR_HEIGTH", ir_heigth);
    p->getValue("FPS", fps);

    
    // Add desired streams to configuration
    cfg->enable_stream(RS2_STREAM_COLOR,    rgb_width, rgb_heigth, RS2_FORMAT_BGR8, fps);
    cfg->enable_stream(RS2_STREAM_INFRARED, ir_width,  ir_heigth,  RS2_FORMAT_Y8,   fps);
    cfg->enable_stream(RS2_STREAM_DEPTH,    ir_width,  ir_heigth,  RS2_FORMAT_Z16,  fps);

}

void camSettings(rs2::config *cfg, int image_width, int image_heigth, int fps){
    
    // Add desired streams to configuration
    cfg->enable_stream(RS2_STREAM_COLOR,    image_width, image_heigth, RS2_FORMAT_BGR8, fps);
    cfg->enable_stream(RS2_STREAM_INFRARED, image_width, image_heigth, RS2_FORMAT_Y8,   fps);
    cfg->enable_stream(RS2_STREAM_DEPTH,    image_width, image_heigth, RS2_FORMAT_Z16,  fps);

}

void PCViewer(PntCld::Ptr cloud, PntCldV::Ptr viewer){
    
    // Set all the parameters of the point cloud viewer and add to it the point cloud that we want to reppresent
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // viewer->addCoordinateSystem(0.001);    

}

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

void downsampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, float* leaf)
{
    /*
    This downsampling function uses a filter to reduce
    the point cloud resolution (takes some time)
    */

    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud_in, *cloud2);

    pcl::PCLPointCloud2 cloud_tmp;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud2);
    sor.setLeafSize (leaf[0], leaf[1], leaf[2]);
    sor.filter (cloud_tmp);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(cloud_tmp, *cloud_out);
}


void down_sampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, int n)
{
    /*
    This downsampling function takes, simply, a point every 
    n points (a lot of faster than the previous)
    */

    size_t i = 0.7*cloud_in->size();
    do{
        cloud_out->points.push_back(cloud_in->at(i));
        i +=  n;
    }while( i < cloud_in->size());
}

