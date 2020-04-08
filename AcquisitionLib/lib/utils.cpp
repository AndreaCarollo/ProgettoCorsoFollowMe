#include "./utils.h"

const int IMAGE_WIDTH  = 640;
const int IMAGE_HEIGTH = 480;
const int FRAME_RATE   =  30;

cv::Size IMAGE_SIZE = cv::Size(IMAGE_WIDTH, IMAGE_HEIGTH);


// --------------- FUNCTIONS --------------- //


/* Set the camera settings */
void camSettings(rs2::config *cfg){
    
    // Add desired streams to configuration
    cfg->enable_stream(RS2_STREAM_COLOR,    IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_BGR8, FRAME_RATE);
    cfg->enable_stream(RS2_STREAM_INFRARED, IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_Y8,   FRAME_RATE);
    cfg->enable_stream(RS2_STREAM_DEPTH,    IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_Z16,  FRAME_RATE);

}


/* Point cloud visualizer */
void PCViewer(PntCld::Ptr cloud, PntCldV::Ptr viewer){
    
    // Set all the parameters of the point cloud viewer and add to it the point cloud that we want to reppresent
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(0.001);    

}