#include "./utils.h"


// --------------- FUNCTIONS --------------- //


/* Set the camera settings */
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


/* Point cloud visualizer */
void PCViewer(PntCld::Ptr cloud, PntCldV::Ptr viewer){
    
    // Set all the parameters of the point cloud viewer and add to it the point cloud that we want to reppresent
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(0.001);    

}