#include "./acq_lib.h"


const int IMAGE_WIDTH  = 640;
const int IMAGE_HEIGTH = 480;
const int FRAME_RATE   =  30;

cv::Size IMAGE_SIZE = cv::Size(IMAGE_WIDTH, IMAGE_HEIGTH);


// --------------- FUNCTIONS --------------- //


/* Set the camera settings */
void camSettings(rs2::config *cfg){
    
    //Add desired streams to configuration
    cfg->enable_stream(RS2_STREAM_COLOR,    IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_BGR8, FRAME_RATE);
    cfg->enable_stream(RS2_STREAM_INFRARED, IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_Y8,   FRAME_RATE);
    cfg->enable_stream(RS2_STREAM_DEPTH,    IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_Z16,  FRAME_RATE);

}


/* Get the color frame from the camera and transform it in a OpenCV Mat */
void RGB_acq(cv::Mat *color_frame, rs2::frameset frames){
    
    // Align the color frame to the depth image
    /*
    rs2::align align_obj(RS2_STREAM_DEPTH);
    frames = align_obj.process(frames);
    */

    // Acquisition of the color frame
    rs2::video_frame color = frames.get_color_frame();
    
    // Convert the rs2 frame in a OpenCV Mat
    cv::Mat tmp( IMAGE_WIDTH, IMAGE_HEIGTH, CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);
    (*color_frame) = tmp;   // Potrebbe dare segmentation

}


/* Get the infrared frame from the camera and transform it in a OpenCV Mat */
void IR_acq(cv::Mat *infrared_frame, rs2::frameset frames){
    
    int w  = IMAGE_WIDTH;
    int h  = IMAGE_HEIGTH;

    // Acquisition of the infrared frame
    rs2::video_frame infrared = frames.get_infrared_frame();
    
    // Convert the rs2 frame in a OpenCV Mat
    cv::Mat tmp( IMAGE_WIDTH, IMAGE_HEIGTH, CV_8UC1, (void *) infrared.get_data(), cv::Mat::AUTO_STEP);
    (*infrared_frame) = tmp;

}


/* Get the depth frame from the camera and transform it in a OpenCV Mat */
void DEPTH_acq(cv::Mat *depth_frame, rs2::frameset frames){

    int w  = IMAGE_WIDTH;
    int h  = IMAGE_HEIGTH;

    // Acquisition of the depth frame
    rs2::video_frame depth = frames.get_depth_frame();

    // Convert the rs2 frame in a OpenCV Mat
    cv::Mat tmp( IMAGE_WIDTH, IMAGE_HEIGTH, CV_8UC3, (void *) depth.get_data(), cv::Mat::AUTO_STEP);
    (*depth_frame) = tmp;

}


/* Transform an object point in a point cloud */
PntCld points_to_pcl(const rs2::points& points){

    // Create a point cloud object
    PntCld cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Set all the paramethers of the point clouds
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = - ptr->x;
        p.y = - ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}


/* Transform a depth image to a point cloud */
PntCld PC_acq( rs2::frameset frames){

    // Acquisition of the depth frame
    rs2::video_frame depth    = frames.get_depth_frame();

    // Initialization of some params
    rs2::pointcloud pc;
    rs2::points points;

    // Realsense point cloud generation (points object)
    pc.map_to(depth);
    points = pc.calculate(depth);

    // Transform the points object of rs2 in a point cloud of pcl
    PntCld point_cloud = points_to_pcl(points);
}


/* Point cloud visualizer */
void PCViewer(PntCld cloud, PntCldV viewer){

    /* USE METHOD : 
        viewer-> initCameraParameters();                // initialize the camera parameters
        PCViewer(point_cloud, viewer);                  // Function call
        ...
        waitKey(1);
        ...
        viewer -> removePointCloud("sample cloud");     // Remove the cloud from the window
    */

    // Set all the parameters of the point cloud viewer and add to it the point cloud that we want to reppresent
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(0.001);
    // Show the viewer
    viewer -> spinOnce();

}