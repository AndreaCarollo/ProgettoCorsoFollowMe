#include "./rs_stream.h"


// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------
Stream::Stream(rs2::frameset *frames)
{
    // Internal parameters
    this->frames = (*frames);

    this->color = frames->get_color_frame();
    this->depth = frames->get_depth_frame();
    this->color_profile = this->color.get_profile();
    this->depth_profile = this->depth.get_profile();   

    this->depth_scale = 0.001;
}

void Stream::RGB_acq()
{
    int w = color.as<rs2::video_frame>().get_width();
    int h = color.as<rs2::video_frame>().get_height();
    
    // Convert the rs2 frame in a OpenCV Mat
    cv::Mat tmp( cv::Size(w, h), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);

    // Color conversion
    cv::cvtColor(tmp,this->color_frame,cv::COLOR_RGB2BGR);
}

void Stream::IR_acq()
{
    // Acquisition of the infrared frame
    rs2::video_frame infrared = frames.get_infrared_frame();

    int w = infrared.get_width();
    int h = infrared.get_height();
    
    // Convert the rs2 frame in a OpenCV Mat
    cv::Mat tmp( cv::Size(w, h), CV_8UC1, (void *) color.get_data(), cv::Mat::AUTO_STEP);
    this->infrared_frame = tmp;
    
}

/* Transform an object point in a point cloud */
PntCld::Ptr Stream::points_to_pcl(const rs2::points& points){

    // Create a point cloud object
    PntCld::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

void Stream::PC_acq(bool flag = false)
{
    // Initialization of some params
    rs2::pointcloud pc;
    rs2::points points;

    // Realsense point cloud generation (points object)
    pc.map_to(depth);
    points = pc.calculate(depth);

    // Transform the points object of rs2 in a point cloud of pcl
    this->cloud = points_to_pcl(points);

    if (flag){
        int w = depth.as<rs2::video_frame>().get_width();
        int h = depth.as<rs2::video_frame>().get_height();

        // Convert the rs2 frame in a OpenCV Mat
        cv::Mat tmp( cv::Size(w, h), CV_8UC3, (void *) depth.get_data(), cv::Mat::AUTO_STEP);
        this->depth_frame = tmp;
    }
    
}

void Stream::project_RGB2DEPTH(cv::Point *input)
{

    this->rgb_point = (*input);

    this->rgb_pixel[0] = (float) input->x/2;
    this->rgb_pixel[1] = (float) input->y/2;

    auto depth_intrin = this->depth_profile.as<rs2::video_stream_profile>().get_intrinsics();
    auto color_intrin = this->color_profile.as<rs2::video_stream_profile>().get_intrinsics();
    auto depth_extrin_to_color = this->depth_profile.as<rs2::video_stream_profile>().get_extrinsics_to(this->color_profile);
    auto color_extrin_to_depth = this->color_profile.as<rs2::video_stream_profile>().get_extrinsics_to(this->depth_profile);

    rs2_project_color_pixel_to_depth_pixel(this->depth_pixel, 
                                           reinterpret_cast<const uint16_t*>(this->depth.get_data()), 
                                           this->depth_scale, 0.5, 10,
                                           &depth_intrin, &color_intrin, 
                                           &color_extrin_to_depth, &depth_extrin_to_color, 
                                           this->rgb_pixel);
    
    this->depth_point = cv::Point((int) depth_pixel[0], (int) depth_pixel[1]);
    this->refPnt = this->cloud->at(this->depth_point.x, this->depth_point.y);

}