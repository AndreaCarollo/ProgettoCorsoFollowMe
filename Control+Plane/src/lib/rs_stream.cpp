#include "./rs_stream.h"


// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------
Stream::Stream(std::string stream_name, rs2::frameset *frames, ConfigReader* cfg)
{
    cfg->getValue("ACQUISITION_LEAF", leaf);

    this->stream_name = stream_name;
    this->depth_scale = 0.001;

    update(frames);
    rs2::frame* infrared = NULL;

    this->color_frame = cv::Mat();
    this->depth_frame = cv::Mat();
    this->infrared_frame = cv::Mat();
    this->tmp = cv::Mat();

    w_RGB = color.as<rs2::video_frame>().get_width();
    h_RGB = color.as<rs2::video_frame>().get_height();
    w_IR  = depth.as<rs2::video_frame>().get_width();
    h_IR  = depth.as<rs2::video_frame>().get_height();

    cloud = PntCld::Ptr (new PntCld);
    cloud->width = w_IR/leaf;
    cloud->height = h_IR;
    cloud->is_dense = false;
    cloud->points.resize(w_IR*h_IR/leaf);

    this->color_profile = this->color.get_profile();
    this->depth_profile = this->depth.get_profile();
}

void Stream::update(rs2::frameset *frames)
{
    // Internal parameters
    this->frames = (*frames);

    this->color = frames->get_color_frame();
    this->depth = frames->get_depth_frame();

}

void Stream::RGB_acq()
{
    // Convert the rs2 frame in a OpenCV Mat
    tmp = cv::Mat( cv::Size(w_RGB, h_RGB), CV_8UC3, (void *) color.get_data(), cv::Mat::AUTO_STEP);

    // Color conversion
    cv::cvtColor(tmp,this->color_frame,cv::COLOR_RGB2BGR);
}

void Stream::IR_acq()
{
    // Acquisition of the infrared frame
    infrared = frames.get_infrared_frame();
    
    // Convert the rs2 frame in a OpenCV Mat
    this->infrared_frame = cv::Mat( cv::Size(w_IR, h_IR), CV_8UC1, (void *) color.get_data(), cv::Mat::AUTO_STEP);
    
}

/* Transform an object point in a point cloud */
void Stream::points_to_pcl()
{
    // Set all the paramethers of the point clouds
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = - ptr->x;
        p.y = - ptr->y;
        p.z = ptr->z;
        ptr += leaf;
    }
}

void Stream::PC_acq(bool flag)
{
    // Realsense point cloud generation (points object)
    points = pc.calculate(depth);

    // Transform the points object of rs2 in a point cloud of pcl
    points_to_pcl();

    if (flag){
        // Convert the rs2 frame in a OpenCV Mat
        this->depth_frame = cv::Mat (cv::Size(w_IR, h_IR), CV_8UC3, (void *) depth.get_data(), cv::Mat::AUTO_STEP);
    }
}

void Stream::project_RGB2DEPTH(cv::Point *input)
{
    this->rgb_pixel[0] = (float) input->x;
    this->rgb_pixel[1] = (float) input->y;

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
    
    // this->refPnt = cloud->points[(int) ((( depth_pixel[1] - 1 ) * w_IR + depth_pixel[0])/leaf)];
    this->refPnt = cloud->at((((int) depth_pixel[1] - 1) * w_IR + (int) depth_pixel[0])/leaf);
}