// Double inclusion guard
#ifndef RS_STREAM
#define RS_STREAM

#include "./followme.h"


// --------------------------------------------
// -------------Class declarations-------------
// --------------------------------------------
class Stream{
    public:
        rs2::frameset frames;

        cv::Mat color_frame;
        cv::Mat infrared_frame;
        cv::Mat depth_frame;

        PntCld::Ptr cloud;

        cv::Point rgb_point;
        cv::Point depth_point;
        pcl::PointXYZ refPnt;

        Stream(rs2::frameset *frames);
        void RGB_acq();
        void IR_acq();
        void PC_acq(bool flag);
        void project_RGB2DEPTH(cv::Point *input);

    private:
        rs2::frame depth;
        rs2::frame color;
        rs2::stream_profile depth_profile;
        rs2::stream_profile color_profile;

        float depth_scale;

        float rgb_pixel[2];
        float depth_pixel[2];

        PntCld::Ptr points_to_pcl(const rs2::points& points);
};


#endif