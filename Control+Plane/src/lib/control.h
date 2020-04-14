// Double inclusion guard
#ifndef CONTROL
#define CONTROL

#include "./followme.h"
#include "./rs_stream.h"
#include "./configurator.h"


// --------------------------------------------
// -------------Class declarations-------------
// --------------------------------------------
class Control{
    public:

        cv::Mat interface;

        Control(ConfigReader *p, bool flag);
        void update(cv::Point* targetPoint2D, Stream* stream);
        // Test Phase
        void update(cv::Point* targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize);
        

    private:

        cv::Size interface_size;
        int r, offset;
        double font_scale;
        float max_dist, low_threshold, up_threshold;
        ushort obstacle_resolution;
        float scale;
        cv::Scalar backgroundColor, obstacleColor, targetColor, robotColor, arrowColor;

        bool flag, there_is_an_obstacle;

        int x_robot, y_robot;

        float tmp, m;
        int x_target,y_target;
        int x1_arrow, x2_arrow, y1_arrow, y2_arrow;

        double dist_rt, dist_max, dist_min;

        void obstacle_finding(PntCld::Ptr cloud);
        void put_arrow();
        void A_star();

};


#endif