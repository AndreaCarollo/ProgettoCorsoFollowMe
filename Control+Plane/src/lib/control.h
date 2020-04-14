// Double inclusion guard
#ifndef CONTROL
#define CONTROL

#include "./followme.h"
#include "./rs_stream.h"
#include "./configurator.h"

int AStarScale = 10;


typedef struct Cel_struct AStar_cel;

struct Cel_struct {

    int row;
    int col;
    int path_lenght;
    bool free = true;
    int visited = 0;
    AStar_cel *came_from;

};

typedef std::vector<AStar_cel*> AStar_mtx;


// --------------------------------------------
// -------------Class declarations-------------
// --------------------------------------------
class Control{
    public:

        cv::Mat interface;

        Control(ConfigReader *p, bool flag = false);
        void update(cv::Point* targetPoint2D, Stream* stream);
        void update(pcl::PointXYZ* refPnt, PntCld::Ptr PointCloud, cv::Size cvFrameSize);
        
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

        AStar_cel *start;
        AStar_cel *stop;
        
        AStar_cel *up;

        int max_row, max_col;
        AStar_mtx path;

        int x_rect,y_rect;
        AStar_cel *tmp_cel;

        void obstacle_finding(PntCld::Ptr cloud);
        void put_arrow();
        void A_star();
        bool search(AStar_cel* current_cel);

};


#endif