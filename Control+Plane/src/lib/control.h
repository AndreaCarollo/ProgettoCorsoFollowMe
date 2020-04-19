// Double inclusion guard
#ifndef CONTROL
#define CONTROL

#include "followme.h"
#include "rs_stream.h"
#include "segmentation.h"
#include "configurator.h"
#include "interface.h"

struct Position {
    int row;
    int col;
};

struct AStar_cell {
    bool free;
    bool visited;
    int path_lenght;
    AStar_cell *came_from;
    Position cell;
};


typedef std::vector< std::vector<struct AStar_cell> > AStar_mtx;


// --------------------------------------------
// -------------Class declarations-------------
// --------------------------------------------
class Control{
    public:

        AStar_mtx grid;
        class Interface *interface;
        class Plane* plane;
        pcl::PointXYZ refPnt;

        Control(ConfigReader *p);
        void update(cv::Point* targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize);
        void update(cv::Point* targetPoint2D, Stream* stream);
        
    private:

        Position robot, target;
        float max_dist, low_threshold, up_threshold, target_threshold;
        int offset_from_targer;
        ushort obstacle_resolution;
        int grid_size;
        float scale;
        float distance_robot_target, distance_threshold;
        
        bool path_planning, there_is_an_obstacle;

        pcl::PointXYZ tmp_pnt;

        AStar_cell *start;
        AStar_cell *stop;

        std::queue<AStar_cell*> frontier;

        int x_rect,y_rect;
        AStar_cell *current;

        void obstacle_finding(PntCld::Ptr cloud);
        void A_star();
        void neighbors(AStar_cell* current);

};


#endif