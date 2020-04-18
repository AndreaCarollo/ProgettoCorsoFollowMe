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

        Control(ConfigReader *p, bool flag = false);
        void update(pcl::PointXYZ* refPnt, PntCld::Ptr PointCloud, cv::Size cvFrameSize, Plane* plane);
        void update(cv::Point* targetPoint2D, Stream* stream, Plane* plane);
        
    private:

        Position robot, target;
        float max_dist, low_threshold, up_threshold;
        ushort obstacle_resolution;
        int grid_size;
        float scale;
        
        bool path_planning, there_is_an_obstacle;

        pcl::PointXYZ tmp_pnt;

        AStar_cell *start;
        AStar_cell *stop;

        std::queue<AStar_cell*> frontier;

        int x_rect,y_rect;
        AStar_cell *current;

        void obstacle_finding(PntCld::Ptr cloud, Plane* plane);
        void A_star();
        void neighbors(AStar_cell* current);

};


#endif