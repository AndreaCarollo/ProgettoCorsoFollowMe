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


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~ Class declarations ~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class Control{
    friend class Interface;
    
    public:

        AStar_mtx grid;
        class Interface *interface;
        class Plane* plane;
        pcl::PointXYZ refPnt;


        // ~~~~~~~~~~~ REMOVE IT ~~~~~~~~~~~
        // std::chrono::microseconds duration;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        Control(ConfigReader *p);
        void update(cv::Point* targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize);
        void update(cv::Point* targetPoint2D, Stream* stream);
        
    private:

        Position robot, target;
        float max_dist, low_threshold, up_threshold, target_threshold;
        int offset_from_targer;
        ushort obstacle_resolution;
        int grid_size;
        float scale, look_down;
        float distance_robot_target, distance_threshold;

        pcl::PointXYZ tmpPnt;
        
        bool path_planning;

        std::queue<AStar_cell*> frontier;

        AStar_cell *current;


        // ~~~~~~~~~~~ REMOVE THEM ~~~~~~~~~
        // float x_start, z_start;
        // pcl::PointXYZ Point;
        // std::chrono::_V2::system_clock::time_point start_add_plane, stop_add_plane;
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


        void obstacle_finding(PntCld::Ptr cloud);
        void A_star();
        void neighbors(AStar_cell* current);

};


#endif