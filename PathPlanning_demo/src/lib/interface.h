// Double inclusion guard
#ifndef INTERFACE_H
#define INTERFACE_H

#include "followme.h"
#include "configurator.h"
#include "PathPlanning.h"


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~ Class declarations ~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class Interface{
    friend class PathPlanning;       // specify who can access protected elements of this class

    public:
        static Interface* interfInstance;
        cv::Mat interface;

        static Interface* getInstance(ConfigReader *p);
        void update(class PathPlanning *plan);   // gets either the path or the arrow coords

    private:

        cv::Size interface_size;
        int gs, scale;
        cv::Point robot, target,
                arrow_tail, arrow_head;

        float m, dist_min, dist_max;

        float font_scale, transf, max_dist;
        int offset, r;
        cv::Scalar backgroundColor, obstacleColor, targetColor, robotColor, arrowColor;
        struct AStar_cell* tmp_cell;
        
        Interface(ConfigReader *p);
        Interface(const Interface& obj){}
    
        void put_arrow();
        void put_path(std::vector< std::vector<struct AStar_cell> >& grid, struct Position& target);
        void idle();
        void put_obstacle(int p_col, int p_row);
        void put_references();
        void clean();
};

#endif