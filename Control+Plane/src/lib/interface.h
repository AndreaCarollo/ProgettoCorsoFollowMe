// Double inclusion guard
#ifndef INTERFACE_H
#define INTERFACE_H

#include "followme.h"
#include "configurator.h"
#include "control.h"


class Interface{
    friend class Control;       // specify who can access protected elements of this class

    public:
        static Interface* interfInstance;
        cv::Mat interface;

        static Interface* getInstance(ConfigReader *p);
        void update(pcl::PointXYZ* refPnt);  // gets either the path or the arrow coords

    private:

        cv::Size interface_size;
        int gs, scale;
        int x_robot, y_robot, x_target, y_target;

        int x1_arrow, x2_arrow, y1_arrow, y2_arrow;
        float m, dist_min, dist_max;

        float font_scale, transf, max_dist;
        int offset, r;
        cv::Scalar backgroundColor, obstacleColor, targetColor, robotColor, arrowColor;
        struct AStar_cell* tmp_cell;
        
        Interface(ConfigReader *p);
        Interface(const Interface& obj){}
    
        void put_arrow();
        // void put_path(AStar_mtx grid, struct Position& target);
        void put_path(std::vector< std::vector<struct AStar_cell> >& grid, struct Position& target);
        void put_obstacle(int p_col, int p_row);
        void put_references();
};

#endif