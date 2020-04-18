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
        void update();  // gets either the path or the arrow coords

    private:
        cv::Size interface_size;
        int x_robot, y_robot;
        float font_scale;
        int offset, r;
        cv::Scalar backgroundColor, obstacleColor, targetColor, robotColor, arrowColor;
        
        Interface(ConfigReader *p);
        Interface(const Interface& obj){}
    
        void put_arrow(struct Position& robot, struct Position& target);
        void put_path();
};

#endif