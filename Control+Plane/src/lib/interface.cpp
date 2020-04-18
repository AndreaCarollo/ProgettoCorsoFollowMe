#include "interface.h"


// --------------------------------------------
// ---------------Class methods----------------
// --------------------------------------------

Interface* Interface::interfInstance = NULL;

Interface::Interface(ConfigReader *p)
{
    p->getValue("FONT_SCALE", font_scale);
    p->getValue("BG_COLOR", backgroundColor);
    p->getValue("ARROW_COLOR", arrowColor);
    p->getValue("OBSTACLE_COLOR", obstacleColor);
    p->getValue("ROBOT_COLOR", robotColor);
    p->getValue("TARGET_COLOR", targetColor);

    int sz;
    p->getValue("INTERFACE_SIZE", sz);
    interface_size = cv::Size(sz,sz);           // human graphic interface size
    x_robot = interface_size.width/2;           // Position of the robot in the graphic interface
    y_robot = interface_size.height - offset;
    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    r = 15;                                         // r      -> Circle radius for robot and target
    offset = 18;                                    // offset -> Offset distance (some differene graphic uses)
}

Interface* Interface::getInstance(ConfigReader *parser)
{
    // No need to use double re-check lock mechanism here
    // because this getInstance() will call at the time of
    // initialization only and mostly, at the time of
    // initialization, there will be only one thread.
    if(interfInstance == NULL)
        interfInstance = new Interface(parser);

    return interfInstance;
}

void Interface::update()
{
    // Clean the interface matrix
    interface.release();
    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    // // put a circle in the interface where there are an obstacle
    //         cv::circle(interface, cv::Point(x_p, y_p), 2, obstacleColor, 2);

    //         // Distance from "trajectory" and possible-obstacle point
    //         int d_rect = ((y_p - y_robot) / m) - (x_p - x_robot);

    //         // If the obstacle is between max and min distance and it is close to the rect,
    //         // there is an obstacle (self-explained flag = true)
    //         if ( abs(d_rect) < r ) {

	// 	        // Distance from robot and possible-obstacle point
    //             double dist = (x_p - x_robot)^2 + (y_p - y_robot)^2;

    //             if ( (dist <= dist_max) && (dist >= dist_min) ) {

    //                 there_is_an_obstacle = true;

    //             }
    //         }
}


void Interface::put_path()
{
    cv::putText(interface, "To reach the target point,", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);
    cv::putText(interface, "follow the indicated path", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);

    // aggiungere modo di ricostruire il path e printarlo sull'interfaccia
}

void Interface::put_arrow(Position& robot, Position& target)
{
    cv::putText(interface, "If the arrow is red there is", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
    cv::putText(interface, "an obstacle on the trajectory", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);

     // Arrow from robot to target (the arrow is shown only if the distance of the robot from the target is under a certain threshold {1 mt})
    // if ( dist_rt >= 1000*scale ) {

    //     if ( true ){
    //         cv::arrowedLine(interface, cv::Point(x1, y1), cv::Point(x2, y2),
    //                         cv::Scalar(0,0,255), 5);
    //     } else {
    //         cv::arrowedLine(interface, cv::Point(x1, y1), cv::Point(x2, y2),
    //                         arrowColor, 5);
    //     }

    // }
}
