#include "interface.h"


// --------------------------------------------
// ---------------Class methods----------------
// --------------------------------------------

Interface* Interface::interfInstance = NULL;

Interface::Interface(ConfigReader *p)
{
    p->getValue("LOOK_AHEAD_DIST", max_dist);               // Maximum ahead distance
    p->getValue("FONT_SCALE", font_scale);
    p->getValue("BG_COLOR", backgroundColor);
    p->getValue("ARROW_COLOR", arrowColor);
    p->getValue("OBSTACLE_COLOR", obstacleColor);
    p->getValue("ROBOT_COLOR", robotColor);
    p->getValue("TARGET_COLOR", targetColor);
    p->getValue("GRID_SIZE", gs);
    p->getValue("RADIOUS", r);                      // Circle radius for robot and target

    offset = 18;                                    // offset -> Offset distance (some differene graphic uses)

    int sz;
    p->getValue("INTERFACE_SIZE", sz);
    interface_size = cv::Size(sz,sz);           // human graphic interface size

    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    x_robot = interface.cols/2;           // Position of the robot in the graphic interface
    y_robot = interface.rows - offset;

    scale = sz/gs;
    transf = sz/max_dist;

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

void Interface::update(pcl::PointXYZ* refPnt)
{
    // Clean the interface matrix
    interface.release();
    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    x_target = x_robot - (refPnt->x)*transf;
    y_target = y_robot - (refPnt->z)*transf;

}


// void Interface::put_path(AStar_mtx grid, Position& target) 
void Interface::put_path(std::vector< std::vector<struct AStar_cell> > grid, Position& target)
{
    cv::putText(interface, "To reach the target point,", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);
    cv::putText(interface, "follow the indicated path", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);



    tmp_cell = &(grid[target.row][target.col]);       // Starting from the target point ...
    // ... follow the path until the starting point where the come_from pointer is NULL

    do {

         int x_rect = tmp_cell->cell.col * scale;
         int y_rect = tmp_cell->cell.row * scale;

         cv::rectangle(interface, 
                       cv::Point(x_rect,y_rect), cv::Point(x_rect+scale,y_rect+scale), 
                       arrowColor, -1);

        tmp_cell = tmp_cell->came_from;

    } while( tmp_cell != NULL);

}

void Interface::put_arrow()
{
    /*
    cv::putText(interface, "If the arrow is red there is", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
    cv::putText(interface, "an obstacle on the trajectory", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
    */

    cv::putText(interface, "Follow the arrow to reach ", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);
    cv::putText(interface, "the target", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);

    // Angulat coefficient of the rect that goes from robot to target
    m = (float)((y_target-y_robot)/(x_target-x_robot));

    // Arrow starting and final point coordinates
    x1_arrow = - (r + offset) / m + x_robot;
    y1_arrow = y_robot - r - offset;
    x2_arrow = (r + offset) / m + x_target;
    y2_arrow = y_target + r + offset;

    // Maximum and minimum distance between robot and target
    dist_max = std::sqrt( std::pow(x2_arrow - x_robot,2) + std::pow(y2_arrow - y_robot,2) );
    dist_min = std::sqrt( std::pow(x1_arrow - x_robot,2) + std::pow(y1_arrow - y_robot,2) );

    // Arrow from robot to target (the arrow is shown only if the distance of the robot from the target is under a certain threshold {1 mt})
    if ( dist_min < dist_max ) {

        cv::arrowedLine(interface, cv::Point(x1_arrow, y1_arrow), cv::Point(x2_arrow, y2_arrow),
                        arrowColor, 5);

        /*
        if ( true ){
            cv::arrowedLine(interface, cv::Point(x1_arrow, y1_arrow), cv::Point(x2_arrow, y2_arrow),
                            cv::Scalar(0,0,255), 5);
        } else {
            cv::arrowedLine(interface, cv::Point(x1_arrow, y1_arrow), cv::Point(x2_arrow, y2_arrow),
                            arrowColor, 5);
        }
        */

    }
}

void Interface::put_obstacle(int p_col, int p_row)
{

    cv::rectangle(interface, 
                  cv::Point(p_col*scale, p_row*scale), 
                  cv::Point(p_col*scale+scale, p_row*scale+scale), 
                  obstacleColor, -1);

}

void Interface::put_references()
{

    // Circle for the robot position
    cv::circle(interface, cv::Point(x_robot,y_robot), r, robotColor, cv::FILLED, cv::LINE_8);
    cv::circle(interface, cv::Point(interface_size.width - offset,offset), r, robotColor,  cv::FILLED, cv::LINE_8);
    cv::putText(interface, "Robot", cv::Point(interface_size.width - offset - r - 140, offset + r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, robotColor, 2);

    // Green cyrcle for the target postion
    cv::circle(interface, cv::Point(x_target,y_target), r, targetColor, cv::FILLED, cv::LINE_8);
    cv::circle(interface, cv::Point(interface_size.width - offset,2 * offset + 2 * r), r, targetColor, cv::FILLED, cv::LINE_8);
    cv::putText(interface, "Target", cv::Point(interface_size.width - offset - r - 140, 2 * offset + 3 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, targetColor, 2);


}
