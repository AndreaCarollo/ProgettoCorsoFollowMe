#include "interface.h"


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~ Class functions ~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Interface* Interface::interfInstance = NULL;

Interface::Interface(ConfigReader *p)
{
    p->getValue("LOOK_AHEAD_DIST", max_dist);               // Maximum ahead distance
    p->getValue("FONT_SCALE", font_scale);                  // Font scale
    p->getValue("BG_COLOR", backgroundColor);               // Color of the backgroud of the interface
    p->getValue("ARROW_COLOR", arrowColor);                 // Color of the arrow (if the path planning is not used)
    p->getValue("OBSTACLE_COLOR", obstacleColor);           // Color of the obstacle zone
    p->getValue("ROBOT_COLOR", robotColor);                 // Color of the robot point marker
    p->getValue("TARGET_COLOR", targetColor);               // Color of the target point marker
    p->getValue("GRID_SIZE", gs);                           // Size of the grid of the path planning
    p->getValue("RADIUS", r);                               // Circle radius for robot and target points marker
    p->getValue("OFFSET", offset);                          // Offset distance (some differene graphic uses)
    p->getValue("INTERFACE_SIZE", interface_size);          // Size of the interface

    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    // Position of the robot in the graphic interface
    robot = cv::Point(interface.cols/2, interface.rows - offset);

    // Definition of the scale that maps the grid distance into interface distance
    scale = interface_size.width/gs;

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

void Interface::update(Control *ctrl)
{
    target = cv::Point(ctrl->target.col*scale,
                        ctrl->target.row*scale);

    if (ctrl->path_planning)
        put_path(ctrl->grid, ctrl->target);
    else
        put_arrow();

    put_references();

}

void Interface::put_path(std::vector< std::vector<struct AStar_cell> >& grid, Position& target)
{
    cv::putText(interface, "To reach the target point,", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);
    cv::putText(interface, "follow the indicated path", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);

    // Starting from the target point follow the path until 
    // the starting point where the come_from pointer is NULL
    tmp_cell = &grid[target.row][target.col];

    do {
         int x_rect = tmp_cell->cell.col * scale;
         int y_rect = tmp_cell->cell.row * scale;

         cv::rectangle(interface, 
                       cv::Point(x_rect-scale/2,y_rect-scale/2), cv::Point(x_rect+scale/2-1,y_rect+scale/2-1), 
                       arrowColor, -1);

        tmp_cell = tmp_cell->came_from;

    } while( tmp_cell != NULL);
}

void Interface::put_arrow()
{
    cv::putText(interface, "Follow the arrow to reach ", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);
    cv::putText(interface, "the target", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);

    // Angulat coefficient of the rect that goes from robot to target
    m = (float)((target.y - robot.y)/(target.x - robot.x));

    // Arrow starting and final point coordinates
    arrow_tail = cv::Point (-(r + offset) / m + robot.x, robot.y - r - offset);
    arrow_head = cv::Point ((r + offset) / m + target.x, target.y + r + offset);

    // Distance from the robot of the two extremes of the arrow
    dist_max = std::sqrt( std::pow(arrow_head.x - robot.x,2) + std::pow(arrow_head.y - robot.y,2) );
    dist_min = std::sqrt( std::pow(arrow_tail.x - robot.x,2) + std::pow(arrow_tail.y - robot.y,2) );

    // Arrow from robot to target
    if ( dist_min < dist_max )
        cv::arrowedLine(interface, arrow_tail, arrow_head, arrowColor, 5);
}

void Interface::put_obstacle(int p_col, int p_row)
{
    cv::rectangle(interface, 
                  cv::Point(p_col*scale-scale/2, p_row*scale-scale/2), 
                  cv::Point(p_col*scale+scale/2-1, p_row*scale+scale/2-1), 
                  obstacleColor, -1);
}

void Interface::put_references()
{
    // Circle for the robot position + Legend
    cv::circle(interface, robot, r, robotColor, cv::FILLED, cv::LINE_8);
    cv::circle(interface, cv::Point(interface_size.width - offset,offset), r, robotColor,  cv::FILLED, cv::LINE_8);
    cv::putText(interface, "Robot", cv::Point(interface_size.width - offset - r - 140, offset + r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, robotColor, 2);

    // Green cyrcle for the target postion + Legend
    cv::circle(interface, target, r, targetColor, cv::FILLED, cv::LINE_8);
    cv::circle(interface, cv::Point(interface_size.width - offset,2 * offset + 2 * r), r, targetColor, cv::FILLED, cv::LINE_8);
    cv::putText(interface, "Target", cv::Point(interface_size.width - offset - r - 140, 2 * offset + 3 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, targetColor, 2);
}

void Interface::clean()
{
    // Clean the interface matrix
    interface.release();
    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);
}