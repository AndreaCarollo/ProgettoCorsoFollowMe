#include "control.h"


// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------
Control::Control(ConfigReader *p, bool flag)
{
    p->getValue("LOOK_AHEAD_DIST", max_dist);               // Maximum ahead distance
    p->getValue("OBST_MIN_THRESH", low_threshold);          // Minimum height to be considered an obstacle
    p->getValue("OBST_MAX_THRESH", up_threshold);           // Maximum height to be considered an obstacle
    p->getValue("OBSTACLE_LEAF", (int&) obstacle_resolution);
    p->getValue("GRID_SIZE", grid_size);
    
    scale = grid_size / max_dist;         // distance scale from real [mm] to grid
    robot = {grid_size-1, grid_size/2};           // position of the robot in the grid (index terms of row and column)

    // Set the grid
    grid = AStar_mtx (grid_size, std::vector<AStar_cell>(grid_size, {true, false, 1, nullptr, {0,0}}));
    grid[robot.row][robot.col].visited  = true;
    grid[robot.row][robot.col].cell     = robot;

    this->path_planning = flag;          // If it is true, use the path planning algorithm
    
    interface = Interface::getInstance(p);

}

// void Control::update(cv::Point* targetPoint2D, Stream* stream, Plane* plane)
// {

//     // Clean the interface matrix
//     interface.release();
//     interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

//     // Projection from RGB to depth
//     stream->project_RGB2DEPTH(targetPoint2D);

//     tmp = (stream->refPnt.x)*scale;
//     x_target = x_robot - tmp;
//     tmp = (stream->refPnt.z)*scale;
//     y_target = y_robot - tmp;

//     // Angulat coefficient of the rect that goes from robot to target
//     m = (float)((y_target-y_robot)/(x_target-x_robot));

//     // Arrow starting and final point coordinates
//     x1_arrow = - 2 * offset / m + x_robot;
//     y1_arrow = y_robot - 2 * offset;
//     x2_arrow = 2 * offset / m + x_target;
//     y2_arrow = y_target + 2 * offset;

//     // Maximum and minimum distance between robot and target
//     dist_rt  = (x_target - x_robot)^2 + (y_target - y_robot)^2;
//     dist_max = (x2_arrow - x_robot)^2 + (y2_arrow - y_robot)^2;
//     dist_min = (x1_arrow - x_robot)^2 + (y1_arrow - y_robot)^2;


//     // GRAPHIC INTERFACE PRELIMINARY RENDER
//     there_is_an_obstacle = false;
//     obstacle_finding(stream->cloud, plane);

//     if (!path_planning){

//         cv::putText(interface, "If the arrow is red there is", cv::Point(offset, offset + r), 
//                     cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
//         cv::putText(interface, "an obstacle on the trajectory", cv::Point(offset, offset + r + 35 * font_scale), 
//                     cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);

//         put_arrow();

//     } else {

//         cv::putText(interface, "To reach the target point,", cv::Point(offset, offset + r), 
//                     cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
//         cv::putText(interface, "follow the indicated path", cv::Point(offset, offset + r + 35 * font_scale), 
//                     cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);


//         A_star();
//     }

//     // Circle for the robot position
//     cv::circle(interface, cv::Point(x_robot,y_robot), r, robotColor, cv::FILLED, cv::LINE_8);
//     cv::circle(interface, cv::Point(interface_size.width - offset,offset), r, robotColor,  cv::FILLED, cv::LINE_8);
//     cv::putText(interface, "Robot", cv::Point(interface_size.width - offset - r - 140, offset + r), 
//                 cv::FONT_HERSHEY_SIMPLEX, font_scale, robotColor, 2);

//     // Green cyrcle for the target postion
//     cv::circle(interface, cv::Point(x_target,y_target), r, targetColor, cv::FILLED, cv::LINE_8);
//     cv::circle(interface, cv::Point(interface_size.width - offset,2 * offset + 2 * r), r, targetColor, cv::FILLED, cv::LINE_8);
//     cv::putText(interface, "Target", cv::Point(interface_size.width - offset - r - 140, 2 * offset + 3 * r), 
//                 cv::FONT_HERSHEY_SIMPLEX, font_scale, targetColor, 2);
    
//     // Obstacles
//     cv::circle(interface, cv::Point(interface_size.width - offset,3 * offset + 4 * r), 4, obstacleColor,  cv::FILLED, cv::LINE_8);
//     cv::putText(interface, "Obstacles", cv::Point(interface_size.width - offset - r - 140, 3 * offset + 5 * r), 
//                 cv::FONT_HERSHEY_SIMPLEX, font_scale, obstacleColor, 2);

// }

void Control::update(pcl::PointXYZ* refPnt, PntCld::Ptr PointCloud, cv::Size cvFrameSize, Plane* plane)
{
    // update the astar grid
    // find the path (refPnt and robot position)
    // update the interface

    // update the astar grid
    // map the target point into the grid
    target.row = robot.row - (refPnt->z)*scale;
    target.col = robot.col - (refPnt->x)*scale;
    

    // Maximum and minimum distance between robot and target
    // dist_rt  = (x_target - x_robot)^2 + (y_target - y_robot)^2;
    // dist_max = (x2_arrow - x_robot)^2 + (y2_arrow - y_robot)^2;
    // dist_min = (x1_arrow - x_robot)^2 + (y1_arrow - y_robot)^2;


    // GRAPHIC INTERFACE PRELIMINARY RENDER
    there_is_an_obstacle = false;

    if (path_planning)
        path_finding(PointCloud, plane);
    else
        interface->put_arrow(robot, target);

    interface->update();

}

void Control::path_finding(PntCld::Ptr cloud, Plane* plane)
{
    for (size_t i = 0; i<cloud->size()/obstacle_resolution; i++)
    {
        // tmp_pnt = pcl::transformPoint(cloud->points[i], plane->transf_mtx);
        tmp_pnt = cloud->points[obstacle_resolution*i];
        
        int d = abs(plane->coefficients->values[0] * tmp_pnt.x +
                    plane->coefficients->values[1] * tmp_pnt.y +
                    plane->coefficients->values[2] * tmp_pnt.z +
                    plane->coefficients->values[3]) /
                    std::sqrt(std::pow(plane->coefficients->values[0],2) + 
                                std::pow(plane->coefficients->values[1],2) + 
                                std::pow(plane->coefficients->values[2],2));
        
        if ( d > low_threshold && d < up_threshold )
        {
            int p_row = robot.row - (tmp_pnt.z)*scale;
            int p_col = robot.col - (tmp_pnt.x)*scale;

            if (p_row >= 0 && p_col >= 0 && p_row < grid_size && p_col < grid_size)     // Obstacle inside the grid
            {
                if (!( (p_row > target.row -1 ) && (p_row < target.row + 1) && (p_col > target.col - 1) && (p_col < target.col + 1)))
                // if ( (abs(p_row-target.row) <= 1) && (abs(p_col-target.col) <= 1) )
                {
                    grid[p_row][p_col].cell.row = p_row;
                    grid[p_row][p_row].cell.col = p_col;
                    grid[p_row][p_col].free = false;
                }
            }
        }
    }
    A_star();
}

void Control::A_star()
{
    start = &grid[robot.row][robot.col];

    // Set the target point in the grid - STOP
    grid[target.row][target.col].cell = target;
    
    frontier.push(start);

    do {
        current = frontier.front();
        frontier.pop();
        neighbors(current);
    } while (frontier.size() != 0);
    
    tmp_cel = &grid[target.row][target.col];  // Starting from the target point ...
    // ... follow the path until the starting point where the come_from pointer is NULL

    // do {

    //     x_rect = tmp_cel->col*AStarScale;
    //     y_rect = tmp_cel->row*AStarScale;

    //     cv::rectangle(interface, 
    //                   cv::Point(x_rect,y_rect), cv::Point(x_rect+AStarScale,y_rect+AStarScale), 
    //                   arrowColor, -1);

    //     tmp_cel = tmp_cel->came_from;

    // } while( tmp_cel != NULL);

}

void Control::neighbors(AStar_cell* current)
{
    int r = current->cell.row, c = current->cell.col;

    for (int drow = -1; drow <= 1; drow++)
        for (int dcol = -1; dcol <= 1; dcol++)
            if (abs(drow) != abs(dcol))
                if ((r+drow >= 0) && (c+dcol >= 0) && (r+drow < grid_size) && (c+dcol < grid_size))
                    if (!(grid[r+drow][c+dcol].visited) && (grid[r+drow][c+dcol].free))
                    {
                        grid[r+drow][c+dcol].cell.col   = c+dcol;
                        grid[r+drow][c+dcol].cell.row   = r+drow;
                        grid[r+drow][c+dcol].came_from  = current;
                        grid[r+drow][c+dcol].visited    = true;
                        frontier.push(&grid[r+drow][c+dcol]);   
                    }
}