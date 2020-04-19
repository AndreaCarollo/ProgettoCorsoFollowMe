#include "control.h"


// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------
Control::Control(ConfigReader *p)
{
    p->getValue("LOOK_AHEAD_DIST", max_dist);               // Maximum ahead distance
    p->getValue("OBST_MIN_THRESH", low_threshold);          // Minimum height to be considered an obstacle
    p->getValue("OBST_MAX_THRESH", up_threshold);           // Maximum height to be considered an obstacle
    p->getValue("OBST_TARGET_THRESH", target_threshold);           // Maximum height to be considered an obstacle
    p->getValue("DISTANCE_THRESHOLD", distance_threshold);  // Distance from the target at which the control stops
    p->getValue("OBSTACLE_LEAF", (int&) obstacle_resolution);
    p->getValue("GRID_SIZE", grid_size);
    p->getValue("PATH_PLANNING", path_planning);            // If path_planning is true, use the path planning algorithm 
    
    scale = grid_size / max_dist;                   // distance scale from real [mm] to grid
    robot = {grid_size-1, grid_size/2};             // position of the robot in the grid (index terms of row and column)

    offset_from_targer = target_threshold*scale;    // An obstacle, to be considered, must be at least at target_threshold [mm] from the target

    // Set the grid
    grid = AStar_mtx (grid_size, std::vector<AStar_cell>(grid_size, {true, false, 1, nullptr, {0,0}}));
    grid[robot.row][robot.col].visited  = true;
    grid[robot.row][robot.col].cell     = robot;

    refPnt = pcl::PointXYZ(0, 0, 0);
    
    interface = Interface::getInstance(p);
    plane = Plane::getInstance(p);
}

void Control::update(cv::Point* targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize)
{

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~ HERE SHOULD BE ADDED THE PLANE SEGMENTATION PART ~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // We could use the downsampled version of the cloud both for the
    // plane finding and obstacle identification. We must use a correct
    // value of resolution

    plane->update(PointCloud);

    refPnt = PointCloud->points[(targetPoint2D->y-1)*cvFrameSize.width+targetPoint2D->x];
    refPnt = pcl::transformPoint(refPnt, plane->transf_mtx);

    // update the astar grid
    // find the path (refPnt and robot position)
    // update the interface

    // update the astar grid
    // map the target point into the grid
    target.row = robot.row - (refPnt.z)*scale;
    target.col = robot.col - (refPnt.x)*scale;

    distance_robot_target = std::sqrt( std::pow(refPnt.z,2) + std::pow(refPnt.x,2) );

    // GRAPHIC INTERFACE PRELIMINARY RENDER

    interface->update(&refPnt);

    obstacle_finding(PointCloud);        // Search and draw the obstacles

    if ( distance_robot_target >= distance_threshold ) {
        if (path_planning) {

            A_star();                               // Path planning
            interface->put_path(grid, target);      // Draw the path

        } else
            interface->put_arrow();    // Draw an arrow from the input to the output
    }

    interface->put_references();          // Draw the robot and the target position

}


void Control::update(cv::Point* targetPoint2D, Stream* stream)
{
    stream->PC_acq();

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~ HERE SHOULD BE ADDED THE PLANE SEGMENTATION PART ~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // We could use the downsampled version of the cloud both for the
    // plane finding and obstacle identification. We must use a correct
    // value of resolution

    stream->project_RGB2DEPTH(targetPoint2D);

    plane->update(stream->cloud);

    refPnt = stream->cloud->points[(targetPoint2D->y-1)*stream->w_IR+targetPoint2D->x];
    refPnt = pcl::transformPoint(refPnt, plane->transf_mtx);

    // update the astar grid
    // find the path (refPnt and robot position)
    // update the interface

    // update the astar grid
    // map the target point into the grid
    target.row = robot.row - (stream->refPnt.z)*scale;
    target.col = robot.col - (stream->refPnt.x)*scale;

    distance_robot_target = std::sqrt( std::pow(stream->refPnt.z,2) + std::pow(stream->refPnt.x,2) );

    // GRAPHIC INTERFACE PRELIMINARY RENDER

    interface->update(&(stream->refPnt));

    obstacle_finding(stream->cloud);        // Search and draw the obstacles

    if ( distance_robot_target >= distance_threshold ) {
        if (path_planning) {

            A_star();                               // Path planning
            interface->put_path(grid, target);      // Draw the path

        } else
            interface->put_arrow();                 // Draw an arrow from the input to the output
    }

    interface->put_references();                // Draw the robot and the target position

}


void Control::obstacle_finding(PntCld::Ptr cloud)
{

    for (size_t i = 0; i<cloud->size()/obstacle_resolution; i++)
    {

        // Considering that the plane is the one in which lives the robot, all the points
        // that do not be in this plane are obstacles, in principle
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
                
                // Obstacle not attached to the target
                if ( !( (p_row > target.row - offset_from_targer) && (p_row < target.row + offset_from_targer) && 
                        (p_col > target.col - offset_from_targer) && (p_col < target.col + offset_from_targer) ) )
                {
                    grid[p_row][p_col].cell.row = p_row;
                    grid[p_row][p_row].cell.col = p_col;
                    grid[p_row][p_col].free = false;

                    // The for used for find the obstacle is also used to draw the obstacle in the interface
                    interface->put_obstacle(p_col, p_row);

                }
            }
        }
    }

}

void Control::A_star()
{

    start = &grid[robot.row][robot.col];

    // Set the target point in the grid
    // grid[target.row][target.col].cell = target;
    
    frontier.push(start);

    do {
        current = frontier.front();
        frontier.pop();
        neighbors(current);
    } while (frontier.size() != 0);

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
