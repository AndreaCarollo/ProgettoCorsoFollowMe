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
    
    scale = grid_size / max_dist;                   // distance scale from real [mm] to grid
    robot = {grid_size-1, grid_size/2};             // position of the robot in the grid (index terms of row and column)

    // Set the grid
    grid = AStar_mtx (grid_size, std::vector<AStar_cell>(grid_size, {true, false, 1, nullptr, {0,0}}));
    grid[robot.row][robot.col].visited  = true;
    grid[robot.row][robot.col].cell     = robot;

    this->path_planning = flag;          // If it is true, use the path planning algorithm
    
    interface = Interface::getInstance(p);
}

void Control::update(pcl::PointXYZ* refPnt, PntCld::Ptr PointCloud, cv::Size cvFrameSize, Plane* plane)
{

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~ HERE SHOULD BE ADDED THE PLANE SEGMENTATION PART ~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // We could use the downsampled version of the cloud both for the
    // plane finding and obstacle identification. We must use a correct
    // value of resolution

    // update the astar grid
    // find the path (refPnt and robot position)
    // update the interface

    // update the astar grid
    // map the target point into the grid
    target.row = robot.row - (refPnt->z)*scale;
    target.col = robot.col - (refPnt->x)*scale;

    // GRAPHIC INTERFACE PRELIMINARY RENDER
    there_is_an_obstacle = false;

    interface->update(refPnt);

    obstacle_finding(PointCloud, plane);        // Search and draw the obstacles

    if (path_planning) {

        A_star();                               // Path planning
        interface->put_path(grid, target);      // Draw the path

    } else
        interface->put_arrow();    // Draw an arrow from the input to the output

    interface->put_references();          // Draw the robot and the target position

}


void Control::update(cv::Point* targetPoint2D, Stream* stream, Plane* plane)
{
    stream->PC_acq();

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~ HERE SHOULD BE ADDED THE PLANE SEGMENTATION PART ~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // We could use the downsampled version of the cloud both for the
    // plane finding and obstacle identification. We must use a correct
    // value of resolution

    stream->project_RGB2DEPTH(targetPoint2D);

    // update the astar grid
    // find the path (refPnt and robot position)
    // update the interface

    // update the astar grid
    // map the target point into the grid
    target.row = robot.row - (stream->refPnt.z)*scale;
    target.col = robot.col - (stream->refPnt.x)*scale;

    // GRAPHIC INTERFACE PRELIMINARY RENDER
    there_is_an_obstacle = false;

    interface->update(&(stream->refPnt));

    obstacle_finding(stream->cloud, plane);        // Search and draw the obstacles

    if (path_planning) {

        A_star();                               // Path planning
        interface->put_path(grid, target);      // Draw the path

    } else
        interface->put_arrow();                 // Draw an arrow from the input to the output

    interface->put_references();                // Draw the robot and the target position

}


void Control::obstacle_finding(PntCld::Ptr cloud, Plane* plane)
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
                if ( !( (p_row > target.row - 1 ) && (p_row < target.row + 1) && (p_col > target.col - 1) && (p_col < target.col + 1) ) )
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
