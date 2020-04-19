#include "control.h"


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~ Class functions ~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Control::Control(ConfigReader *p)
{
    p->getValue("LOOK_AHEAD_DIST", max_dist);                   // Maximum ahead distance
    p->getValue("OBST_MIN_THRESH", low_threshold);              // Minimum height to be considered an obstacle
    p->getValue("OBST_MAX_THRESH", up_threshold);               // Maximum height to be considered an obstacle
    p->getValue("OBST_TARGET_THRESH", target_threshold);        // Maximum height to be considered an obstacle
    p->getValue("DISTANCE_THRESHOLD", distance_threshold);      // Distance from the target at which the control stops
    p->getValue("OBSTACLE_LEAF", (int&) obstacle_resolution);   // Resolution with which the obstacles are searched
    p->getValue("GRID_SIZE", grid_size);                        // Size of the grid of the path planning
    p->getValue("PATH_PLANNING", path_planning);                // If path_planning is true, use the path planning algorithm 
    p->getValue("OBST_LOOK_DOWN", look_down);                   // How much of the total point cloud is used to search the obstacle
    
    scale = grid_size / max_dist;                               // distance scale from real [mm] to grid
    robot = {grid_size-1, grid_size/2};                         // position of the robot in the grid (index terms of row and column)

    offset_from_targer = target_threshold*scale;                // An obstacle, to be considered, must be at least at target_threshold [mm] from the target

    // Set the grid
    grid = AStar_mtx (grid_size, std::vector<AStar_cell>(grid_size, {true, false, 1, nullptr, {0,0}}));
    grid[robot.row][robot.col].visited  = true;
    grid[robot.row][robot.col].cell     = robot;

    refPnt = pcl::PointXYZ(0, 0, 0);                            // Reference point inizialization
    
    interface = Interface::getInstance(p);                      // Interface class initialization
    plane = Plane::getInstance(p);                              // Plane class initialization
}

void Control::update(cv::Point* targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize)
{

    // update the plane with the current point cloud
    // find the reference point in 3D and transform using the plane transformation mtr
    // update the astar grid
    // Search and draw the obstacles into the interface and grid too
    // find the path (if the path_planning is true)
    // update the interface

    plane->update(PointCloud);

    refPnt = PointCloud->points[(targetPoint2D->y-1)*cvFrameSize.width+targetPoint2D->x];
    refPnt = pcl::transformPoint(refPnt, plane->transf_mtx);

    target.row = robot.row - (refPnt.z)*scale;
    target.col = robot.col - (refPnt.x)*scale;

    distance_robot_target = std::sqrt( std::pow(refPnt.z,2) + std::pow(refPnt.x,2) );

    interface->clean();

    obstacle_finding(PointCloud);

    if ( distance_robot_target >= distance_threshold )
        if (path_planning)
            A_star();
            
    interface->update(this);

}


void Control::update(cv::Point* targetPoint2D, Stream* stream)
{

    // acquire the point cloud from the stream
    // project the input point (RGB RF) in the correct reference freme (IR RF)
    // update the plane with the current point cloud
    // find the reference point in 3D and transform using the plane transformation mtr
    // update the astar grid
    // Search and draw the obstacles into the interface and grid too
    // find the path (if the path_planning is true)
    // update the interface

    stream->PC_acq();

    stream->project_RGB2DEPTH(targetPoint2D);

    plane->update(stream->cloud);

    refPnt = stream->cloud->points[(targetPoint2D->y-1)*stream->w_IR+targetPoint2D->x];
    refPnt = pcl::transformPoint(refPnt, plane->transf_mtx);

    target.row = robot.row - (stream->refPnt.z)*scale;
    target.col = robot.col - (stream->refPnt.x)*scale;

    distance_robot_target = std::sqrt( std::pow(stream->refPnt.z,2) + std::pow(stream->refPnt.x,2) );

    interface->clean();

    obstacle_finding(stream->cloud);

    if ( distance_robot_target >= distance_threshold )
        if (path_planning)
            A_star();

    interface->update(this);

}


void Control::obstacle_finding(PntCld::Ptr cloud)
{

    for (size_t i = look_down*cloud->size(); i<cloud->size(); i += obstacle_resolution)
    {
        /*
        // The point of the cloud must be transformed by the plene transformation mtx (~7.5 ms)
        tmpPnt = cloud->points[i];
        tmpPnt = pcl::transformPoint(tmpPnt, plane->transf_mtx);
        */
        
        // We can decide to compute the y of the plane connected to the x and z of the point (~6.5 ms)
        float y_pnt = - (plane->coefficients->values[0] * cloud->points[i].x + 
                         plane->coefficients->values[2] * cloud->points[i].z + 
                         plane->coefficients->values[3] ) /
                         plane->coefficients->values[1];
        float d = cloud->points[i].y - y_pnt;
        
        /*
        // Considering that the plane is the one in which lives the robot, all the points
        // that do not be in this plane are obstacles, in principle (~5.5 ms)
        float d = abs(-plane->coefficients->values[0] * cloud->points[i].x +
                       plane->coefficients->values[1] * cloud->points[i].y -
                       plane->coefficients->values[2] * cloud->points[i].z +
                       plane->coefficients->values[3]) /
                    std::sqrt(std::pow(plane->coefficients->values[0],2) + 
                              std::pow(plane->coefficients->values[1],2) + 
                              std::pow(plane->coefficients->values[2],2));
        */
        
                
        // if ( tmpPnt.y > low_threshold && tmpPnt.y < up_threshold )      // If we use the transformed points
        if ( d > low_threshold && d < up_threshold )                    // If we use the point/plane distance or the difference in y
        {
            /*
            // If we use the transformed points
            int p_row = robot.row - (tmpPnt.z)*scale;
            int p_col = robot.col - (tmpPnt.x)*scale;
            */
            
            // If we use the point/plane distance or the difference in y
            int p_row = robot.row - (cloud->points[i].z)*scale;
            int p_col = robot.col - (cloud->points[i].x)*scale;
            

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

    current = &grid[robot.row][robot.col];  // the starting point
    
    frontier.push(current);

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
