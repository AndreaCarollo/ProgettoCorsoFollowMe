#include "./control.h"


// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------
Control::Control(ConfigReader *p, bool flag)
{
    
    // Interface option
    interface_size = cv::Size(640,640);             // human graphic interface size
    r = 15;                                         // r      -> Cyrcle radious for robot and target
    offset = 18;                                    // offset -> Offset distance (some differene graphic uses)

    font_scale = 0.9f;                              // Font scale for text

    p->getValue("BG_COLOR", backgroundColor);
    p->getValue("ARROW_COLOR", arrowColor);
    p->getValue("OBSTACLE_COLOR", obstacleColor);
    p->getValue("ROBOT_COLOR", robotColor);
    p->getValue("TARGET_COLOR", targetColor);

    p->getValue("LOOK_AHEAD_DIST", max_dist);               // Maximum ahead distance
    p->getValue("OBST_MIN_THRESH", low_threshold);          // Minimum height to be considered an obstacle
    p->getValue("OBST_MAX_THRESH", up_threshold);           // Maximum height to be considered an obstacle

    p->getValue("OBSTACLE_GRAIN", (int&) obstacle_resolution);
    
    AStarScale = 10;

    scale = interface_size.height / max_dist;         // distance scale from real [mm] to graphic interface


    // cv::Mat for the graphic interface
    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    this->flag = flag;          // If it is true, use the path planning algorithm

    // Position of the robot in the graphic interface
    x_robot = interface_size.width/2;
    y_robot = interface_size.height - offset;

    max_row = interface.rows / AStarScale;
    max_col = interface.cols / AStarScale;
    grid = std::vector<std::vector<AStar_cel>>(max_col, std::vector<AStar_cel>(max_row, {true, false, 0, nullptr, 0, 0}));

}

void Control::update(cv::Point* targetPoint2D, Stream* stream, Plane* plane)
{

    // Clean the interface matrix
    interface.release();
    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    // Projection from RGB to depth
    stream->project_RGB2DEPTH(targetPoint2D);

    tmp = (stream->refPnt.x)*scale;
    x_target = x_robot - tmp;
    tmp = (stream->refPnt.z)*scale;
    y_target = y_robot - tmp;

    // Angulat coefficient of the rect that goes from robot to target
    m = (float)((y_target-y_robot)/(x_target-x_robot));

    // Arrow starting and final point coordinates
    x1_arrow = - 2 * offset / m + x_robot;
    y1_arrow = y_robot - 2 * offset;
    x2_arrow = 2 * offset / m + x_target;
    y2_arrow = y_target + 2 * offset;

    // Maximum and minimum distance between robot and target
    dist_rt  = (x_target - x_robot)^2 + (y_target - y_robot)^2;
    dist_max = (x2_arrow - x_robot)^2 + (y2_arrow - y_robot)^2;
    dist_min = (x1_arrow - x_robot)^2 + (y1_arrow - y_robot)^2;


    // GRAPHIC INTERFACE PRELIMINARY RENDER
    there_is_an_obstacle = false;
    obstacle_finding(stream->cloud, plane);

    if (!flag){

        cv::putText(interface, "If the arrow is red there is", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
        cv::putText(interface, "an obstacle on the trajectory", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);

        put_arrow();

    } else {

        cv::putText(interface, "To reach the target point,", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
        cv::putText(interface, "follow the indicated path", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);

        grid = std::vector<std::vector<AStar_cel>>(max_col, std::vector<AStar_cel>(max_row, {true, false, 0, nullptr, 0, 0}));

        A_star();
    }

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
    
    // Obstacles
    cv::circle(interface, cv::Point(interface_size.width - offset,3 * offset + 4 * r), 4, obstacleColor,  cv::FILLED, cv::LINE_8);
    cv::putText(interface, "Obstacles", cv::Point(interface_size.width - offset - r - 140, 3 * offset + 5 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, obstacleColor, 2);

}

void Control::update(pcl::PointXYZ* refPnt, PntCld::Ptr PointCloud, cv::Size cvFrameSize, Plane* plane)
{
    
    // Clean the interface matrix
    interface.release();
    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    tmp = (refPnt->x)*scale;
    x_target = x_robot - tmp;
    tmp = (refPnt->z)*scale;
    y_target = y_robot - tmp;

    // Angulat coefficient of the rect that goes from robot to target
    m = (float)((y_target-y_robot)/(x_target-x_robot));

    // Arrow starting and final point coordinates
    x1_arrow = - 2 * offset / m + x_robot;
    y1_arrow = y_robot - 2 * offset;
    x2_arrow = 2 * offset / m + x_target;
    y2_arrow = y_target + 2 * offset;

    // Maximum and minimum distance between robot and target
    dist_rt  = (x_target - x_robot)^2 + (y_target - y_robot)^2;
    dist_max = (x2_arrow - x_robot)^2 + (y2_arrow - y_robot)^2;
    dist_min = (x1_arrow - x_robot)^2 + (y1_arrow - y_robot)^2;


    // GRAPHIC INTERFACE PRELIMINARY RENDER
    there_is_an_obstacle = false;
    obstacle_finding(PointCloud, plane);
    
    
    if (flag){

        cv::putText(interface, "To reach the target point,", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);
        cv::putText(interface, "follow the indicated path", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, arrowColor, 2);
        
        grid = std::vector<std::vector<AStar_cel>>(max_col, std::vector<AStar_cel>(max_row, {true, false, 0, NULL, 0, 0}));

        A_star();

    } else {
        
        cv::putText(interface, "If the arrow is red there is", cv::Point(offset, offset + r), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
        cv::putText(interface, "an obstacle on the trajectory", cv::Point(offset, offset + r + 35 * font_scale), 
                    cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);

        put_arrow();

    }

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
    
    // Obstacles
    cv::circle(interface, cv::Point(interface_size.width - offset,3 * offset + 4 * r), 4, obstacleColor,  cv::FILLED, cv::LINE_8);
    cv::putText(interface, "Obstacles", cv::Point(interface_size.width - offset - r - 140, 3 * offset + 5 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, obstacleColor, 2);

}

void Control::obstacle_finding(PntCld::Ptr cloud, Plane* plane)
{
    
    // Obstacle finding
    for (int i = 0; i<cloud->size()/obstacle_resolution; i++){

        tmp_pnt = pcl::transformPoint(cloud->points[obstacle_resolution*i], plane->transf_mtx);

        if ( tmp_pnt.y > low_threshold && 
             tmp_pnt.y < up_threshold ){

            tmp = (tmp_pnt.x)*scale;
            int x_p = x_robot - tmp;
            tmp = (tmp_pnt.z)*scale;
            int y_p = y_robot - tmp;

            if (x_p >= 0 && y_p >= 0 && x_p < interface.cols && y_p < interface.rows){    // Obstacle inside the interface
                if ( (abs(x_p-x_target) > AStarScale) && (abs(y_p-y_target) > AStarScale) ){    // Obstacle at a certain distance from the robot
                    
                    grid[x_p/AStarScale][y_p/AStarScale].free = false;

                }
            }

            // put a circle in the interface where there are an obstacle
            cv::circle(interface, cv::Point(x_p, y_p), 2, obstacleColor, 2);

            // Distance from "trajectory" and possible-obstacle point
            int d_rect = ((y_p - y_robot) / m) - (x_p - x_robot);

            // If the obstacle is between max and min distance and it is close to the rect,
            // there is an obstacle (self-explained flag = true)
            if ( abs(d_rect) < r ) {

		        // Distance from robot and possible-obstacle point
                double dist = (x_p - x_robot)^2 + (y_p - y_robot)^2;

                if ( (dist <= dist_max) && (dist >= dist_min) ) {

                    there_is_an_obstacle = true;

                }
            }

        }
    }

}

void Control::put_arrow()
{

    // Arrow from robot to target (the arrow is shown only if the distance of the robot from the target is under a certain threshold {1 mt})
    if ( dist_rt >= 1000*scale ) {

        if (there_is_an_obstacle){
            cv::arrowedLine(interface, cv::Point(x1_arrow, y1_arrow), cv::Point(x2_arrow, y2_arrow),
                            cv::Scalar(0,0,255), 5);
        } else {
            cv::arrowedLine(interface, cv::Point(x1_arrow, y1_arrow), cv::Point(x2_arrow, y2_arrow),
                            arrowColor, 5);
        }

    }

}

void Control::A_star()
{

    // Set the starting point in the grid - START
    grid[x_robot/AStarScale][y_robot/AStarScale].visited    = true;
    grid[x_robot/AStarScale][y_robot/AStarScale].col        = x_robot/AStarScale;
    grid[x_robot/AStarScale][y_robot/AStarScale].row        = y_robot/AStarScale;

    start = &grid[x_robot/AStarScale][y_robot/AStarScale];

    
    // Set the target point in the grid - STOP
    grid[x_target/AStarScale][y_target/AStarScale].col      = x_target/AStarScale;
    grid[x_target/AStarScale][y_target/AStarScale].row      = y_target/AStarScale;

    
    frontier.push(start);

    do {
        
        current = frontier.front();
        frontier.pop();

        neighbors(current);

    } while (frontier.size() != 0);

    
    tmp_cel = &grid[x_target/AStarScale][y_target/AStarScale];  // Starting from the target point ...
    // ... follow the path until the starting point where the come_from pointer is NULL

    do {

        x_rect = tmp_cel->col*AStarScale;
        y_rect = tmp_cel->row*AStarScale;

        cv::rectangle(interface, 
                      cv::Point(x_rect,y_rect), cv::Point(x_rect+AStarScale,y_rect+AStarScale), 
                      arrowColor, -1);

        tmp_cel = tmp_cel->came_from;

    } while( tmp_cel != NULL);

}

void Control::neighbors(AStar_cel* current_cel){

    int x = current_cel->col, y = current_cel->row;

    for (int xx = -1; xx <= 1; xx++)
    {
        for (int yy = -1; yy <= 1; yy++)
        {
            if (abs(xx) != abs(yy))
            {
                if ((x+xx >= 0) && (y+yy >= 0) && (x+xx < max_col) && (y+yy < max_row))
                {
                    if ((grid[x+xx][y+yy].free) && !(grid[x+xx][y+yy].visited)){

                        grid[x+xx][y+yy].col        = x+xx;
                        grid[x+xx][y+yy].row        = y+yy;
                        grid[x+xx][y+yy].came_from  = current_cel;
                        grid[x+xx][y+yy].visited    = true;

                        frontier.push(&grid[x+xx][y+yy]);
                    }
                }
            }
        }
    }
}