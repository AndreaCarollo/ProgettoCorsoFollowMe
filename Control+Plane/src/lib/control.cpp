#include "./control.h"


// --------------------------------------------
// --------------Class functions---------------
// --------------------------------------------
Control::Control(ConfigReader *p, bool flag)
{
    
    // Interface option
    interface_size = cv::Size(640,640);             // human graphic interface size
    r = 10;                                         // r      -> Cyrcle radious for robot and target
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

    obstacle_resolution;                                    // How many obstacle points in a row
    p->getValue("OBSTACLE_GRAIN", (int&) obstacle_resolution);

    scale = interface_size.height / max_dist;         // distance scale from real [mm] to graphic interface


    // cv::Mat for the graphic interface
    cv::Mat interface(interface_size, CV_8UC3, backgroundColor);

    this->flag = flag;          // If it is true, use the path planning algorithm

    // Position of the robot in the graphic interface
    x_robot = interface_size.width/2;
    y_robot = interface_size.height - offset;

}

void Control::update(cv::Point* targetPoint2D, Stream* stream)
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

    obstacle_finding(stream->cloud);

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

    // Message for error
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

        // A_star();
    }

}

// Test Phase
void Control::update(cv::Point* targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize)
{

    // Clean the interface matrix
    interface.release();
    interface = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    pcl::PointXYZ refPnt = PointCloud->at(targetPoint2D->x, targetPoint2D->y);

    tmp = (refPnt.x)*scale;
    x_target = x_robot - tmp;
    tmp = (refPnt.z)*scale;
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

    obstacle_finding(PointCloud);

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

    // Message for error
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

        // A_star();
    }

}

void Control::obstacle_finding(PntCld::Ptr cloud)
{

    // Obstacle finding
    for (int i = 0; i<cloud->size()/obstacle_resolution; i++){

        if ( cloud->points[obstacle_resolution*i].y > low_threshold && 
             cloud->points[obstacle_resolution*i].y < up_threshold ){

            tmp = (cloud->points[obstacle_resolution*i].x)*scale;
            int x_p = x_robot - tmp;
            tmp = (cloud->points[obstacle_resolution*i].z)*scale;
            int y_p = y_robot - tmp;

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

void Control::A_star(){

    // To implement

}