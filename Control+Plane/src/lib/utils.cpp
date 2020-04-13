#include "utils.h"
#include "rs_stream.h"

const int IMAGE_WIDTH  = 640;
const int IMAGE_HEIGTH = 480;
const int FRAME_RATE   =  30;

cv::Size IMAGE_SIZE = cv::Size(IMAGE_WIDTH, IMAGE_HEIGTH);


// --------------------------------------------
// ----------------- FUNCTIONS ----------------
// --------------------------------------------

void camSettings(rs2::config *cfg){
    
    // Add desired streams to configuration
    cfg->enable_stream(RS2_STREAM_COLOR,    IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_BGR8, FRAME_RATE);
    cfg->enable_stream(RS2_STREAM_INFRARED, IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_Y8,   FRAME_RATE);
    cfg->enable_stream(RS2_STREAM_DEPTH,    IMAGE_WIDTH, IMAGE_HEIGTH, RS2_FORMAT_Z16,  FRAME_RATE);

}

void camSettings(rs2::config *cfg, int image_width, int image_heigth, int fps){
    
    // Add desired streams to configuration
    cfg->enable_stream(RS2_STREAM_COLOR,    image_width, image_heigth, RS2_FORMAT_BGR8, fps);
    cfg->enable_stream(RS2_STREAM_INFRARED, image_width, image_heigth, RS2_FORMAT_Y8,   fps);
    cfg->enable_stream(RS2_STREAM_DEPTH,    image_width, image_heigth, RS2_FORMAT_Z16,  fps);

}

void PCViewer(PntCld::Ptr cloud, PntCldV::Ptr viewer){
    
    // Set all the parameters of the point cloud viewer and add to it the point cloud that we want to reppresent
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // viewer->addCoordinateSystem(0.001);    

}

Visualizer::Ptr simpleVis (PntCld::ConstPtr cloud)
{
    Visualizer::Ptr viewer (new Visualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (0.001);
    viewer->initCameraParameters ();
    return (viewer);
}

void downsampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, float* leaf)
{  

    pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud_in, *cloud2);

    // std::cerr << "PointCloud before filtering: " << cloud_in->width * cloud_in->height
    //             << " data points." << std::endl;

    pcl::PCLPointCloud2 cloud_tmp;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud2);
    sor.setLeafSize (leaf[0], leaf[1], leaf[2]);
    sor.filter (cloud_tmp);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2(cloud_tmp, *cloud_out);

    // std::cerr << "PointCloud after filtering: " << cloud_out->width * cloud_out->height
    //             << " data points." << std::endl;

}


void down_sampling(PntCld::Ptr cloud_in, PntCld::Ptr cloud_out, int n)
{
    size_t i = 0.7*cloud_in->size();
    do{
        cloud_out->points.push_back(cloud_in->at(i));
        i +=  n;
    }while( i < cloud_in->size());
}


void interfaceBuilding (cv::Mat *output_matrix, cv::Point targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize,  ConfigReader *p) { 

    // Interface option

    // float max_dist = 6000.0;                             // Maximum ahead distance
    cv::Size interface_size = cv::Size(640,640);            // human graphic interface size
    int r = 18, offset = 18;                                // r                   -> Cyrcle radious for robot and target
                                                            // offset              -> Offset distance (some differene graphic uses)
                                                            // obstacle_resolution -> How many point for the image lenght (obstacle density) (=54)
    double font_scale = 0.9f;                               // Font scale for text

    cv::Scalar backgroundColor, obstacleColor, targetColor, robotColor, arrowColor;
    p->getValue("BG_COLOR", backgroundColor);
    p->getValue("ARROW_COLOR", arrowColor);
    p->getValue("OBSTACLE_COLOR", obstacleColor);
    p->getValue("ROBOT_COLOR", robotColor);
    p->getValue("TARGET_COLOR", targetColor);

    float max_dist, low_threshold, up_threshold;
    p->getValue("LOOK_AHEAD_DIST", max_dist);
    p->getValue("OBST_MIN_THRESH", low_threshold);
    p->getValue("OBST_MAX_THRESH", up_threshold);

    ushort obstacle_resolution;
    p->getValue("OBSTACLE_GRAIN", (int&) obstacle_resolution);

    float scale = interface_size.height / max_dist;         // distance scale from real [mm] to graphic interface
    // float low_threshold = 10, up_threshold = 1500;       // min and max obstacle heigth



    // Position of the robot in the graphic interface
    int x_robot = interface_size.width/2, y_robot = interface_size.height - offset;

    // Position of the target in the graphic interface
    pcl::PointXYZ refPnt = PointCloud->points[(targetPoint2D.y-1)*cvFrameSize.width+targetPoint2D.x];

    float tmp;
    tmp = (refPnt.x)*scale;
    int x_target = x_robot - tmp;
    tmp = (refPnt.z)*scale;
    int y_target = y_robot - tmp;

    // Angulat coefficient of the rect that goes from robot to target
    float m = (float)((y_target-y_robot)/(x_target-x_robot));

    // Arrow starting and final point coordinates
    int x1_arrow = - 2 * offset / m + x_robot;
    int y1_arrow = y_robot - 2 * offset;
    int x2_arrow = 2 * offset / m + x_target;
    int y2_arrow = y_target + 2 * offset;

    // Maximum and minimum distance between robot and target
    double dist_rt  = (x_target - x_robot)^2 + (y_target - y_robot)^2;
    double dist_max = (x2_arrow - x_robot)^2 + (y2_arrow - y_robot)^2;
    double dist_min = (x1_arrow - x_robot)^2 + (y1_arrow - y_robot)^2;

    // Flag self-explained
    bool there_is_an_obstacle = false;

    // cv::Mat for the graphic interface
    output_matrix->release();
    (*output_matrix) = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    // Obstacle finding
    for (int i = 0; i<PointCloud->size()/obstacle_resolution; i++){

        if ( PointCloud->points[obstacle_resolution*i].y > low_threshold && 
             PointCloud->points[obstacle_resolution*i].y < up_threshold ){

            tmp = (PointCloud->points[obstacle_resolution*i].x)*scale;
            int x_p = x_robot - tmp;
            tmp = (PointCloud->points[obstacle_resolution*i].z)*scale;
            int y_p = y_robot - tmp;

            // put a circle in the interface where there are an obstacle
            cv::circle(*output_matrix, cv::Point(x_p, y_p), 2, obstacleColor, 2);

            // Distance from "trajectory" and possible-obstacle point
            int d_rect = ((y_p - y_robot) / m) - (x_p - x_robot);

            // If the obstacle is between max and min distance and it is close to the rect,
            // there is an obstacle (self-explained flag = true)
            if ( abs(d_rect) < r ) {

		// Distance from robot and possible-obstacle point
            	double dist = (x_p - x_robot)^2 + (y_p - y_robot)^2;

                if ( (dist <= dist_max) && (dist >= dist_min) ) {
                    // If there are an obsatacle the control arrow is red, if there isn't the control arrow is red
                    arrowColor = cv::Scalar(0,0,255);
                }
            }

        }
    }


    // Circle for the robot position
    cv::circle(*output_matrix, cv::Point(x_robot,y_robot), r, robotColor, cv::FILLED, cv::LINE_8);
    cv::circle(*output_matrix, cv::Point(interface_size.width - offset,offset), r, robotColor, cv::FILLED, cv::LINE_8);
    cv::putText(*output_matrix, "Robot", cv::Point(interface_size.width - offset - r - 140, offset + r/2), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, robotColor, 2);

    // Green cyrcle for the target postion
    cv::circle(*output_matrix, cv::Point(x_target,y_target), r, targetColor, cv::FILLED, cv::LINE_8);
    cv::circle(*output_matrix, cv::Point(interface_size.width - offset,2 * offset + 2 * r), r, targetColor, cv::FILLED, cv::LINE_8);
    cv::putText(*output_matrix, "Target", cv::Point(interface_size.width - offset - r - 140, 2 * offset + 2.5 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, targetColor, 2);
    
    cv::circle(*output_matrix, cv::Point(interface_size.width - offset,3 * offset + 4 * r), 4, obstacleColor, cv::FILLED, cv::LINE_8);
    cv::putText(*output_matrix, "Obstacles", cv::Point(interface_size.width - offset - r - 140, 3 * offset + 4.5 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, obstacleColor, 2);

    cv::putText(*output_matrix, "If the arrow is red there is", cv::Point(offset, offset + r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
    cv::putText(*output_matrix, "an obstacle on the tarjectory", cv::Point(offset, offset + r + 35 * font_scale), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);   


    // Arrow from robot to target (the arrow is shown only if the distance of the robot from the target is under a certain threshold {1 mt})
    if ( dist_rt >= 1000*scale ) {
        cv::arrowedLine(*output_matrix, cv::Point(x1_arrow, y1_arrow), cv::Point(x2_arrow, y2_arrow),
                        arrowColor, 5);
    }

}


void interfaceBuilding (cv::Mat *output_matrix, cv::Point* targetPoint2D, Stream* stream, ConfigReader *p) { 

    // Interface option

    // float max_dist = 6000.0;                                // Maximum ahead distance
    cv::Size interface_size = cv::Size(640,640);            // human graphic interface size
    int r = 10, offset = 18;// , obstacle_resolution = 54;      // r                   -> Cyrcle radious for robot and target
                                                            // offset              -> Offset distance (some differene graphic uses)
                                                            // obstacle_resolution -> How many point for the image lenght (obstacle density)
    double font_scale = 0.9f;                               // Font scale for text

    cv::Scalar backgroundColor, obstacleColor, targetColor, robotColor, arrowColor;
    p->getValue("BG_COLOR", backgroundColor);
    p->getValue("ARROW_COLOR", arrowColor);
    p->getValue("OBSTACLE_COLOR", obstacleColor);
    p->getValue("ROBOT_COLOR", robotColor);
    p->getValue("TARGET_COLOR", targetColor);

    float max_dist, low_threshold, up_threshold;
    p->getValue("LOOK_AHEAD_DIST", max_dist);
    p->getValue("OBST_MIN_THRESH", low_threshold);
    p->getValue("OBST_MAX_THRESH", up_threshold);

    ushort obstacle_resolution;
    p->getValue("OBSTACLE_GRAIN", (int&) obstacle_resolution);

    float scale = interface_size.height / max_dist;         // distance scale from real [mm] to graphic interface
    // float low_threshold = 10, up_threshold = 1500;          // min and max obstacle heigth



    // Position of the robot in the graphic interface
    int x_robot = interface_size.width/2, y_robot = interface_size.height - offset;

    // Projection from RGB to depth
    stream->project_RGB2DEPTH(targetPoint2D);

    float tmp;
    tmp = (stream->refPnt.x)*scale;
    int x_target = x_robot - tmp;
    tmp = (stream->refPnt.z)*scale;
    int y_target = y_robot - tmp;

    // Angulat coefficient of the rect that goes from robot to target
    float m = (float)((y_target-y_robot)/(x_target-x_robot));

    // Arrow starting and final point coordinates
    int x1_arrow = - 2 * offset / m + x_robot;
    int y1_arrow = y_robot - 2 * offset;
    int x2_arrow = 2 * offset / m + x_target;
    int y2_arrow = y_target + 2 * offset;

    // Maximum and minimum distance between robot and target
    double dist_rt  = (x_target - x_robot)^2 + (y_target - y_robot)^2;
    double dist_max = (x2_arrow - x_robot)^2 + (y2_arrow - y_robot)^2;
    double dist_min = (x1_arrow - x_robot)^2 + (y1_arrow - y_robot)^2;

    // Flag self-explained
    bool there_is_an_obstacle = false;

    // cv::Mat for the graphic interface
    output_matrix->release();
    (*output_matrix) = cv::Mat(interface_size, CV_8UC3, backgroundColor);

    // Obstacle finding
    for (int i = 0; i<stream->cloud->size()/obstacle_resolution; i++){

        if ( stream->cloud->points[obstacle_resolution*i].y > low_threshold && 
             stream->cloud->points[obstacle_resolution*i].y < up_threshold ){

            tmp = (stream->cloud->points[obstacle_resolution*i].x)*scale;
            int x_p = x_robot - tmp;
            tmp = (stream->cloud->points[obstacle_resolution*i].z)*scale;
            int y_p = y_robot - tmp;

            // put a circle in the interface where there are an obstacle
            cv::circle(*output_matrix, cv::Point(x_p, y_p), 2, obstacleColor, 2);

            // Distance from "trajectory" and possible-obstacle point
            int d_rect = ((y_p - y_robot) / m) - (x_p - x_robot);

            // If the obstacle is between max and min distance and it is close to the rect,
            // there is an obstacle (self-explained flag = true)
            if ( abs(d_rect) < r ) {

		// Distance from robot and possible-obstacle point
            	double dist = (x_p - x_robot)^2 + (y_p - y_robot)^2;

                if ( (dist <= dist_max) && (dist >= dist_min) ) {
                    // If there are an obsatacle the control arrow is red, if there isn't the control arrow is red
                    arrowColor = cv::Scalar(0,0,255);
                }
            }

        }
    }


    // Circle for the robot position
    cv::circle(*output_matrix, cv::Point(x_robot,y_robot), r, robotColor, cv::FILLED, cv::LINE_8);
    cv::circle(*output_matrix, cv::Point(interface_size.width - offset,offset), r, robotColor,  cv::FILLED, cv::LINE_8);
    cv::putText(*output_matrix, "Robot", cv::Point(interface_size.width - offset - r - 140, offset + r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, robotColor, 2);

    // Green cyrcle for the target postion
    cv::circle(*output_matrix, cv::Point(x_target,y_target), r, targetColor, cv::FILLED, cv::LINE_8);
    cv::circle(*output_matrix, cv::Point(interface_size.width - offset,2 * offset + 2 * r), r, targetColor, cv::FILLED, cv::LINE_8);
    cv::putText(*output_matrix, "Target", cv::Point(interface_size.width - offset - r - 140, 2 * offset + 3 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, targetColor, 2);
    
    cv::circle(*output_matrix, cv::Point(interface_size.width - offset,3 * offset + 4 * r), 4, obstacleColor,  cv::FILLED, cv::LINE_8);
    cv::putText(*output_matrix, "Obstacles", cv::Point(interface_size.width - offset - r - 140, 3 * offset + 5 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, obstacleColor, 2);

    cv::putText(*output_matrix, "If the arrow is red there is", cv::Point(offset, offset + r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
    cv::putText(*output_matrix, "an obstacle on the trajectory", cv::Point(offset, offset + r + 35 * font_scale), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);   


    // Arrow from robot to target (the arrow is shown only if the distance of the robot from the target is under a certain threshold {1 mt})
    if ( dist_rt >= 1000*scale ) {
        cv::arrowedLine(*output_matrix, cv::Point(x1_arrow, y1_arrow), cv::Point(x2_arrow, y2_arrow),
                        arrowColor, 5);
    }
                    
}
