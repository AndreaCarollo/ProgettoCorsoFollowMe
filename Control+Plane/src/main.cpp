#include "./lib/followme.h"
#include "./lib/utils.h"
#include "./lib/segmentation.h"


// New include
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


using namespace std;



// --------------------------------------------
// ------------------ main --------------------
// --------------------------------------------
int main (int argc, char** argv)
{

    // ------------------ OpenCV Part ----------------------- //

    // The image is read (This image corresponds to the .ply file)
    cv::Mat cvFrame = cv::imread("../test.png");

    // Target point choosing
    float y_rel = 0.5, x_rel = 0.46;
    int y_cv = cvFrame.rows * y_rel;
    int x_cv = cvFrame.cols * x_rel;

    // A rectangle is put on the image as a marker
    cv::rectangle(cvFrame,cv::Point(x_cv-5,y_cv-5),cv::Point(x_cv+5,y_cv+5),cv::Scalar(0,0,255),5);


    auto start_all = std::chrono::high_resolution_clock::now();


    // ----------------- Plane Identification --------------- //

    PntCld::Ptr cloud_blob (new PntCld), refCld (new PntCld);
    PntCld::Ptr cloud_filtered (new PntCld), cloud_tmp (new PntCld);
    Visualizer::Ptr viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_filtered, 0, 255, 0);
    pcl::PLYReader reader;
    pcl::PLYWriter writer;
    std::stringstream ss;
    Eigen::Vector3f normal = Eigen::Vector3f(0.0, 1.0, 0.0);    // normal to plane to be found, we can "help"
                                                                // the search tilting in the expected direction
    ushort angle = 20;      // [deg] points within +- threshold are inliers of plane
    float threshold = 50;   // [mm] plane can deviate of +- angle in the other two axis

    // Fill in the cloud data
    reader.read ("../test.ply", *cloud_blob);

    // Downsample the original cloud, requires a filtering object with same scale of pcl
    float leaf[3];
    leaf[0] = 200; leaf[1] = 200; leaf[2] = 10;         // [mm]
    downsampling(cloud_blob, cloud_filtered, leaf);

    // Write the downsampled version to disk
    writer.write<pcl::PointXYZ> ("../test_downsampled.ply", *cloud_filtered, false);
    
    // Initialize plane object
    Plane plane(&normal, threshold, angle);
    
    // Call the update method, to be put in a loop
    plane.update(cloud_filtered);

    // Save the inliers on disk
    // writer.write<pcl::PointXYZ> ("../plane.ply", *plane.plane_cloud);

    // Create a point cloud with only the reference point:
    // to this cloud we can apply the transformation
    refCld->width = 1;
    refCld->height = 1;
    refCld->points.resize(1);
    refCld->is_dense = false;
    refCld->points[0] = cloud_blob->points[(y_cv-1)*cvFrame.cols+x_cv];

    // Apply transformation mtx to plane and pcl
    pcl::transformPointCloud(*cloud_filtered, *cloud_tmp, plane.transf_mtx);
    cloud_filtered.swap (cloud_tmp);
    pcl::transformPointCloud(*plane.plane_cloud, *cloud_tmp, plane.transf_mtx);
    plane.plane_cloud.swap (cloud_tmp);

    // Apply the transformation also to the cloud point with the terget point only
    pcl::transformPointCloud(*refCld, *cloud_tmp, plane.transf_mtx);
    refCld.swap (cloud_tmp);

    // Print the target point coordinate in the transformed frame
    std::cerr << endl << "Target point: " << endl
              << "x = " << refCld->points[0].x << endl
              << "y = " << refCld->points[0].y << endl
              << "z = " << refCld->points[0].z << endl;

    // If we pass to the progam also an argument, we can test if there are any 
    // obstacle between the robot and the camera 
    if (argv[1] != NULL){
        // We create a whall on the "trajectory" that simulate an obstacle
        float x_start = -100.0;
        float y_start = 500.0;
        pcl::PointXYZ Point;

        cloud_filtered->points.resize(cloud_filtered->size()+10000);

        for (int i = 0; i < 500; i++){
            for (int j = 0; j < 20; j++){
                Point.x = x_start+i;
                Point.y = y_start+5*j;
                Point.z = 1800.0;

                cloud_filtered->points.push_back(Point);
            }
        }
    }

    // All the point clouds are added to the visualizer
    viewer = simpleVis(cloud_filtered);
    viewer->addPointCloud(plane.plane_cloud, color_handler);
    viewer->addCoordinateSystem(1000, "RF_plane");
    viewer->addCoordinateSystem(1000, plane.transf_mtx, "RF_cam");
    // The cube has the marker meaning
    viewer->addCube(refCld->points[0].x-30,refCld->points[0].x+30,
                    refCld->points[0].y-30,refCld->points[0].y+30,
                    refCld->points[0].z-30,refCld->points[0].z+30,
                    1.0,0.0,0.0);


    
    // ----------------Control Part -------------------------- //

    // Start chrono time
    auto start = std::chrono::high_resolution_clock::now();

    // viewer option for control
    int w_image = 640, h_image = 640;   // human graphic interface size
    int r = 10;                         // Cyrcle radious for robot and target
    int offset = 18;                    // Offset distance (some differene graphic uses)
    float max_dist = 6000.0;            // Maximum ahead distance
    double font_scale = 0.9;            // Font scale for text
    float scale = h_image / max_dist;   // distance scale from real [mm] to graphic interface
    float low_threshold = 10, up_threshold = 1500;      // min and max obstacle heigth

    // Position of the robot in the graphic interface
    int x_robot = w_image/2, y_robot = h_image - offset;

    // Position of the target in the graphic interface
    float tmp;
    tmp = (refCld->points[0].x)*scale;
    int x_target = x_robot - tmp;
    tmp = (refCld->points[0].z)*scale;
    int y_target = y_robot - tmp;

    // Angulat coefficient of the rect that goes from robot to target
    float m = (float)((y_target-y_robot)/(x_target-x_robot));

    // Arrow starting and final point coordinates
    int x1_arrow = - 2 * offset / m + x_robot;
    int y1_arrow = y_robot - 2 * offset;
    int x2_arrow = 2 * offset / m + x_target;
    int y2_arrow = y_target + 2 * offset;

    // Maximum and minimum distance between robot and target
    double dist_max = (x2_arrow - x_robot)^2 + (y2_arrow - y_robot)^2;
    double dist_min = (x1_arrow - x_robot)^2 + (y1_arrow - y_robot)^2;

    // Flag self-explained
    bool there_is_an_obstacle = false;

    // cv::Mat for the graphic interface
    cv::Mat controlMat = cv::Mat(cv::Size(w_image,h_image), CV_8UC3, cv::Scalar(200,200,200));

    // Obstacle finding
    for (int i = 0; i<cloud_filtered->size(); i++){
        if ( cloud_filtered->points[i].y > low_threshold && cloud_filtered->points[i].y < up_threshold ){

            tmp = (cloud_filtered->points[i].x)*scale;
            int x_p = x_robot - tmp;
            tmp = (cloud_filtered->points[i].z)*scale;
            int y_p = y_robot - tmp;

            // put a circle in the interface where there are an obstacle
            cv::circle(controlMat, cv::Point(x_p, y_p), 2, cv::Scalar(0,0,0), 2);

            // Distance from robot and possible-obstacle point
            double dist = (x_p - x_robot)^2 + (y_p - y_robot)^2;

            // Distance from "trajectory" and possible-obstacle point
            int d_rect = (y_p - y_robot) - (float) (m * (x_p - x_robot));

            // If the obstacle is between max and min distance and it is close to the rect,
            // there is an obstacle (self-explained flag = true)
            if ( (dist <= dist_max) && (dist >= dist_min) ){
                if ( abs(d_rect) < r ) {
                    there_is_an_obstacle = true;
                }
            }

        }
    }

    // If there are an obsatacle the control arrow is red, if there isn't the control arrow is red
    cv::Scalar arrowColor = cv::Scalar(255,0,0);

    if (there_is_an_obstacle){
        arrowColor = cv::Scalar(0,0,255);
    }

    // Circle for the robot position
    cv::circle(controlMat, cv::Point(x_robot,y_robot), r, cv::Scalar(0,255,255),3);
    cv::circle(controlMat, cv::Point(w_image - offset,offset), r, cv::Scalar(0,255,255),3);
    cv::putText(controlMat, "Robot", cv::Point(w_image - offset - r - 140, offset + r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,255,255), 2);

    // Green cyrcle for the target postion
    cv::circle(controlMat, cv::Point(x_target,y_target), r, cv::Scalar(0,180,0),3);
    cv::circle(controlMat, cv::Point(w_image - offset,2 * offset + 2 * r), r, cv::Scalar(0,180,0),3);
    cv::putText(controlMat, "Target", cv::Point(w_image - offset - r - 140, 2 * offset + 3 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,180,0), 2);
    
    cv::circle(controlMat, cv::Point(w_image - offset,3 * offset + 4 * r), 2, cv::Scalar(0,0,0), 2);
    cv::putText(controlMat, "Obstacles", cv::Point(w_image - offset - r - 140, 3 * offset + 5 * r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,0), 2);

    cv::putText(controlMat, "If the arrow is red there is", cv::Point(offset, offset + r), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);
    cv::putText(controlMat, "an obstacle on the tarjectory", cv::Point(offset, offset + r + 35 * font_scale), 
                cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0,0,255), 2);   


    // Arrow from robot to target
    cv::arrowedLine(controlMat, cv::Point(x1_arrow, y1_arrow), cv::Point(x2_arrow, y2_arrow),
                    arrowColor, 5);

    // Stop chrono time
    auto stop = std::chrono::high_resolution_clock::now();
    
    // Duration
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    cout << endl << "Duration time        : " << duration.count() << endl;
    auto duration_complete = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start_all);
    cout << endl << "All program duration : " << duration_complete.count() << endl;



    // --------------- Rappresentation part ------------------ //

        
    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100); // wait for some microseconds, makes the viewer interactive
        cv::imshow("Image",cvFrame);
        cv::imshow("Control",controlMat);
        cv::waitKey(1);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    

    return (0);
}