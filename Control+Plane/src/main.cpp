#include "./lib/followme.h"
#include "./lib/utils.h"
#include "./lib/segmentation.h"
#include "./lib/configurator.h"
#include "./lib/rs_stream.h"
#include "./lib/control.h"


using namespace std;



// --------------------------------------------
// ------------------ main --------------------
// --------------------------------------------
int main (int argc, char** argv)
{
    // ------------------ Configuration file ----------------- //
    
    // Create the configurator object and parse conf.ini file
    ConfigReader *p = ConfigReader::getInstance();
    p->parseFile("../config.ini");

    // ushort leaf;
    // p->getValue("LEAF", (int&) leaf);

    


    // ------------------ OpenCV Part ----------------------- //

    // The image is read (This image corresponds to the .ply file)
    cv::Mat cvFrame = cv::imread("../test.png");

    // Target point choosing
    float y_rel = 0.5, x_rel = 0.46;
    int y_cv = cvFrame.rows * y_rel;
    int x_cv = cvFrame.cols * x_rel;

    cv::Point target_point = cv::Point(x_cv, y_cv);

    // A rectangle is put on the image as a marker
    cv::rectangle(cvFrame,cv::Point(x_cv-5,y_cv-5),cv::Point(x_cv+5,y_cv+5),cv::Scalar(0,0,255),5);




    // ----------------- Plane Identification --------------- //

    PntCld::Ptr cloud_blob (new PntCld);
    PntCld::Ptr cloud_tmp (new PntCld);
    PntCldV::Ptr viewer(new PntCldV ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_blob, 0, 255, 0);
    pcl::PLYReader reader;
    pcl::PLYWriter writer;
    std::stringstream ss;

    // Fill in the cloud data
    reader.read ("../test.ply", *cloud_blob);

    // OBSOLETE: It is done inside Control
    /*
    auto start_plane = std::chrono::high_resolution_clock::now();
    
    // Initialize plane object and pass to it the configurator
    Plane *plane = Plane::getInstance(p);
    
    // Call the update method, to be put in a loop
    plane->update(cloud_blob);

    auto stop_plane = std::chrono::high_resolution_clock::now();
    */

    // If we pass to the progam also an argument, we can test if there are any 
    // obstacle between the robot and the camera 
    if (argv[1] != NULL){
        // We create a whall on the "trajectory" that simulate an obstacle
        float x_start = -700.0;
        float y_start = -800.0;
        pcl::PointXYZ Point;

        for (int i = 0; i < 500; i++){
            for (int j = 0; j < 20; j++){
                Point.x = x_start+5*i;
                Point.y = y_start+5*j;
                Point.z = 1500.0;
                cloud_blob->points.push_back(Point);
            }
        }
    }

    // ----------------Control Part -------------------------- //

    // OBSOLETE: It is done inside Control
    /*
    // Create the point in which we store the 3D position of the target
    pcl::PointXYZ refPnt;
    refPnt = cloud_blob->points[(y_cv-1)*cvFrame.cols+x_cv];

    // auto start_pt = std::chrono::high_resolution_clock::now();

    // Apply the transformation also to the cloud point with the terget point only
    refPnt = pcl::transformPoint(refPnt, plane->transf_mtx);

    auto stop_pt = std::chrono::high_resolution_clock::now();
    */

    // Start chrono time
    auto start_gi = std::chrono::high_resolution_clock::now();

    // Graphic interface for the control
    Control ctrl = Control(p);
    ctrl.update(&target_point, cloud_blob, cv::Size(cvFrame.cols, cvFrame.rows));

    // Stop chrono time
    auto stop_gi = std::chrono::high_resolution_clock::now();

    
    std::cerr << "Times: " << endl;

    // auto duration_plane = std::chrono::duration_cast<std::chrono::milliseconds>(stop_plane - start_plane);
    // cout << endl << "Plane finding        : " << duration_plane.count() << endl;

    // auto duration_pt = std::chrono::duration_cast<std::chrono::milliseconds>(stop_pt - start_pt);
    // cout << endl << "Point tansformation  : " << duration_pt.count() << endl;

    auto duration_gi = std::chrono::duration_cast<std::chrono::microseconds>(stop_gi - start_gi);
    cout << "Overall control algorithm :\t" << (float) duration_gi.count()/1000 << " [ms]" << endl;

    // Print the target point coordinate in the transformed frame
    std::cerr << endl << "Target point: " << endl
              << "x = " << ctrl.refPnt.x << endl
              << "y = " << ctrl.refPnt.y << endl
              << "z = " << ctrl.refPnt.z << endl;





    // --------------- Rappresentation part ------------------ //

    // Add a cube to the visualizer that works as a marker
    viewer->addCube(ctrl.refPnt.x-30,ctrl.refPnt.x+30,
                    ctrl.refPnt.y-30,ctrl.refPnt.y+30,
                    ctrl.refPnt.z-30,ctrl.refPnt.z+30,
                    1.0,0.0,0.0);

    // Apply transformation mtx to plane and pcl
    pcl::transformPointCloud(*cloud_blob, *cloud_tmp, ctrl.plane->transf_mtx);
                             cloud_blob.swap (cloud_tmp);
    pcl::transformPointCloud(*ctrl.plane->easy_cloud, *cloud_tmp, ctrl.plane->transf_mtx);
                            ctrl.plane->easy_cloud.swap (cloud_tmp);
    pcl::transformPointCloud(*ctrl.plane->plane_cloud, *cloud_tmp, ctrl.plane->transf_mtx);
                             ctrl.plane->plane_cloud.swap (cloud_tmp);

    // All the point clouds are added to the visualizer
    viewer->initCameraParameters();
    PCViewer(ctrl.plane->easy_cloud, viewer);
    viewer->addPointCloud(ctrl.plane->plane_cloud, color_handler);
    viewer->addCoordinateSystem(1000, "RF_plane");
    // viewer->addCoordinateSystem(1000, ctrl.plane->transf_mtx, "RF_cam");
    
    while (!viewer->wasStopped())
    {
        cv::imshow("Image",cvFrame);
        cv::imshow("Control",ctrl.interface->interface);
        viewer->spinOnce (100); // wait for some microseconds, makes the viewer interactive

        cv::waitKey(1);
        
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    

    return (0);
}
