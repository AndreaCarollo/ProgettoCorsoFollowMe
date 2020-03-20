#include "acq_lib.h"
#include "pcl/io/ply_io.h"
#include <vector>
#include <string.h>


int main() {

    PntCld point_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::string PLYName = "/home/matteo/Documenti/Robotics_Project/Ply+jpeg/test01_05/a/cloud_sn_745412070302_t_001574095435579.ply";
    std::string ImageName = "/home/matteo/Documenti/Robotics_Project/Ply+jpeg/test01_05/a/ir_sn_745412070302_t_001574095435579.png";

    cv::Mat image = cv::imread(ImageName,0);

    pcl::PLYReader reader;
    reader.read(PLYName, *point_cloud);

    PntCldV viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->initCameraParameters();


    while(1){
        PCViewer(point_cloud, viewer);

        cv::imshow("Image", image);

        if (cv::waitKey(1) == 27)
        {
            break;
        }
        viewer -> removePointCloud("sample cloud");
    }
    viewer -> removePointCloud("sample cloud");

    return 0;

}