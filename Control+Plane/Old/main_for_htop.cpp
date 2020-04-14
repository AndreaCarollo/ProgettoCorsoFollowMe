#include "./lib/followme.h"
#include "./lib/utils.h"
#include "./lib/segmentation.h"
#include "./lib/configurator.h"
#include "./lib/rs_stream.h"


using namespace std;



// --------------------------------------------
// ------------------ main --------------------
// --------------------------------------------
int main (int argc, char** argv)
{
    // ------------------ Initialization part ----------------- //

    // Initialize the configurator object and parse conf.ini file
    ConfigReader *p = ConfigReader::getInstance();
    p->parseFile("../config.ini");

    // Initialize the plane object and the configurator to it
    Plane *plane = Plane::getInstance(p);

    // Create plane, viewer and other stuff
    PntCld::Ptr cloud_blob (new PntCld);
    // PntCldV::Ptr viewer(new PntCldV ("3D Viewer"));
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_blob, 0, 255, 0);
    pcl::PLYReader reader;
    
    int i=0;
    while (true)
    {
        // Fill in the cloud data
        std::stringstream ss;
        ss << "../test" << i%2 << ".ply";
        reader.read (ss.str(), *cloud_blob);
    
        // Call the update method, to be put in a loop
        plane->update(cloud_blob);
        i++;
    }

    return (0);
}