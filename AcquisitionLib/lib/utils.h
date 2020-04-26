#ifndef UTILS_H_
#define UTILS_H_

#include "./followme.h"
#include "./configurator.h"

/* Set the camera settings */
void camSettings(rs2::config *cfg, ConfigReader* p);

/* Point cloud visualizer */
void PCViewer(PntCld::Ptr cloud, PntCldV::Ptr viewer);


#endif