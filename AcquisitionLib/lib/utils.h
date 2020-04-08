#ifndef UTILS_H_
#define UTILS_H_

#include "./followme.h"

/* Set the camera settings */
void camSettings(rs2::config *cfg);

/* Point cloud visualizer */
void PCViewer(PntCld::Ptr cloud, PntCldV::Ptr viewer);


#endif