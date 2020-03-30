//  ______    _____    __       __        _____   ___          ___  __      __   ______
// |   ___|  /  _  \  |  |     |  |      /  _  \  \  \        /  / |   \  /   | |  ____| 
// |  |__   |  | |  | |  |     |  |     |  | |  |  \  \  /\  /  /  | |\ \/ /| | | |___
// |   __|  |  | |  | |  |     |  |     |  | |  |   \  \/  \/  /   | | \__/ | | |  ___|
// |  |     |  |_|  | |  |___  |  |___  |  |_|  |    \   /\   /    | |      | | | |____
// |__|      \_____/  |______| |______|  \_____/      \_/  \_/     |_|      |_| |______|
//
// This is the common project header file
// Declare and define here types and structures that are of common usage


// Double inclusion guard
#ifndef FOLLOWME_H
#define FOLLOWME_H

// System headers
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <chrono> // for time measurement

// Point Cloud headers
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

// Data types
typedef uint32_t index_t; // for indexes
typedef pcl::PointCloud<pcl::PointXYZ> PntCld;


#endif