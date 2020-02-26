#ifdef PERSONLIBRARY
#define PERSONLIBRARY
#else
#define PERSONLIBRARY
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracking.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// LIBRARY FUNCTIONS

void detection_on_frame(Mat *frame, vector<Rect> *peoples);