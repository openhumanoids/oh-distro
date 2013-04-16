#ifndef PRINT_UTILS_H__
#define PRINT_UTILS_H__

#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace opencv_utils { 
    std::ostream& operator << (std::ostream& os, const cv::Rect& r);
}

#endif // PRINT_UTILS_H
