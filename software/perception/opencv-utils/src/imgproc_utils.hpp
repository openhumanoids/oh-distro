#ifndef OPENCV_IMGPROC_UTILS_H__
#define OPENCV_IMGPROC_UTILS_H__

#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace opencv_utils { 
    void simple_sobel(cv::Mat& dst, cv::Mat& img);
}

#endif // OPENCV_IMGPROC_UTILS_H
