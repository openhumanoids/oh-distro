#ifndef OPENCV_PLOT_UTILS_H__
#define OPENCV_PLOT_UTILS_H__

#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace opencv_utils { 
    cv::Scalar heatmap_bgr( float val );
    cv::Vec3b heatmap_bgrvec( float val );
    void fillColors( std::vector<cv::Scalar>& colors );
    void plot_1D_hist(cv::Mat& hist, int bins, const std::string& name);
}

#endif // OPENCV_PLOT_UTILS_H
