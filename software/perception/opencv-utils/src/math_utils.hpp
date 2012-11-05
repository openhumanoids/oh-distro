#ifndef OPENCV_MATH_UTILS_H__
#define OPENCV_MATH_UTILS_H__

#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace opencv_utils { 
    double l2_dist(const cv::Point& p1, const cv::Point& p2);
    double l2_dist(const cv::Point2f& p1, const cv::Point2f& p2);
    double l2_dist(const cv::Point3f& p1, const cv::Point3f& p2);
    // template<typename _Tp> double l2_dist(const cv::Point_<_Tp>& p1, const cv::Point_<_Tp>& p2);
// template<typename _Tp> inline double l2_dist(const cv::Point3_<_Tp>& p1, const cv::Point3_<_Tp>& p2);

    cv::Mat compute_1D_histogram(cv::Mat& img, cv::Mat& mask, const std::string& name, float min, float max, int nbins);
}

#endif // OPENCV_MATH_UTILS_H
