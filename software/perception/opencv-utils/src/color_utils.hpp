#ifndef OPENCV_COLOR_UTILS_H__
#define OPENCV_COLOR_UTILS_H__

#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace opencv_utils { 
    cv::Vec3b hsv_to_bgrvec(const cv::Point3f& hsv);
    cv::Scalar hsv_to_bgr(const cv::Point3f& hsv);
    cv::Vec3b bgr_to_hsvvec(const cv::Point3f& bgr);
    cv::Scalar bgr_to_hsv(const cv::Point3f& bgr);
}

#endif // OPENCV_COLOR_UTILS_H
