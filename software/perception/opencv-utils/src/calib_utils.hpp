#ifndef CALIB_UTILS_H
#define CALIB_UTILS_H

#include <opencv2/opencv.hpp>
#include <bot_core/bot_core.h>

// namespace opencv_utils {
#ifndef PI
#define PI 3.14159265358979323846
#endif

// normalize the point
cv::Point2f normalize_point(const cv::Point2f& p, cv::Mat_<double>& Kmat);

// backproject to 3D with a given depth
cv::Point3f backproject(const cv::Point2f& p, double depth, cv::Mat_<double>& Kmat, cv::Mat_<double>& Rmat, cv::Mat_<double>& tmat);

// pinhole camera = [fx fy skew cx cy] -> K
cv::Mat_<double> pinhole_to_mat(double* pinhole); 

// conversion from double
cv::Mat_<double> distortion_to_mat(double* d);

// Extracts the extrinsics in T = [rvec tvec] where rvec is the rodrigues form, and tvec is as usual
void extract_camera_extrinsics(cv::Mat_<double>& T, cv::Mat_<double>& rvec, cv::Mat_<double>& tvec);

// Combines R and t to form T = [R t]
cv::Mat_<double> Rt_combine(cv::Mat_<double>& R, cv::Mat_<double>& t);

// Extracts R and t from T = [R t]
void Rt_extract(cv::Mat_<double>& T, cv::Mat_<double>& R, cv::Mat_<double>& t);

// Assumption is that this is T = [R t] where R is a rotational transform (orthogonal), and t is translation
cv::Mat_<double> invert_T(cv::Mat_<double>& T);


// }

#endif
