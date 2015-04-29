#ifndef _planeseg_Types_hpp_
#define _planeseg_Types_hpp_

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace planeseg {

typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointCloud<pcl::PointXYZL> LabeledCloud;
typedef Eigen::Matrix<float,Eigen::Dynamic,3> MatrixX3f;

}

#endif

