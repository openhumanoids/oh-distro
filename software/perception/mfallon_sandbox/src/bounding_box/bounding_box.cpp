#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d_omp.h>
//#include <pcl/features/normal_3d_tbb.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/boundary.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/intensity_spin.h>
#include <pcl/features/rift.h>

int
main (int argc, char** argv)
{

  if (argc != 2) {
    fprintf(stderr, "Correct Usage: %s <filename.pcd>\n", argv[0]);
    exit(1);
  }

  std::string file_name = std::string(argv[1]);//  "path_to_your.urdf";


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;


    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;


    // Estimate the XYZ centroid
    pcl::compute3DCentroid (*cloud, xyz_centroid);


    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix (*cloud, xyz_centroid, covariance_matrix);


    // Extract the eigenvalues and eigenvectors
    EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
    EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
    pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);


  std::cout << xyz_centroid << " cent\n";
  std::cout << eigen_vectors << " vectors\n";
  std::cout << covariance_matrix << " a\n";
  std::cout << eigen_values << " c\n";


  return (0);
}
