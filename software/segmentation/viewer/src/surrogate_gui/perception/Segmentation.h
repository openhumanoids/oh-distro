/*
 * Segmentation.h
 *
 *  Created on: Jan 19, 2012
 *      Author: mfleder
 */

#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <set>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/PolygonMesh.h>

namespace surrogate_gui
{

	/**non-instantiable class for segmenting a point cloud*/
	class Segmentation
	{	private:
			Segmentation();
			virtual ~Segmentation();

		public:
			static std::vector<pcl::PointIndices::Ptr> segment(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
															   boost::shared_ptr<std::set<int> >  subcloudIndices);

			static std::vector<pcl::PointIndices::Ptr> getRansacSegments(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
										     pcl::PointIndices::Ptr subcloudIndices,
										     uint maxNumSegments, pcl::SacModel shapeToFind);

			struct FittingParams{
				double yaw,pitch,roll; // target yaw,pitch,roll
				double maxAngle;       // allowed deviation from yaw,pitch,roll
				double minRadius,maxRadius; // target radius for circular objects
				double distanceThreshold;   // allow distance between points //TODO better description
				
			  FittingParams():yaw(0),pitch(0),roll(0),maxAngle(6.28),minRadius(0.01),maxRadius(0.30),distanceThreshold(0.09){}
			};

                        struct Plane {
                          Eigen::Vector3f xyz, ypr;
                          float width, length;
                          std::vector<Eigen::Vector3f> convexHull;
                          std::vector<std::vector<float> >inliers;
                          std::vector<float> inlier_distances;
                        };

			static pcl::PointIndices::Ptr fitCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
								  boost::shared_ptr<std::set<int> > subcloudIndices,
									const FittingParams& fp,
								  double &x, double &y, double &z,
								  double &roll, double &pitch, double &yaw, 
								  double &radius,
								  double &height,
                  std::vector<Eigen::Vector3f>& points,
                  std::vector<Eigen::Vector3i>& triangles,
								  std::vector<double> &inliers_distances);
			
			static pcl::PointIndices::Ptr fitSphere(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                                                boost::shared_ptr<std::set<int> > subcloudIndices,
                                                                const FittingParams& fp,
                                                                double &x, double &y, double &z,
                                                                double &radius,
                                                                std::vector< std::vector<float> > &inliers);

			static pcl::PointIndices::Ptr fitCircle3d(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                                                  boost::shared_ptr<std::set<int> >  subcloudIndices,
                                                                  const FittingParams& fp,
                                                                  double &x, double &y, double &z,
                                                                  double &roll, double &pitch, double &yaw, 
                                                                  double &radius,
                                                                  std::vector< std::vector<float> > &inliers,
                                                                  std::vector<double> & inliers_distances);

			static void fitPlanes(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                              boost::shared_ptr<std::set<int> > subcloudIndices,
                                              const FittingParams& fp, 
                                              std::vector<Plane>& planeList);

    static void fitPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                              boost::shared_ptr<std::set<int> >  subcloudIndices,
                              const FittingParams& fp, 
                              bool isInitialSet, Eigen::Vector3f initialXYZ, 
                              Eigen::Vector3f initialYPR,
                              Eigen::Vector3f& xyz, Eigen::Vector3f& ypr,
                              std::vector<pcl::PointCloud<pcl::PointXYZRGB> >& clouds);

			static std::vector<pcl::PointIndices::Ptr> getEuclideanClusters(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
																			pcl::PointIndices::Ptr indicesToCluster);


			static double getPlaneFitStatistics(pcl::ModelCoefficients::Ptr planeCoeffs,
											    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
											    pcl::PointIndices::Ptr planeIndices);
                        
			static Eigen::Vector2f getLengthWidth(pcl::PointCloud<pcl::PointXYZRGB>& subcloud, 
                                                              float plane[4],Eigen::Vector3f normal, float theta,
                                                              Eigen::Vector3f& center, Eigen::Vector3f& ypr);
                        
			static Eigen::Vector2f findOptimalRectangle(pcl::PointCloud<pcl::PointXYZRGB>& subcloud, 
                                                                    float plane[4], Eigen::Vector3f& center, 
                                                                    Eigen::Vector3f& ypr);

    static void cloudToTriangles(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                                      pcl::PointIndices::Ptr indices,
                                      std::vector<Eigen::Vector3f>& points,
                                      std::vector<Eigen::Vector3i>& triangles);


	}; //class Segmentation

} //namespace surrogate_gui

#endif /* SEGMENTATION_H_ */

