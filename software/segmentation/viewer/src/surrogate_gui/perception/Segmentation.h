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

			static pcl::PointIndices::Ptr fitCylinderNew(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
								  boost::shared_ptr<std::set<int> > subcloudIndices,
								  double &x, double &y, double &z,
								  double &roll, double &pitch, double &yaw, 
								  double &radius,
								  double &height);
			static pcl::PointIndices::Ptr fitCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
								  boost::shared_ptr<std::set<int> > subcloudIndices,
									const FittingParams& fp,
								  double &x, double &y, double &z,
								  double &roll, double &pitch, double &yaw, 
								  double &radius,
								  double &height,
								  std::vector<double> &inliers_distances);
			
			static pcl::PointIndices::Ptr fitSphere(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
								  boost::shared_ptr<std::set<int> > subcloudIndices,
									const FittingParams& fp,
								  double &x, double &y, double &z,
								  double &radius);



			static pcl::PointIndices::Ptr fitCircle3d(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
								boost::shared_ptr<std::set<int> >  subcloudIndices,
								const FittingParams& fp,
					      double &x, double &y, double &z,
					      double &roll, double &pitch, double &yaw, 
					      double &radius,
					      std::vector<double> & inliers_distances);

			static std::vector<pcl::PointIndices::Ptr> getEuclideanClusters(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
																			pcl::PointIndices::Ptr indicesToCluster);


			static double getPlaneFitStatistics(pcl::ModelCoefficients::Ptr planeCoeffs,
											    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
											    pcl::PointIndices::Ptr planeIndices);
	}; //class Segmentation

} //namespace surrogate_gui

#endif /* SEGMENTATION_H_ */

