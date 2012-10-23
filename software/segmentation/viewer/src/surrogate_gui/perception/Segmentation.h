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

			static pcl::PointIndices::Ptr fitCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
								  boost::shared_ptr<std::set<int> > subcloudIndices,
								  double &x, double &y, double &z,
								  double &roll, double &pitch, double &yaw, 
								  double &radius,
								  double &height);

			static std::vector<pcl::PointIndices::Ptr> getEuclideanClusters(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
																			pcl::PointIndices::Ptr indicesToCluster);


			static double getPlaneFitStatistics(pcl::ModelCoefficients::Ptr planeCoeffs,
											    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
											    pcl::PointIndices::Ptr planeIndices);
	}; //class Segmentation

} //namespace surrogate_gui

#endif /* SEGMENTATION_H_ */

