/*
 * ObjectTracking.h
 *
 *  Created on: Nov 30, 2011
 *      Author: mfleder
 */

#ifndef OBJECTTRACKING_H_
#define OBJECTTRACKING_H_

//pcl

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"

#include "DisplayInfo.h"
#include <list>

namespace surrogate_gui
{

	class ObjectTracker
	{

		//=======fields
		public:
			typedef boost::shared_ptr<ObjectTracker> Ptr;
			std::list<pcl::PointXYZ> _centroidTrace; //sequence of tracked centroids

		private:
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr _object_to_track;
			Eigen::Matrix4f _object_transform;
			bool _tracked_once;

			//debug
			int _print_counter;
			const int _PRINT_THROTTLE_RESET;

		//================constructor/destructor
		public:
			ObjectTracker(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_used_for_segmentation,
						  SetIntPtr objectIndices);
			virtual ~ObjectTracker();


		//====tracking methods
		public:
			void findObjectInCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr newest_cloud_pcl,
								   const Eigen::Vector4f &expectedCentroid);
			void findObjectInCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr newest_cloud_pcl);
			pcl::PointXYZ getTrackedCentroid(void);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTrackedObject(void);
			Eigen::Matrix4f getTrackedObjectTransform(void);
			pcl::PointXYZ getPlaneNormal(void);
			bool trackedOnce(void);
	};

} //namespace surrogate_gui

#endif /* OBJECTTRACKING_H_ */
