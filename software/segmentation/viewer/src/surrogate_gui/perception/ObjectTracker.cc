/*
 * ObjectTracking.cpp
 *
 *  Created on: Nov 30, 2011
 *      Author: mfleder
 */

#include "ObjectTracker.h"
#include "PclSurrogateUtils.h"
#include <set>

#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h> //for computePointNormal
#include <sys/time.h>

using namespace std;
using namespace pcl;

namespace surrogate_gui
{

	//===================constructor / destructor
	ObjectTracker::ObjectTracker(const PointCloud<PointXYZRGB>::ConstPtr cloud_used_for_segmentation,
								  SetIntPtr objectIndices)
		: _tracked_once(false), _print_counter(10), _PRINT_THROTTLE_RESET(10)
	{
		//_cloud_used_for_segmentation = lkr_color_point_cloud_t_copy(cloud_used_for_segmentation);
		_object_to_track 	= PclSurrogateUtils::extractIndexedPoints(objectIndices, cloud_used_for_segmentation);
		//_object_to_track 	= downsample(_object_to_track);
	}
	//	PointCloud<PointXYZRGB>::Ptr ObjectTracker::extractIndexedPoints(const vector<int> &indices, PointCloud<PointXYZRGB>::Ptr originalCloud)

	ObjectTracker::~ObjectTracker()
	{
		//lkr_color_point_cloud_t_destroy(_cloud_used_for_segmentation);
	}



	//===========tracking methods

	//====debugging
	/*struct timeval {
	  time_t tv_sec;
	  suseconds_t tv_usec;
	};*/
	//=========


	/**@modifies _last_object_location*/
	void ObjectTracker::findObjectInCloud(const PointCloud<PointXYZRGB>::ConstPtr newest_cloud_pcl,
										  const Eigen::Vector4f &expectedCentroid)
	{
		//PointCloud<PointXYZRGB>::Ptr downsampled_cloud = downsample(newest_cloud_pcl);

		//====make a translation guess
		/*PointXYZ lastCentroid = getTrackedCentroid();
		Eigen::Vector4f lastObjTrackCentr(lastCentroid.x,  lastCentroid.y, lastCentroid.z, 0);
		Eigen::Vector4f difference = lastObjTrackCentr - expectedCentroid;
		Eigen::Matrix4f translationGuess = Eigen::Matrix4f::Identity();
		translationGuess(0,3) = difference(0);
		translationGuess(1,3) = difference(1);
		translationGuess(2,3) = difference(2);
		printf("\ntranslation guess = (%f, %f, %f)\n", difference(0), difference(1), difference(2));
		*/

		//iterative closest point : setup inputs
		IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
		icp.setInputCloud(PclSurrogateUtils::downsample(_object_to_track, 4));
		icp.setInputTarget(PclSurrogateUtils::downsample(newest_cloud_pcl, 4));

		if (_print_counter <= 0)
		{
			cout << "\n downsampled big cloud from size = " << newest_cloud_pcl->points.size()
					<< " to " << icp.getInputTarget()->size()
					<< " and object from size: " << _object_to_track->points.size() <<
					" to " << icp.getInputTarget()->size() << endl;

			cout << "\ndoing find object w/ sizes " << icp.getInputTarget()->size() << " : and "
					<< icp.getInputCloud()->size() << endl;
		}

		//outputs
		//icp.setMaxCorrespondenceDistance(0.05);

		//=========timer start
		timeval start, end;
		long mtime, seconds, useconds;
		gettimeofday(&start, NULL);

		//---icp
		PointCloud<PointXYZRGB>::Ptr downsampled_output (new PointCloud<PointXYZRGB>);
		icp.align(*downsampled_output);
		//icp.align(*_object_to_track); //*newest_cloud_pcl); //*_object_to_track;

		//---transform: apply transform from the downsampled output to the full-res _object_to_track
		if (_tracked_once)
			_object_transform = icp.getFinalTransformation()*_object_transform;
		else
			_object_transform = icp.getFinalTransformation();

		pcl::transformPointCloud(*_object_to_track, *_object_to_track, icp.getFinalTransformation());


		gettimeofday(&end, NULL);
		seconds  = end.tv_sec  - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
		//==============timer end

		if (_print_counter-- <= 0)
		{
			printf("\nicp align + transform time: %ld milliseconds\n", mtime);
			cout << "\n tracking has run " << _PRINT_THROTTLE_RESET << " times" << endl;
			_print_counter = _PRINT_THROTTLE_RESET;
		}

		//cout << "icp has converged: " << icp.hasConverged() << " score: " <<
		//			icp.getFitnessScore() << endl;
		//cout << icp.getFinalTransformation() << endl;

		_tracked_once = true;

		_centroidTrace.push_back(getTrackedCentroid());
	}

	/**searchest for _object_to_track in newest_cloud and
	 * updates member variables accordingly*/
	void ObjectTracker::findObjectInCloud(const PointCloud<PointXYZRGB>::ConstPtr newest_cloud_pcl)
	{

		//PointCloud<PointXYZRGB>::Ptr downsampled_cloud = downsample(newest_cloud_pcl);
		PointXYZ c = getTrackedCentroid();
		findObjectInCloud(newest_cloud_pcl, Eigen::Vector4f(c.x, c.y, c.z,0));
	}

	/**Computes a plane fit to the current object and returns
	 * the normal.  The normal vector starts at the centroid.*/
	PointXYZ ObjectTracker::getPlaneNormal()
	{
		//----------compute plane fit
		//if plane is: Ax + By + Cy = D.  The normal is (A,B,C)
		Eigen::Vector4f plane_parameters;
		float curvature;
		pcl::computePointNormal(*_object_to_track, plane_parameters, curvature);
		PointXYZ normal(plane_parameters[0],
						plane_parameters[1],
						plane_parameters[2]);
		return normal;
	}

	PointXYZ ObjectTracker::getTrackedCentroid(void)
	{
		Eigen::Vector4f centroid4f;
		compute3DCentroid(*_object_to_track, centroid4f);
		PointXYZ centroid(centroid4f[0], centroid4f[1], centroid4f[2]);
		return centroid;
	}

	PointCloud<PointXYZRGB>::Ptr ObjectTracker::getTrackedObject(void)
	{
		return _object_to_track;
	}

	Eigen::Matrix4f ObjectTracker::getTrackedObjectTransform(void)
	{
		return _object_transform;
	}

	bool ObjectTracker::trackedOnce(void)
	{
		return _tracked_once;
	}

} //namespace surrogate_gui
