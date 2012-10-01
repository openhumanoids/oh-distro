/*
 * render_info.h
 *
 *  Created on: Dec 17, 2011
 *      Author: mfleder
 */

#ifndef RENDER_INFO_H_
#define RENDER_INFO_H_

#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include "SurrogateException.h"
#include "Color.h"
#include "ObjectPoints.h"
#include "AdjustableVector.h"

#include <exception>
#include <set>
#include <map>
#include <string>
#include <vector>
#include <list>

namespace surrogate_gui
{

	//==========test display
	typedef struct
	{
		std::string text;
		RGB_PCL color;
	} TextDisplayInfo;

	//============rectangle info
	typedef struct
	{
		int shouldDraw; //0 or 1
		double x0, y0, x1, y1;
	} Rectangle2D;

	//========line segment
	typedef struct
	{
		pcl::PointXYZ endA;
		pcl::PointXYZ endB;
	} LINE_SEG_3D;

	enum AffordanceModel
	{
		WHEEL,
		LEVER,
		UNKNOWN
	};

	//=========circle
	typedef struct
	{
		pcl::PointXYZ planeVec0; //corresponds to theta == 0, like the new x-axis
		pcl::PointXYZ planeVec1; //corresponds to theta == pi/2, new y-axis
		double radius;			  //circle radius
		double minTheta;		 //min observed theta : from [-pi, pi]
		double maxTheta;		 //max observed theta : from [-pi, pi]
		pcl::PointXYZ center;	 //circle center in R3
	} Circle3D;

	//=========segmentation info
	typedef std::map<std::string, ObjectPointsPtr> ObjectNamePointsMap;

	typedef struct
	{
		int numObjects; // number of objects that have been segmented

		//maps from: object name --> that objects point indices
		//where object name is the name in the drop-down menu
		boost::shared_ptr<ObjectNamePointsMap> objectPtsMap;

		bool color_segments; //'s' key: indicates should color segmented objects

	} UserSegmentation;

	typedef struct
	{
		std::vector<Circle3D> circles;
		std::vector<LINE_SEG_3D> lines;
		bool haveModel;
		Circle3D rotationAxisGuess;
		bool drawCircles;
		bool drawLines;
		bool displayOn;
	} ModelFit;


	//============Object Tracking
	typedef struct
	{
		std::list<pcl::PointXYZ> centroidTrace; //time history of centroid location
		pcl::PointXYZ planeNormal; //normal to planeFit(trackedObject).  vector hopefully approx starts at objTrackCentroid
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr trackedObject;
		Eigen::Matrix4f trackedObjectTransform;
		bool displayTrackingCloud;
	} TrackingInfo;

	//==========Object grasping
	typedef struct
	{
		AdjustableVector forceVector;
		AdjustableVector rotationAxis;
		bool displayTrajectory;
		//std::vector<pcl::PointXYZ> trajectory; //ordered sequence of points for full-circle trajectory
	} GraspInfo;

	//===========render info

	typedef struct
	{
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
		std::vector<Color::StockColor> highlightColors;  //indices correspond to those in cloud
		bool displayLcmCloud;
		std::string lcmCloudFrameId;

		Rectangle2D rectangle2D;
		UserSegmentation user_seg_info;
		ModelFit model_fit;
		TrackingInfo tracking_info;
		GraspInfo graspInfo;
	} SurrogateDisplayInfo;

	//=================
	enum CameraView {CAMERA_FRONT, CAMERA_TOP, CAMERA_SIDE};

	//=======

} //namespace surrogate_gui

#endif /* RENDER_INFO_H_ */
