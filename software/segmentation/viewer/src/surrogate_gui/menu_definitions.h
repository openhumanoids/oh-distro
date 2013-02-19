/*
 * menu_definition.h
 *
 *  Created on: Nov 21, 2011
 *      Author: mfleder
 */

#ifndef MENU_DEFINITION_H_
#define MENU_DEFINITION_H_

namespace surrogate_gui
{
	//=======segmentated objects
	#define PARAM_NAME_NEW_OBJECT "New Object"
	#define PARAM_NAME_CLEAR_OBJECT "Clear ObjPts"
	#define PARAM_NAME_CURR_OBJECT "Current Object"
	#define PARAM_NAME_AFFORDANCE_PUB "Affordance Pub"
	#define NO_OBJECT 0

	//======mouse mode
	#define PARAM_NAME_MOUSE_MODE "Mouse Mode"
	enum MouseMode{CAMERA_MOVE, RECTANGLE_SELECT};

	//======geometric primitive
	#define PARAM_NAME_GEOMETRIC_PRIMITIVE "Geometric Primitive"
	enum GeometricPrimitive{CYLINDER, SPHERE, PLANE, LINE, TORUS, CUBE};

	//======DOF Controls
	#define PARAM_NAME_YAW "Yaw"
	#define PARAM_NAME_PITCH "Pitch"
	#define PARAM_NAME_ROLL "Roll"
	#define PARAM_NAME_MAX_ANGLE "Max Angle"
	#define PARAM_NAME_MIN_RADIUS "Min Radius"
	#define PARAM_NAME_MAX_RADIUS "Max Radius"
	#define PARAM_NAME_DISTANCE_THRESHOLD "Distance Threshold"

	//=============pause
	#define PARAM_NAME_CLOUD_PAUSE "Pause Cloud Updates"
	//#define PARAM_NAME_SHOW_CLOUD "Show Cloud Updates"

	//===========pull from map
	#define PARAM_NAME_PULL_MAP "Get Map"

	//==========tracking
	#define PARAM_NAME_TRACK_METHOD "Tracking Method"
	enum TrackMode{ICP}; //, TRACK_2D};

	//manipulation action type
	#define PARAM_NAME_MANIP_TYPE "Manipulation Type"
	#define PARAM_NAME_TRACK_OBJECTS "Track Live"

	//======reset
	#define PARAM_NAME_RESET "Reset All"
	#define PARAM_NAME_CLEAR_WARNING_MSGS "Clear Warning Msgs"

} //namespace surrogate_gui

#endif /* MENU_DEFINITION_H_ */
