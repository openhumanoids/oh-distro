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
	#define PARAM_NAME_PR2_GRASP "Pr2 Grasp"
	#define NO_OBJECT 0

	//======mouse mode
	#define PARAM_NAME_MOUSE_MODE "Mouse Mode"
	enum MouseMode{CAMERA_MOVE, RECTANGLE_SELECT};

	//=============pause
	#define PARAM_NAME_CLOUD_PAUSE "Pause Cloud Updates"
	//#define PARAM_NAME_SHOW_CLOUD "Show Cloud Updates"


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
