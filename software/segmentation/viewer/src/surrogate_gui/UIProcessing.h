/*
 * ui_processing.h
 *
 *  Created on: Dec 17, 2011
 *      Author: mfleder
 */

#ifndef UIPROCESSING_H_
#define UIPROCESSING_H_

//================includes
#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_vis/glm.h>
#include <bot_frames/bot_frames.h>
//#include <bot_param/param_client.h>
//#include <bot_param/param_util.h>
//#include "param_widget.h"

#include <lcmtypes/lkr_color_point_cloud_t.h>
#include <lcmtypes/lkr_point_xyzrgb_t.h>
#include <lcmtypes/lkr_point_xyz_t.h>
#include <lcmtypes/ptools_pointcloud2_t.h>
#include <lcmtypes/ptools_pointfield_t.h>
#include <vector>
#include <string>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>


#include "menu_definitions.h"
#include "DisplayInfo.h"
#include "SurrogateRenderer.h"
#include "ObjectTracker.h"
#include "EdgesToR3.h"

namespace surrogate_gui
{

	//===========state enumeration
	enum GuiState {SEGMENTING, TRACKING};

	typedef struct
	{
		bool shift_L_is_down;
		bool shift_R_is_down;
	} ButtonStates;


	//================class UI_Processing

	/**Class for processing user input: keystrokes, menu selections, mouse button clicks*/
	class UIProcessing
	{
		//======fields
		private:
			GuiState _gui_state; //segmenting or tracking
			SurrogateRenderer _surrogate_renderer; //graphics, display
			lcm_t     *_lcm; //incoming messages
			ButtonStates _button_states;  //for monitoring which keys are being held down

			BotEventHandler *_ehandler;
			BotEventHandler *_mode_handler;

			boost::shared_ptr<ObjectTracker> _objTracker;
			boost::shared_ptr<EdgesToR3> _edgeFinder;

		//=====constructor/destructor
		public:
			UIProcessing(BotViewer *viewer, lcm_t *lcm, std::string kinect_channel);
			~UIProcessing();

		//=====observers
		public:
			MouseMode getMouseMode() const; //rectangle or camera move
			std::string getCurrentObjectSelectedName() const;
			ObjectPointsPtr getCurrentObjectSelected(); //object currently being segmented
			SurrogateDisplayInfo *getDisplayInfo();
			TrackMode getTrackMode() const;
			int8_t getManipulationType() const;
		//=====mutators
		private:

			//segmentation
			void addIndicesToCurrentObject(std::set<int> &selected_indices);
			void intersectViews(std::set<int> &selected_indices);
			void suggestSegments();
			void runModelFitting();

			void handleMouseModeAdjust(BotEventHandler *default_handler, BotGtkParamWidget *pw); //user clicks on MouseMode drop-down menu
			void handleTrackLiveButton(BotGtkParamWidget *pw);
			void handlePr2GraspButton();
			void handleFullResetButton(BotGtkParamWidget *pw);

			static void copy(const pcl::PointXYZ &p, lkr_point_xyz_t &dest);

			//callbacks
			void on_param_widget_changed_xyzrgb (BotGtkParamWidget *pw, const char *name, void *user); //menu item clicked / adjusted
			int mouse_press(BotViewer *viewer, BotEventHandler *ehandler,
							const double ray_start[3], const double ray_dir[3], const GdkEventButton *event);
			int mouse_release (BotViewer *viewer, BotEventHandler *ehandler,
							 const double ray_start[3], const double ray_dir[3], const GdkEventButton *event);
			int mouse_motion(BotViewer *viewer, BotEventHandler *ehandler,
							 const double ray_start[3], const double ray_dir[3], const GdkEventMotion *event);

			int key_press (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey *event);
			int key_release (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey  *event);

			void right_click_popup(GtkWidget *menuItem);

			void unpack_pointcloud2(const ptools_pointcloud2_t *msg,
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

			void on_kinect_frame (const lcm_recv_buf_t *rbuf, const char *channel,
					      const ptools_pointcloud2_t *msg, void *user_data);


		//-----static helpers
		private:
			static bool stringsEqual(string a, string b);


		//--static callbacks that will call the functions above, since not sure boost can convert to plain c function.
		private:
			static void cb_right_click_popup_menu(GtkWidget *menuItem, gpointer userdata);

			static void cb_on_param_widget_changed_xyzrgb (BotGtkParamWidget *pw, const char *name, void *user); //menu item clicked / adjusted
			static int cb_mouse_press(BotViewer *viewer, BotEventHandler *ehandler,
								const double ray_start[3], const double ray_dir[3], const GdkEventButton *event);
			static int cb_mouse_release (BotViewer *viewer, BotEventHandler *ehandler,
									 const double ray_start[3], const double ray_dir[3], const GdkEventButton *event);
			static int cb_mouse_motion(BotViewer *viewer, BotEventHandler *ehandler,
									 const double ray_start[3], const double ray_dir[3], const GdkEventMotion *event);

			static int cb_key_press (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey *event);
			static int cb_key_release (BotViewer *viewer, BotEventHandler *ehandler, const GdkEventKey  *event);

			static void cb_on_kinect_frame (const lcm_recv_buf_t *rbuf, const char *channel,
							const ptools_pointcloud2_t *msg, void *user_data);
	}; //class UIProcessing
} //namespace surrogate_gui

//====================================
#endif /* UIPROCESSING_H_ */
