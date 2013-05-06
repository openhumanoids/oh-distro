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
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/ptools_pointcloud2_t.h>
#include <lcmtypes/ptools_pointfield_t.h>

#include <bot_vis/bot_vis.h>
#include <bot_vis/glm.h>
#include <bot_frames/bot_frames.h>
//#include <bot_param/param_client.h>
//#include <bot_param/param_util.h>
//#include "param_widget.h"

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
#include "Segmentation.h"

#include <lcmtypes/drc_lcmtypes.hpp>

//-------pulling maps
#include <maps/ViewClient.hpp>
#include <boost/thread.hpp>

#include <bot_lcmgl_client/lcmgl.h>

namespace affordance {
  class AffordanceUpWrapper;
}

namespace surrogate_gui
{

	//===========state enumeration
	enum GuiState {SEGMENTING, TRACKING};

	typedef struct
	{
		bool shift_L_is_down;
		bool shift_R_is_down;
		bool ctrl_L_is_down;
		bool ctrl_R_is_down;
	} ButtonStates;

	//================class UI_Processing

	/**Class for processing user input: keystrokes, menu selections, mouse button clicks*/
	class UIProcessing
	{

    typedef boost::shared_ptr<drc::affordance_plus_t> AffPlusPtr;  

		//======fields
		private:
			GuiState _gui_state; //segmenting or tracking
			SurrogateRenderer _surrogate_renderer; //graphics, display
			boost::shared_ptr<lcm::LCM> _lcmCpp; //cpp version of _lcm

			ButtonStates _button_states;  //for monitoring which keys are being held down

			BotEventHandler *_ehandler;
			BotEventHandler *_mode_handler;

			boost::shared_ptr<ObjectTracker> _objTracker;
			
			boost::shared_ptr<maps::ViewClient> _mViewClient; //map pulling
          boost::shared_ptr<affordance::AffordanceUpWrapper> _affordance_wrapper;  // affordance store interface

      bot_lcmgl_t* _lcmgl;

      // affordance listener and selector
      std::vector<drc::affordance_plus_t> _currentAffordances;
      std::vector<std::string> _currentAffordanceNames;
      boost::mutex _currentAffordancesMutex;
      std::string _selectedAffordanceName;
      void affordanceMsgHandler(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel, 
                const drc::affordance_plus_collection_t* collection);
      void handleAffordanceSelectChange(BotGtkParamWidget *pw);
      AffPlusPtr getSelectedAffordance();
      volatile bool _updatingAffordanceSelectionMenu;

		//=====constructor/destructor
		public:
			UIProcessing(BotViewer *viewer, boost::shared_ptr<lcm::LCM> lcmCpp, 
				     std::string kinect_channel);
			~UIProcessing();

		//=====observers
		public:
			MouseMode getMouseMode() const; //rectangle or camera move
			GeometricPrimitive getGeometricPrimitive() const; //rectangle or camera move
			std::string getCurrentObjectSelectedName() const;
			ObjectPointsPtr getCurrentObjectSelected(); //object currently being segmented
			SurrogateDisplayInfo *getDisplayInfo();
			TrackMode getTrackMode() const;
			int8_t getManipulationType() const;
		//=====mutators
		private:

			//segmentation
			void addIndicesToCurrentObject(std::set<int> &selected_indices);
			void removeIndicesFromCurrentObject(std::set<int> &selected_indices);
			void intersectViews(std::set<int> &selected_indices);
			void suggestSegments();

			void handleMouseModeAdjust(BotEventHandler *default_handler, BotGtkParamWidget *pw); //user clicks on MouseMode drop-down menu
			void handleTrackLiveButton(BotGtkParamWidget *pw);
			void handleSaveCloudButton(BotGtkParamWidget *pw);
			void handleAffordancePubButton(BotGtkParamWidget *pw);
			void handleAffordancePushButton(BotGtkParamWidget *pw);
			void handleAffordancePubButtonCylinder(const Segmentation::FittingParams& fp);
			void handleAffordancePubButtonSphere(const Segmentation::FittingParams& fp);
			void handleAffordancePubButtonCircle3d(const Segmentation::FittingParams& fp);
			void handleAffordancePubButtonPlane(const Segmentation::FittingParams& fp);
			void handleAffordancePubButtonLine(const Segmentation::FittingParams& fp);
			void handleAffordancePubButtonTorus(const Segmentation::FittingParams& fp);
			void handleAffordancePubButtonCube(const Segmentation::FittingParams& fp);
                        void handleAffordancePubButtonPointCloud(const Segmentation::FittingParams& fp);
			void handleFullResetButton(BotGtkParamWidget *pw);

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
