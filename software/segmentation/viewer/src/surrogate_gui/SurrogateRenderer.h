/*
 * surrogate_renderer.h
 *
 *  Created on: Dec 18, 2011
 *      Author: mfleder
 */

#ifndef SURROGATE_RENDERER_H_
#define SURROGATE_RENDERER_H_
//===========================


#include "DisplayInfo.h"
#include "menu_definitions.h"
#include "display_objects/AdjustableVector.h"
#include <bot_vis/viewer.h>
#include <bot_vis/bot_vis.h>
#include <bot_vis/param_widget.h>

#include <set>
#include <map>

namespace surrogate_gui
{

	class SurrogateRenderer
	{
		friend class UIProcessing;

		//-----------------fields
		private:
			//libbot stuff
			BotRenderer _renderer;
			BotViewer   *_viewer;
			BotGtkParamWidget *_pw; //menu and buttons stuff

			//gui state and display
			SurrogateDisplayInfo _display_info;

			TextDisplayInfo _mode_text; //state, user instructions
			TextDisplayInfo _warning_text; //warning/error messages to display
			TextDisplayInfo _hint_text;

		//--------------constructor/destructor
		public:
			SurrogateRenderer(BotViewer* viewer, BotGtkParamWidget *pw);
			virtual ~SurrogateRenderer();
			static void cb_destruct(BotRenderer *renderer);
			void destruct(BotRenderer *renderer);
			void initVals();

		//-----------observers
		std::set<int> mouseRectTo3D(double x1, double x2, double y1, double y2); //can't declare constant b/c temporarily does some glrotate stuff
		UserSegmentation *getSegInfo();
		ModelFit *getModelInfo();
		TrackingInfo *getTrackInfo();
		GraspInfo *getGraspInfo();
		bool isPaused() const; //display/cloud updates paused?

		std::string getCurrentObjectSelectedName() const;
		ObjectPointsPtr getCurrentObjectSelected(); //object currently being segmented

		//-----------mutators
		void setCamera (CameraView camView);
		void setModeText(std::string text);
		void setHintText(std::string text);
		void setWarningText(std::string text);
		void drawText();
		void drawForceVector();
		static void cb_draw_xyzrgb(BotViewer *viewer, BotRenderer *renderer); //static callback, will call non-static cb
		void draw_xyzrgb(BotViewer *viewer, BotRenderer *renderer);
		void drawRectangle();
		bool shouldDrawTrackingCloud();
		void drawGraspInfo();
		void drawTrackingInfo();
		void drawRevolutePrismaticFitting();
		void drawModelFitting();
		void setPointsToHighlight();
		void doGlRotateStuff();

		void generateTrajectory();//from force and rotation vectors
		static void drawCircle(const Circle3D &circle);
		static inline void vertex3f(pcl::PointXYZ p);
		static inline void vertex3fRgb(pcl::PointXYZRGB p);
		static inline void color3f(const Color::StockColor &c);

	}; //class SurrogateRenderer

} //namespace surrogate_gui

//==================================
#endif /* SURROGATE_RENDERER_H_ */
