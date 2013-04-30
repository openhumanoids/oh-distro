/*
 * surrogate_renderer.cpp
 *
 *  Created on: Dec 18, 2011
 *      Author: mfleder
 */

#include "SurrogateRenderer.h"
#include "SurrogateException.h"
#include "LinearAlgebra.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <PclSurrogateUtils.h> //todo: remove just debugging 3d line fit of trace right now

#include <math.h> //for M_PI

using namespace std;
using namespace pcl;



namespace surrogate_gui
{
	//==============constructor
	/**@param viewer
	 * @param pw : menu/sidebar widget*/
	SurrogateRenderer::SurrogateRenderer(BotViewer* viewer, BotGtkParamWidget *pw)
		: _viewer(viewer), _pw(pw)
	{

		// Setup renderer
		BotRenderer *renderer = &_renderer;

		//mfleder: doesn't seem like C++ member function can be bound to
		//static, C void function
		//draw  callback
		//boost::function<void(BotViewer *, BotRenderer *)>
		//  drawCb(boost::bind(&SurrogateRenderer::draw_xyzrgb, this, _1, _2));
		//renderer->draw 		= drawCb.target<void(BotViewer*, BotRenderer*)>();

		//destroy callback
		//boost::function<void(BotRenderer *)>
		//      destructCb(boost::bind(&SurrogateRenderer::destruct, this, _1));
		//renderer->destroy 	= destructCb.target<void(BotRenderer*)>();


		renderer->draw 		= cb_draw_xyzrgb;
		renderer->destroy 	= cb_destruct;


		renderer->name 		= (char*)"Segmentation";
		renderer->widget 	= GTK_WIDGET(pw);
		renderer->enabled 	= 1;
		renderer->user 		= this;
		bot_viewer_add_renderer(viewer, renderer, 0); //0 = priority

		initVals();
	}




	SurrogateRenderer::~SurrogateRenderer()
	{
	}

	void SurrogateRenderer::cb_destruct(BotRenderer *renderer)
	{
		((SurrogateRenderer*) renderer->user)->destruct(renderer);
	}

	void SurrogateRenderer::destruct(BotRenderer *renderer)
	{
		//todo: anything?
	}

	void SurrogateRenderer::initVals()
	{
		// LCM msg
		_display_info.cloud 			= PointCloud<PointXYZRGB>::Ptr();
		_display_info.highlightColors.clear();
		_display_info.highlightColors.reserve(640*480);
		_display_info.displayLcmCloud 	= true;

		//--background color
		_viewer->backgroundColor[0] =  54/255.0; /*145.0*/ //105.0 /255.0;
		_viewer->backgroundColor[1] =  4/255.0; /*163.0*/ //105.0/255.0;
		_viewer->backgroundColor[2] =  89/255.0; /*176.0*/ //105.0/255.0;


		//rectangle
		_display_info.rectangle2D.shouldDraw = false;

		//self->displayText = strdup(std::string("").c_str());
		// Segmented objects
		getSegInfo()->numObjects 	= 0;
		getSegInfo()->objectPtsMap = boost::shared_ptr<ObjectNamePointsMap>(new map<string, ObjectPointsPtr>);
		getSegInfo()->color_segments = true;

		//---model fitting
		getModelInfo()->displayOn = true;
		getModelInfo()->drawCircles = false;
		getModelInfo()->drawLines = false;
		getModelInfo()->circles.clear();
		getModelInfo()->lines.clear();
		getModelInfo()->haveModel = false;
		Circle3D empty;
		getModelInfo()->rotationAxisGuess = empty;

		// Display options
		getTrackInfo()->displayTrackingCloud = false;

		// Tracked object transformation
		getTrackInfo()->trackedObjectTransform.setIdentity();
		getTrackInfo()->trackedObject =  pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
		getTrackInfo()->centroidTrace.clear();

		//grasp/manipulation info
		getGraspInfo()->forceVector.reset();
		getGraspInfo()->rotationAxis.reset();
		getGraspInfo()->forceVector.displaySelected(true);
		getGraspInfo()->rotationAxis.displaySelected(false);
		getGraspInfo()->displayTrajectory = false;

		//display text
		_mode_text.text 	= "Segmenting";
		_warning_text.text 	= "";
		_hint_text.text 	= "";

		_mode_text.color.red 		= 0;
		_mode_text.color.green 		= 255;
		_mode_text.color.blue 		= 0;
		_mode_text.color.alpha 		= 0;

		_warning_text.color.red 		= 255;
		_warning_text.color.green 		= 0;
		_warning_text.color.blue 		= 0;
		_warning_text.color.alpha 		= 0;

		_hint_text.color.red 		= 0;
		_hint_text.color.green 		= 0;
		_hint_text.color.blue 		= 255;
		_hint_text.color.alpha 		= 0;


		setCamera(CAMERA_FRONT);

		//===========set initial msg to a cube
		PointCloud<PointXYZRGB>::Ptr cube_msg(new PointCloud<PointXYZRGB>);
		cube_msg->width = 5*5;
		cube_msg->height = 5;
		cube_msg->points.resize(cube_msg->width * cube_msg->height);
		int npt = 0;
		for (double x = 0; x <= 4; x++)
		{
			for (double y = 0; y <= 4; y++)
			{
				for (double z = 0; z <= 4; z++)
				{
					uint8_t r = 0, g = 0, b = 0;
					if (x <= 3)
						r = 128;
					if (y <= 3)
					   g = 128;
					if (z <= 3)
					   b = 128;

					cube_msg->points[npt].x = x*10;
					cube_msg->points[npt].y = -y*10;
					cube_msg->points[npt].z = z*10;
					cube_msg->points[npt].r = r;
					cube_msg->points[npt].g = g;
					cube_msg->points[npt].b = b;

					npt++;
				}
			}
		}
		_display_info.cloud = cube_msg;

	}


	//====================

	//==================================================
	//==================================================
	//==================================================
	//==================================================
	//==================================================
	//==================================================
	//==================================================
	//=========================drawing

	void SurrogateRenderer::drawText()
	{
		int width 	= GTK_WIDGET(_viewer->gl_area)->allocation.width;
		int height 	= GTK_WIDGET(_viewer->gl_area)->allocation.height;

		// Setup gl coordinates for drawing
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, width, 0, height, -1, 1);
		glDisable(GL_DEPTH_TEST);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		//glEnable(GL_BLEND);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		//text: note: if using lighting, should turn it off before drawing text
		//note: need to set the color before setting the position

		//mode
		glColor4ub(_mode_text.color.red, _mode_text.color.green, _mode_text.color.blue, _mode_text.color.alpha);
		glRasterPos2i(5, height - 20); //0,0
		glutBitmapString(GLUT_BITMAP_HELVETICA_18, (unsigned char *) _mode_text.text.c_str());

		//warning
		glColor4ub(_warning_text.color.red, _warning_text.color.green, _warning_text.color.blue, _warning_text.color.alpha);
		glRasterPos2i(5, 5); //0,0
		glutBitmapString(GLUT_BITMAP_HELVETICA_18, (unsigned char *) _warning_text.text.c_str());

		//hint
		glColor4ub(_hint_text.color.red, _hint_text.color.green, _hint_text.color.blue, _hint_text.color.alpha);
		glRasterPos2i(width - 18*_hint_text.text.length(), height - 20); //0,0
		glutBitmapString(GLUT_BITMAP_HELVETICA_18, (unsigned char *) _hint_text.text.c_str());


		//glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST); //mf

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}

	// Draw the kinect scene and paint segmentation data
	void SurrogateRenderer::cb_draw_xyzrgb(BotViewer *viewer, BotRenderer *renderer)
	{
		SurrogateRenderer *self = (SurrogateRenderer*) renderer->user;
		self->draw_xyzrgb(viewer, renderer);
	}

	void SurrogateRenderer::draw_xyzrgb(BotViewer *viewer, BotRenderer *renderer)
	{

		//-------draw the axes
		bot_gl_draw_axes();
		//--------

		// Return if no scene to draw
		if (!_display_info.cloud)
		{
			drawText();
			return;
		}

		// get the cloud message
		PointCloud<PointXYZRGB>::ConstPtr msg = _display_info.cloud;

		//clear the color map
		for (uint i = 0; i < _display_info.cloud->points.size(); i++)
			_display_info.highlightColors[i] = Color::NULL_COLOR;

		// Generate map of points that should be highlighted, and in what color
		setPointsToHighlight();

		//mfleder: remove this if issues w/ selecting the wrong points (and remove the corresponding
		doGlRotateStuff();

		// draw the points
		glEnable(GL_DEPTH_TEST);
		glPointSize(2.0f);


		glColor3f(0, 0, 0);
		const uint8_t ALPHA_CLOUD_UB = 40;
		if (shouldDrawTrackingCloud()) //only blend if going to draw tracking info
		{
			glEnable(GL_BLEND); //for dimming contrast
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		}
		glBegin(GL_POINTS);

		float minZ=1e10;
		float maxZ=-1e10;

		for (uint i = 0; i < msg->size(); i++)
	        {
		  minZ = min(minZ,msg->points[i].z);
		  maxZ = max(maxZ,msg->points[i].z);
		}


		//iterate through all the points
		for (uint i = 0; i < msg->size(); i++)
		{
		         float heightRatio = (msg->points[i].z-minZ)/(maxZ-minZ);
			 float *outColor = bot_color_util_jet(heightRatio);

			//extract (original) color
			RGB_PCL pclColor;
			pclColor.float_value = msg->points[i].rgb;

			if (_display_info.highlightColors[i] == Color::NULL_COLOR)
			{
				//blend w/ background color.  see OpenGL superbible page 231
				glColor4f(_viewer->backgroundColor[0], _viewer->backgroundColor[1], _viewer->backgroundColor[2], 255);
				//glColor4ub(pclColor.red, pclColor.green, pclColor.blue, ALPHA_CLOUD_UB); //use original point cloud color
				glColor4f(outColor[0], outColor[1], outColor[2], ALPHA_CLOUD_UB/255.0f); 
			}
			else if (isPaused()) //might be highlighting a selection
			{
				RGB_PCL rgb;
				Color::getColor(_display_info.highlightColors[i], rgb);
				glColor3f(rgb.red, rgb.green, rgb.blue);
			}

			//only draw a vertex if either (a) drawing everything or
			//(b) we have segments to highlight
			if (_display_info.displayLcmCloud || _display_info.highlightColors[i] != Color::NULL_COLOR)
				glVertex3f(msg->points[i].x, msg->points[i].y, msg->points[i].z);
		}
		glEnd(); //-----GL_POINTS

		if (shouldDrawTrackingCloud())
			glDisable(GL_BLEND); //was turned on above in this case

		if (shouldDrawTrackingCloud())
		{
			drawTrackingInfo();
			drawGraspInfo();
		}

		drawText();
		drawModelFitting();

		//draw rectangle
		if (_display_info.rectangle2D.shouldDraw)
			drawRectangle(); //put this at the end since blending happens here
	}

	/**@return true if (a) have a tracked object to display
	 * 	            && (b) displayTrackingCloud is set to true*/
	bool SurrogateRenderer::shouldDrawTrackingCloud()
	{
		return 		(getTrackInfo()->trackedObject != PointCloud<PointXYZRGB>::Ptr())
				&&	(getTrackInfo()->displayTrackingCloud);
	}

	void SurrogateRenderer::drawGraspInfo()
	{
		//display the force vector and rotation axis
		AdjustableVector *fvec = &getGraspInfo()->forceVector;
		AdjustableVector *rvec = &getGraspInfo()->rotationAxis;
		if (fvec->valuesInitialized())
			fvec->draw();
		if (rvec->valuesInitialized())
			rvec->draw();

		//display the trajectory determined by these vectors?
		if (!getGraspInfo()->displayTrajectory)
			return;

		//won't generate trajectory until these are initialized
		if (!fvec->valuesInitialized() ||
			!rvec->valuesInitialized())
			return;

		try
		{
			generateTrajectory(); //todo: draw separtely
		}
		catch(...)
		{
			setWarningText("Please adjust force/rotation vectors before generating trajectory");
			return;
		}
	}

	void SurrogateRenderer::generateTrajectory()
	{
		AdjustableVector *fvec = &getGraspInfo()->forceVector;
		AdjustableVector *rvec = &getGraspInfo()->rotationAxis;

		//get circle center
		Circle3D traj;
		PointXYZ graspPt, misc;  //misc not used
		rvec->getVectorStartEndNonUnit(traj.center, misc);

		//get grasp pt
		fvec->getVectorStartEndNonUnit(graspPt, misc);

		//radius = ||center - graspPt||
		traj.radius = LinearAlgebra::length(LinearAlgebra::sub(traj.center, graspPt));

		//fvec is in the plane of the circle.  We want to determine
		//an orthonormal basis for this plane = {fvec.dir, fvec.dir x rvec.dir}
		traj.planeVec0 = LinearAlgebra::normalize(fvec->getVectorUnitDir());
		traj.planeVec1  = LinearAlgebra::crossProduct(LinearAlgebra::normalize(rvec->getVectorUnitDir()),
													  traj.planeVec0);

		traj.planeVec1 = LinearAlgebra::normalize(traj.planeVec1);
		drawCircle(traj);
	}

	/**Draws tracking centroid and point cloud.
	 * Todo: mmove the tracked cloud into the loop in drawXYZRGB above*/
	void SurrogateRenderer::drawTrackingInfo()
	{
		//do we have a tracked object cloud?
		if (!shouldDrawTrackingCloud())
			throw SurrogateException("Shouldn't draw tracking info");

		//display the centroid
		glPointSize(20.0f);
		glColor3ub(0,0,255);
		glBegin(GL_POINTS);
		vertex3f(getTrackInfo()->centroidTrace.back());
		glEnd(); //GL_POINTS

		//display the cloud
		glEnable(GL_DEPTH_TEST);
		glPointSize(5.0f);
		glEnable(GL_BLEND); //blend w/ white to increase brightness
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //superbible page 231
		const uint8_t ALPHA_CLOUD_UB = 255; //blending with white
		glBegin(GL_POINTS);

		for (uint i = 0; getTrackInfo()->trackedObject && i < getTrackInfo()->trackedObject->points.size(); i++)
		{
			PointXYZRGB next = getTrackInfo()->trackedObject->points[i];

			//-------normals
			RGB_PCL pclColor;
			pclColor.float_value = next.rgb;
			glColor4ub(255,255,255, 255); //white
			glColor4ub(pclColor.red, pclColor.green, pclColor.blue, ALPHA_CLOUD_UB); //use original point cloud color

			/*PointXYZ start, end; //start should be the centroid
			getTranslatedForceVector(start, end); //vector end point
			PointXYZ fVector(end.x - start.x,
							 end.y - start.y,
							 end.z - start.z);
			double length = euclideanDistance(start, end);

			glNormal3f(fVector.x/length, fVector.y/length, fVector.z/length);
			*/
			//----end normals

			//glColor3ub(0, 255, 0);
			glVertex3f(next.x, next.y, next.z);
		}
		glEnd(); //GL_POINTS
		glDisable(GL_BLEND);

		//------draw centroid trace
		if (!getTrackInfo()->trackedObject)
			return;

		list<PointXYZ> &centroidTrace = getTrackInfo()->centroidTrace;

		glPointSize(7.0f);
		glBegin(GL_POINTS);
		glColor3ub(0,255,0);
		for (list<PointXYZ>::iterator it = centroidTrace.begin();
			it != centroidTrace.end(); it++)
		{
			PointXYZ nextPt = *it;
			glVertex3f(nextPt.x, nextPt.y, nextPt.z);
		}
		glEnd(); //GL_POINTS

		drawRevolutePrismaticFitting();
	}

	/**Draws either a line or a circle -- depending on the shape fit
	 * of the centroid trace*/
	void SurrogateRenderer::drawRevolutePrismaticFitting()
	{
		const list<PointXYZ> &centroidTrace = getTrackInfo()->centroidTrace;
		//===============model fitting
		if (!getTrackInfo()->trackedObject || centroidTrace.size() < 4)
			return;

		PclSurrogateUtils::SHAPE_FIT_RESULTS fitResults;
		PclSurrogateUtils::classifyShape(centroidTrace, fitResults);
		if (fitResults.shapeType == PclSurrogateUtils::UNKNOWN)
		{
			cout << "\nCouldn't classify shape" << endl;
			return;
		}

		//good linear fit?
		if (fitResults.shapeType == PclSurrogateUtils::LINE_3D)
		{
			setHintText("Good Linear Fit");
			glLineWidth(10.0);
			glBegin(GL_LINES);
			glColor3ub(255,0,0); //red
			vertex3f(fitResults.lineSeg.endA);
			vertex3f(fitResults.lineSeg.endB);
			glEnd(); //GL_Line
		}
		else if (fitResults.shapeType == PclSurrogateUtils::ARC_3D) //circle fit
		{
			setHintText("Arc Fit");
			glPointSize(10.0);
			glColor3ub(255,0,0); //red
			drawCircle(fitResults.circle);
		}
	}

	/**draw the circles and lines in the model fitting struct*/
	void SurrogateRenderer::drawModelFitting()
	{
		//first and last item should be redundant
		if (!getModelInfo()->drawCircles || !getModelInfo()->displayOn || !getModelInfo()->haveModel)
			return;
		if (getModelInfo()->drawLines)
			throw SurrogateException("Drawing model lines not implemented");

		vector<Circle3D> &circles = getModelInfo()->circles;

		if (circles.size() == 0)
			return; //nothing to do

		//get current object so we can look for distinct colors
		ObjectPointsPtr currObj = getCurrentObjectSelected();
		if (currObj->getSegmentBeingDisplayed()->size() == 0)
			return; //no model to display if no relevant object

		vector<Color::StockColor> colors = Color::getDifferentColors(currObj->getColorBeingDisplayed(), circles.size());

		for (uint i = 0; i < circles.size(); i++)
		{
			glPointSize(10.0);
			color3f(colors[i]);
			drawCircle(circles[i]);
		}
	}

	//draws a rectangle
	//see: http://processing.org/discourse/yabb2/YaBB.pl?board=OpenGL;action=display;num=1221122226
	void SurrogateRenderer::drawRectangle()
	{
		if (!_display_info.rectangle2D.shouldDraw)
			return;

		//---color
		RGB_PCL rgb;
		Color::getColor(Color::HIGHLIGHT, rgb);


		int width 	= GTK_WIDGET(_viewer->gl_area)->allocation.width;
		int height 	= GTK_WIDGET(_viewer->gl_area)->allocation.height;

		// Setup gl coordinates for drawing
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glOrtho(0, width, 0, height, -1, 1);
		glDisable(GL_DEPTH_TEST);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		const Rectangle2D& rect = _display_info.rectangle2D;
		glBegin(GL_QUADS);
		//glColor4f(.7, .7, 0, .2);
		glColor4f(rgb.red, rgb.green, rgb.blue, 0.5);
		glVertex2f(rect.x0, height - rect.y0);
		glVertex2f(rect.x1, height - rect.y0);
		glVertex2f(rect.x1, height - rect.y1);
		glVertex2f(rect.x0, height - rect.y1);
		glEnd(); //GL_QUADS
		glDisable(GL_BLEND);

		//glColor3f(.7, .7, 0);
		color3f(Color::HIGHLIGHT);
		glBegin(GL_LINE_LOOP);
		glVertex2f(rect.x0, height - rect.y0);
		glVertex2f(rect.x1, height - rect.y0);
		glVertex2f(rect.x1, height - rect.y1);
		glVertex2f(rect.x0, height - rect.y1);
		glEnd();


		glEnable(GL_DEPTH_TEST); //mf

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		return;
	}

	/**Determine point indices that should be highlighted in the point cloud
	 *@modifies _display_info.highlightColors: (point index)--> color  */
	void SurrogateRenderer::setPointsToHighlight()
	{
		if (_display_info.cloud->points.size() == 0 || !getSegInfo()->color_segments)
			return; //no indices to highlight or not coloring segments

		vector<Color::StockColor> &colorMap = _display_info.highlightColors; //maps from: index of point --> color

		//if cloud size changes, we might need to re-implement this (or just resize)
		if (_display_info.highlightColors.size() != _display_info.cloud->points.size()
				&& _display_info.cloud->points.size() != 0 && colorMap.size() != 0)
			throw SurrogateException("\nsetPointsToHightlight: cloud size changed.");

		//determine rectangle points to highlight

		/*
		if (_display_info.rectangle2D.shouldDraw)
		{
			Rectangle2D *rect = &(_display_info.rectangle2D);
			set<int> rect_highlight = mouseRectTo3D(rect->x0, rect->y0, rect->x1, rect->y1);
			set<int>::iterator it;
			for (it = rect_highlight.begin();
					it != rect_highlight.end();
					it++)
			{
				colorMap[*it] = Color::HIGHLIGHT;
			}
		}
		 */

		// highlight already segmented object points
		map<string, ObjectPointsPtr>::iterator objIter;
		for (objIter =  getSegInfo()->objectPtsMap->begin();
			 objIter != getSegInfo()->objectPtsMap->end();
			 objIter++)
		{
			ObjectPointsPtr nextObject = (*objIter).second; //next segmented object w/ it's possibly auto-segmented sub-components

			//============go thru each of the auto-segmented sub-components
			int numAutoSegments = nextObject->auto_segment_indices.size();  //how many auto-segmented sub-components
			if (numAutoSegments > 0)
			{
				for(int i = 0; i < numAutoSegments; i++) //iterate over sub-components
				{
					 if (i != nextObject->getDisplaySelection()
							 && !nextObject->displayingCompletePointSet()) //only display the desired subcomponent?
						 continue;

					SetIntPtr nextAutoSegIndices = nextObject->auto_segment_indices[i];  //next auto-segmented sub-component
					for (set<int>::iterator nextSubIter = nextAutoSegIndices->begin();
							nextSubIter != nextAutoSegIndices->end();
							nextSubIter++)
					{
						if (colorMap[*nextSubIter] != Color::NULL_COLOR && colorMap[*nextSubIter] != Color::HIGHLIGHT)
							throw SurrogateException("how did this get colored already?");
						colorMap[*nextSubIter] = nextObject->getColorForAutoSegment(i);
					}
				}
			}


			//==========now color any points not auto-segmented
			for (set<int>::iterator pointIter = nextObject->indices->begin();
				pointIter != nextObject->indices->end()
						&& nextObject->displayingCompletePointSet();
					pointIter++)
			{
				if (colorMap[*pointIter] == Color::NULL_COLOR)
				  {
				    if (nextObject->color > Color::HIGHLIGHT)
				      throw SurrogateException("\nbanana: inserted bad color: " + Color::to_string(nextObject->color));
				    colorMap[*pointIter] = nextObject->color;
				}
			}
		}
	}

  	void SurrogateRenderer::doGlRotateStuff()
	{
	  // rotate so that X is forward and Z is up
	  //glRotatef(-90, 1, 0, 0);
	}
  
	void SurrogateRenderer::drawCircle(const Circle3D &circle)
	{
		glBegin(GL_POINTS);
		for (double theta = 0; theta < 2*3.1415; theta += 0.1)
		{
			PointXYZ nextPt = LinearAlgebra::getCirclePt(circle.planeVec0, circle.planeVec1,
														 circle.radius, theta, circle.center);
			vertex3f(nextPt);
		}

		//---draw rotation axis
		Eigen::Vector3f normal = PclSurrogateUtils::toVec(LinearAlgebra::crossProduct(circle.planeVec0, circle.planeVec1));
		normal += PclSurrogateUtils::toVec(circle.center);
		for(double t = -10; t < 10; t += 0.01)
		{
			Eigen::Vector3f nextPt = t*normal + (1-t)*PclSurrogateUtils::toVec(circle.center);
			glVertex3f(nextPt[0], nextPt[1], nextPt[2]);
		}
		glEnd(); //GL_POINTS
	}

	/**switches to the given color*/
	void SurrogateRenderer::color3f(const Color::StockColor &c)
	{
		RGB_PCL rgb;
		Color::getColor(c, rgb);
		glColor3f(rgb.red, rgb.green, rgb.blue);
	}

	/**@p vertex to draw*/
	void SurrogateRenderer::vertex3f(pcl::PointXYZ p)
	{
		if (isnan(p.x) || isnan(p.y) || isnan(p.z))
			return;
		glVertex3f(p.x, p.y, p.z);
	}

	/**@p vertex to draw with the given color*/
	void SurrogateRenderer::vertex3fRgb(pcl::PointXYZRGB p)
	{
		if (isnan(p.x) || isnan(p.y) || isnan(p.z))
			return;

		//extract color
		RGB_PCL pclColor;
		pclColor.float_value = p.rgb;

		glColor3ub(pclColor.red, pclColor.green, pclColor.blue);
		glVertex3f(p.x,p.y,p.z);
	}

	//=============================================
	//=============================================
	//=============================================
	//=============================================
	//=============================================
	//=============================================
	//=============================================
	//============================User Selection
	/**@return indices in renderer->msg (the point cloud) corresponding to the selected points */
	set<int> SurrogateRenderer::mouseRectTo3D(double x1, double y1, double x2, double y2)
	{

		set<int> selected_indices;  //to return
		if (!isPaused())
			return selected_indices;

		// Flip y coordinates for OpenGL
		y1 = GTK_WIDGET(_viewer->gl_area)->allocation.height - y1;
		y2 = GTK_WIDGET(_viewer->gl_area)->allocation.height - y2;

		double xmin = std::min(x1, x2);
		double xmax = std::max(x1, x2);
		double ymin = std::min(y1, y2);
		double ymax = std::max(y1, y2);

		//mfleder: if issues arise w/ selecting points, try removing this (push/pop and doGlRotateStuff)
		glPushMatrix();
		doGlRotateStuff();

		// Get opengl view information for projection
		GLdouble m[16];
		glGetDoublev(GL_MODELVIEW_MATRIX, m);
		GLdouble proj_mat[16];
		glGetDoublev(GL_PROJECTION_MATRIX, proj_mat);
		GLint view[4];
		glGetIntegerv(GL_VIEWPORT, view);

		GLdouble x0, y0, z0, xp, yp, zp;
		PointCloud<PointXYZRGB>::ConstPtr msg = _display_info.cloud;

		//cout << "\n mouseRectTo3D. msg size = \n" <<  msg->points.size() << endl;
		//cout << "(" << x1 << ", " << y1 << ") | (" << x2 << ", " << y2 << ")" << endl;

		for (uint i = 0; i < msg->points.size(); i++)
		{
			x0 = msg->points[i].x, y0 = msg->points[i].y, z0 = msg->points[i].z;
			if (isnan(x0) || isnan(y0) || isnan(z0))
			{
				uint8_t r = msg->points[i].r,
					   g =msg->points[i].g,
					   b = msg->points[i].b;

				if (isnan(r) || isnan(g) || isnan(b))
					continue;

				//if (r != 0 && g!=0 && b!= 0 && (r != 254 && g != 254 && b != 254))
				//	printf("\n\n\n****nan w/ rgb = (%d, %d, %d)     **********8\n\n\n", r,g,b);
					//cout << "\n\n\n****nan w/ rgb = (" << r << ", " << g << ", " << b << ")" << "\n\n\n\n" << endl;
				continue;
			}

			gluProject(x0,y0,z0,m,proj_mat,view,&xp,&yp,&zp); //project into 2D viewing plane

			if (xp < xmin || xp > xmax || yp < ymin || yp > ymax || zp > 1.0) //see if w/i rectangle selection
				continue;

			selected_indices.insert(i);
		}

		//mfleder remove
		glPopMatrix();

		return selected_indices;
	}

	//==================================
	//==================================
	//==================================
	//==================================
	//===============================camera

	// Updates GL matrices with new camera position
	void SurrogateRenderer::setCamera (CameraView camera_view)
	{
		BotViewHandler *vh = _viewer->view_handler;

		double lookat[3] = {0,0,0};
		double up[3] = {0,0,0};
		double eye[3] = {0,0,0};
		double away_coord = 2.5;

		/*
		switch (camera_view) {
		case CAMERA_FRONT:
					printf("CHANGING VIEW: FRONT\n");
			eye[0] = -1.0*away_coord;
			up[2] = 1;
			vh->set_look_at(vh, eye, lookat, up);
			break;
		case CAMERA_SIDE:
					printf("CHANGING VIEW: SIDE\n");
			eye[1] = away_coord;
			up[2] = 1;
			vh->set_look_at(vh, eye, lookat, up);
			break;
		case CAMERA_TOP:
					printf("CHANGING VIEW: TOP\n");
			eye[2] = away_coord;
			up[0] = 1;
			vh->set_look_at(vh, eye, lookat, up);
			break;
		default:
			break;
			}*/
	}


	//displayed on bottom of screen
	void SurrogateRenderer::setWarningText(string warning)
	{
		_warning_text.text = warning;
		cout << endl << "WARNING: " << warning << endl;

	}

	void SurrogateRenderer::setModeText(string modeText)
	{
		if (_mode_text.text != modeText)
			cout << endl << "\n ====MODE TEXT: " << modeText << "====" << endl;

		_mode_text.text = modeText;
	}

	void SurrogateRenderer::setHintText(string hintText)
	{
		if (_hint_text.text != hintText)
			cout << endl << "\n ====HINT TEXT: " << hintText << "====" << endl;

		_hint_text.text = hintText;
	}

	//==========
	UserSegmentation * SurrogateRenderer::getSegInfo()
	{
		return &(_display_info.user_seg_info);
	}

	ModelFit *SurrogateRenderer::getModelInfo()
	{
		return &(_display_info.model_fit);
	}

	TrackingInfo * SurrogateRenderer::getTrackInfo()
	{
		return &(_display_info.tracking_info);
	}

	GraspInfo *SurrogateRenderer::getGraspInfo()
	{
		return &(_display_info.graspInfo);
	}

	bool SurrogateRenderer::isPaused() const
	{
    return true; //TODO fix or remove altogether		
    //return bot_gtk_param_widget_get_bool(_pw, PARAM_NAME_CLOUD_PAUSE) == 1;
	}

	string SurrogateRenderer::getCurrentObjectSelectedName() const
	{
    return "Object 1"; //TODO fix or remove altogether
		//return bot_gtk_param_widget_get_enum_str(_pw, PARAM_NAME_CURR_OBJECT);
	}

	/**@returns the object currently selected in the drop-down menu.  Or
	 * null if no object selected*/
	ObjectPointsPtr SurrogateRenderer::getCurrentObjectSelected()
	{
		string currentObject = getCurrentObjectSelectedName();
		if (strcmp(currentObject.c_str(), "None") == 0) //no object selected?
			return boost::shared_ptr<ObjectPoints>();
		return getSegInfo()->objectPtsMap->at(currentObject);
	}

	//----------

} //namespace surrogate_gui
