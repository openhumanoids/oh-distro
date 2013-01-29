/*
 * OpenGLManipulator.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: mfleder
 */

#include "OpenGL_Manipulator.h"
#include "opengl/opengl_object_sphere.h"

using namespace opengl;
using namespace affordance;
using namespace std;
using namespace boost;
using namespace Eigen;
//--------------constructor/destructor
OpenGL_Manipulator::OpenGL_Manipulator(ManipulatorStateConstPtr manipulator, 
				       bool isHighlighted,
				       Eigen::Vector3f highlightColor)
  : OpenGL_Object(manipulator->getGUIDAsString(), isHighlighted, highlightColor),
    _manipulator(manipulator)
{
}

OpenGL_Manipulator::~OpenGL_Manipulator()
{
}

/**sets the state of the object we want to draw and then draws*/
void OpenGL_Manipulator::draw()
{
  //------------drawing the 
  CollisionGroupPtr colgroup = _manipulator->getLink()->getCollisions("default");
  if (colgroup != NULL && colgroup->size() > 0) 
    {
      // add collision contact widgets!
      OpenGL_Object_Sphere s;
      for (uint i = 0; i < colgroup->size(); i++) 
	{
	  urdf::Pose dot = (*colgroup)[i]->origin;
	  KDL::Frame f;
	  f.p = KDL::Vector(dot.position.x, dot.position.y, dot.position.y);
	  s.set(f, 0.05); //todo : pick a radius
	  
	  if (_isHighlighted) 
	    {
	      s.draw(_highlightColor);
	    }
	  else
	    {
	      s.set_color(_manipulator->getColor());
	      s.draw();
	    }
	}
    } 
}
//------observers
ManipulatorStateConstPtr OpenGL_Manipulator::getManipulator() const
{
  return _manipulator;
}

