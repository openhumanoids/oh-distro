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

//=========

/**sets the state of the object we want to draw and then draws*/
void OpenGL_Manipulator::draw()
{
  vector<KDL::Frame> collisionPts;
  _manipulator->getCollisionContactPoints(collisionPts);

  if (collisionPts.size() == 0)
    return;

  OpenGL_Object_Sphere s;
  for (uint i = 0; i < collisionPts.size(); i++)
    {
      s.set(collisionPts[i], 0.05); //todo : pick a radius
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
//------observers
ManipulatorStateConstPtr OpenGL_Manipulator::getManipulator() const
{
  return _manipulator;
}

