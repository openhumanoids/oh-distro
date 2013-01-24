/*
 * OpenGLManipulator.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: mfleder
 */

#include "OpenGL_Manipulator.h"

using namespace affordance;
//using namespace opengl;
using namespace std;
using namespace boost;
using namespace Eigen;
//--------------constructor/destructor
OpenGL_Manipulator::OpenGL_Manipulator(ManipulatorStateConstPtr manipulator, 
				       bool isHighlighted,
				       Eigen::Vector3f highlightColor)
  : OpenGL_Object(manipulator->getName(), isHighlighted, highlightColor),
    _manipulator(manipulator)
{
}

OpenGL_Manipulator::~OpenGL_Manipulator()
{
}

/**sets the state of the object we want to draw and then draws*/
void OpenGL_Manipulator::draw()
{
  //frame will be used by everything -- only compute once
  KDL::Frame frame = _manipulator->getFrame();
  OpenGL_Object *obj;

  throw NotImplementedException("todo: draw opengl manipulator");

  if (_isHighlighted) 
  {
      obj->draw(_highlightColor);
  }
  else
  {
      obj->set_color(_manipulator->getColor());
      obj->draw();
  }
}


//------observers
ManipulatorStateConstPtr OpenGL_Manipulator::getManipulator() const
{
  return _manipulator;
}

