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
	  double q1, q2, q3, q4;
	  dot.rotation.getQuaternion(q1, q2, q3, q4);
	  KDL::Frame col_in_link(KDL::Rotation::Quaternion(q1, q2, q3, q4), 
				 KDL::Vector(dot.position.x, dot.position.y, dot.position.y));
	  KDL::Frame link_in_world(_manipulator->getLinkFrame());
	  KDL::Frame f(col_in_link * link_in_world);

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

