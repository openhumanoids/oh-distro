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

//=====
string toStr(const KDL::Vector &v)
{
  return string("(") + ModelState::toStr(v.x()) + string(", ") 
    + ModelState::toStr(v.y()) + string(", ") 
    + ModelState::toStr(v.z()) + string(")");
}


string toStr(const KDL::Rotation &r)
{
  double q1,q2,q3,q4;
  r.GetQuaternion(q1,q2,q3,q4);
  return string("(") + ModelState::toStr(q1) + string(", ") 
    + ModelState::toStr(q2) + string(", ") 
    + ModelState::toStr(q3)
    + string(", ")
    + ModelState::toStr(q4) + string(")");
}


string toStr(const KDL::Frame &frame)
{
  return string("Position = ") + toStr(frame.p)
    + string("\t Rotation = ") + toStr(frame.M);
}


std::ostream& operator << (std::ostream &out, const KDL::Rotation &r)
{
  out << toStr(r);
  return out;
}

std::ostream& operator << (std::ostream &out, const KDL::Vector &v)
{
  out << toStr(v);
  return out;
}

std::ostream& operator <<(std::ostream& out, const KDL::Frame &frame)
{
  out << "Position = " << frame.p << "\t Rotation = " << frame.M << endl;
  return out;
}

//=========




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
	  urdf::Pose ctPtPose = (*colgroup)[i]->origin; //contact point pose in link frame
	  double q1, q2, q3, q4;
	  ctPtPose.rotation.getQuaternion(q1, q2, q3, q4);
	  KDL::Frame cPtAsFrame(KDL::Rotation::Quaternion(q1, q2, q3, q4),  //expressed as a frame
				KDL::Vector(ctPtPose.position.x, 
					    ctPtPose.position.y, 
					    ctPtPose.position.z));
	  KDL::Frame f = cPtAsFrame * _manipulator->getLinkFrame(); //manipulator is in robot-oriented frame?
	  s.set(f, 0.05); //todo : pick a radius

	  /*
	  cout << "\n\n=========================================" << endl;
	  cout << "\n cPtAsFrame = " << toStr(cPtAsFrame) << endl;
	  cout << "\n link frame in world = " << toStr(_manipulator->getLinkFrame()) << endl;
	  cout << "cPtAsFrame * manipulator->getLinkFrame() = " << toStr(f) << endl;
	  cout << "manipulator->getLinkFrame() * cPtAsFrame = " << toStr(_manipulator->getLinkFrame() * cPtAsFrame) << endl;
	  */

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

