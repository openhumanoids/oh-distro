/*
 * OpenGLAffordance.cpp
 *
 *  Created on: Jan 14, 2013
 *      Author: mfleder
 */

#include "OpenGL_Affordance.h"

using namespace affordance;
//using namespace opengl;
using namespace std;
using namespace boost;
using namespace Eigen;
//--------------constructor/destructor
OpenGL_Affordance::OpenGL_Affordance(AffConstPtr affordance, 
				     bool isHighlighted,
				     Eigen::Vector3f highlightColor)
  : OpenGL_Object(affordance->getGUIDAsString(), isHighlighted, highlightColor),
    _frame(affordance->getFrame()),
    _color(affordance->getColor())
{

  //frame will be used by everything -- only compute once
  if (affordance->_otdf_type == AffordanceState::CYLINDER)
  {
      _cylinder.set(_frame, Vector2f(affordance->radius(),
				    affordance->length()));
      _obj = &_cylinder;
   }
   else if (affordance->_otdf_type == AffordanceState::LEVER)
      throw runtime_error("not handling lever right now");
   else if (affordance->_otdf_type == AffordanceState::BOX)
   {
      _box.set(_frame, Vector3f(affordance->length(),
			       affordance->width(),
			       affordance->height()));
      _obj = &_box;
    }
    else if (affordance->_otdf_type == AffordanceState::SPHERE)
    {
      _sphere.set(_frame, affordance->radius());
      _obj = &_sphere;
     }
     else
      throw runtime_error("unhandled affordance state");

}

OpenGL_Affordance::~OpenGL_Affordance()
{
}

/**sets the state of the object we want to draw and then draws*/
void OpenGL_Affordance::draw()
{
  if (_isHighlighted) 
    _obj->draw(_highlightColor);
  else
    _obj->draw(_color);
}


bool OpenGL_Affordance::isSupported(affordance::AffConstPtr affordance)
{
  return affordance->_otdf_type == AffordanceState::CYLINDER ||
    affordance->_otdf_type == AffordanceState::LEVER ||
    affordance->_otdf_type == AffordanceState::BOX ||
    affordance->_otdf_type == AffordanceState::SPHERE;
}
