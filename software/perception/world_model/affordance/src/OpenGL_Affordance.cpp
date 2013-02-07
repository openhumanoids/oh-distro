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
    _affordance(affordance)
{
}

OpenGL_Affordance::~OpenGL_Affordance()
{
}

/**sets the state of the object we want to draw and then draws*/
void OpenGL_Affordance::draw()
{
  //frame will be used by everything -- only compute once
  KDL::Frame frame = _affordance->getFrame();

  OpenGL_Object *obj; 
  if (_affordance->_otdf_type == AffordanceState::CYLINDER)
  {
      _cylinder.set(frame, Vector2f(_affordance->radius(),
				    _affordance->length()));
      obj = &_cylinder;
   }
   else if (_affordance->_otdf_type == AffordanceState::LEVER)
      throw runtime_error("not handling lever right now");
   else if (_affordance->_otdf_type == AffordanceState::BOX)
   {
      _box.set(frame, Vector3f(_affordance->length(),
			       _affordance->width(),
			       _affordance->height()));
      obj = &_box;
    }
    else if (_affordance->_otdf_type == AffordanceState::SPHERE)
    {
      _sphere.set(frame, _affordance->radius());
      obj = &_sphere;
     }
     else
      throw runtime_error("unhandled affordance state");
      
  if (_isHighlighted) 
    obj->draw(_highlightColor);
  else
    obj->draw(_affordance->getColor());
}


//------observers
AffConstPtr OpenGL_Affordance::getAffordance() const
{
  return _affordance;
}

bool OpenGL_Affordance::isSupported(affordance::AffConstPtr affordance)
{
  return affordance->_otdf_type == AffordanceState::CYLINDER ||
    affordance->_otdf_type == AffordanceState::LEVER ||
    affordance->_otdf_type == AffordanceState::BOX ||
    affordance->_otdf_type == AffordanceState::SPHERE;
}
