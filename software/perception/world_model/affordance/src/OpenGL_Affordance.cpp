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
				     Eigen::Vector3f highlightedColor)
  : _affordance(affordance), _highlightColor(highlightedColor), _isHighlighted(isHighlighted)
{
}

OpenGL_Affordance::~OpenGL_Affordance()
{
}

/**sets the state of the object we want to draw and then draws*/
void OpenGL_Affordance::draw()
{
  //frame will be used by everything -- only compute once
  KDL::Frame frame;
  _affordance->getFrame(frame);

  OpenGL_Object *obj; 
  switch(_affordance->_otdf_id)
    {
      
    case AffordanceState::CYLINDER:
      _cylinder.set(frame, Vector2f(_affordance->radius(),
				    _affordance->length()));
      obj = &_cylinder;
      break;
      
    case AffordanceState::LEVER:
      throw runtime_error("not handling lever right now");
      break;
      
    case AffordanceState::BOX:
      _box.set(frame, Vector3f(_affordance->length(),
			       _affordance->width(),
			       _affordance->height()));
      obj = &_box;
      break;
      
    case AffordanceState::SPHERE:
      _sphere.set(frame, _affordance->radius());
      obj = &_sphere;
      break;
      
    default:
      throw runtime_error("unhandled affordance state");
    }
  if (_isHighlighted) 
  {
      obj->draw(_highlightColor);
  }
  else
  {
      obj->set_color(_affordance->getColor());
      obj->draw();
  }
}


//-----mutators
void OpenGL_Affordance::setHighlighted(bool highlight)
{
    _isHighlighted = highlight;
}

//------observers
AffConstPtr OpenGL_Affordance::getAffordance() const
{
  return _affordance;
}

