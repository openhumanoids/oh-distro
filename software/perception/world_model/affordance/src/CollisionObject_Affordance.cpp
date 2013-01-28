/*
 * OpenGLAffordance.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: mfleder
 */

#include "CollisionObject_Affordance.h"

using namespace affordance;
//using namespace opengl;
using namespace std;
using namespace boost;
using namespace Eigen;
//--------------constructor/destructor
CollisionObject_Affordance::CollisionObject_Affordance(AffConstPtr affordance, const string &id) 
  : Collision_Object(id), _affordance(affordance)
{
  if (!isSupported(affordance))
    throw ArgumentException("Unsupported affordance passed to CollisionObject_Affordance Constructor: otdf_id = " 
			    + affordance->_otdf_id);
}

CollisionObject_Affordance::~CollisionObject_Affordance()
{
  
}

bool CollisionObject_Affordance::isSupported(AffConstPtr affordance)
{
  return affordance->_otdf_id == AffordanceState::CYLINDER ||
    affordance->_otdf_id == AffordanceState::BOX ||
    affordance->_otdf_id == AffordanceState::SPHERE;
}

