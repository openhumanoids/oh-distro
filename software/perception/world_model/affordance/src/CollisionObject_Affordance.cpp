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
CollisionObject_Affordance::CollisionObject_Affordance(const AffordanceState &affordance, const string &id) 
  : Collision_Object(id), _affordance(affordance)
{
}

CollisionObject_Affordance::~CollisionObject_Affordance()
{
}

