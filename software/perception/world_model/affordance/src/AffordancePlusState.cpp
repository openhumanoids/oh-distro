/*
 * AffordancePlusState.cpp
 *
 *  Created on: April 3, 2013
 *      Author: mfleder
 */

#include "AffordancePlusState.h"
#include <iostream>

using namespace affordance;
using namespace Eigen;
using namespace boost;
using namespace std;

/**Constructs an AffordancePlusState from an lcm message.*/
AffordancePlusState::AffordancePlusState(const drc::affordance_plus_t *msg) 
{
	initHelper(msg);
}


AffordancePlusState& AffordancePlusState::operator=( const AffordancePlusState& rhs )
{
  if (this == &rhs) // protect against invalid self-assignment
    return *this;
  
  //convert to message and then call fromMsg
  drc::affordance_plus_t msg;
  rhs.toMsg(&msg);
  fromMsg(&msg);
 
  return *this;
}

//======================MUTATORS
/**sets the state of this to that of msg*/
void AffordancePlusState::fromMsg(const drc::affordance_plus_t *msg)
{
  clear();  //clear any object state
  initHelper(msg);
}


void AffordancePlusState::clear()
{

  aff = AffPtr();
  points.clear();
  triangles.clear();
}


/**used by constructor and copy constructor*/
void AffordancePlusState::initHelper(const drc::affordance_plus_t *msg)
{
  if (points.size() != 0 || triangles.size() != 0 || aff != AffPtr())
    throw ArgumentException("shouldn't call init if these fields aren't empty");
  
  aff = AffPtr(new AffordanceState(&msg->aff));
  for(int i = 0; i < msg->npoints; i++)
    points.push_back(Vector3f(msg->points[i][0], 
                              msg->points[i][1],
                              msg->points[i][2]));

  for(int i = 0; i < msg->ntriangles; i++)
    triangles.push_back(Vector3i(msg->triangles[i][0], 
                                 msg->triangles[i][1],
                                 msg->triangles[i][2]));
}

AffordancePlusState::~AffordancePlusState()
{
}

//------methods-------
/**convert this to a drc_affordance_plus_t lcm message*/
// @comment: mfallon: wouldn't assignment be quicker here than push_back
void AffordancePlusState::toMsg(drc::affordance_plus_t *msg) const
{
  //set affordance
  aff->toMsg(&msg->aff);

  //set points
  vector<Vector3f>::const_iterator pIter;
  msg->npoints = points.size();
  for (pIter = points.begin(); pIter != points.end(); ++pIter)
    {
      Vector3f next = *pIter; //next vector

      vector<float> nextAsVector; //convert to vector of size 3
      nextAsVector.push_back(next.x());
      nextAsVector.push_back(next.y());
      nextAsVector.push_back(next.z());

      msg->points.push_back(nextAsVector);
    }

  //triangles
  vector<Vector3i>::const_iterator tIter;
  msg->ntriangles = triangles.size();
  for (tIter = triangles.begin(); tIter != triangles.end(); ++tIter)
    {
      Vector3i next = *tIter; //next vector

      vector<int32_t> nextAsVector; //convert to vector of size 3
      nextAsVector.push_back(next.x());
      nextAsVector.push_back(next.y());
      nextAsVector.push_back(next.z());
      
      msg->triangles.push_back(nextAsVector);
    }
}

