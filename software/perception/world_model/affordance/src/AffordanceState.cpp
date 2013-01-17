/*
 * AffordanceState.cpp
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#include "AffordanceState.h"
#include "boost/assign.hpp"
#include <iostream>

using namespace affordance;
using namespace Eigen;
using namespace boost;
using namespace std;


//--------set common name fields for drc::affordance_t

string AffordanceState::X_NAME 		= "x";
string AffordanceState::Y_NAME 		= "y";
string AffordanceState::Z_NAME 		= "z";
string AffordanceState::ROLL_NAME 	= "roll";
string AffordanceState::PITCH_NAME 	= "pitch";
string AffordanceState::YAW_NAME 	= "yaw";
string AffordanceState::RADIUS_NAME = "radius";
string AffordanceState::LENGTH_NAME = "length";

/**Constructs an AffordanceState from an lcm message.*/
AffordanceState::AffordanceState(const drc::affordance_t *msg)
{
	initIdEnumMap(); //todo : should be static

	initHelper(msg);
}

/**copy constructor by using toMsg and then the constructor*/
AffordanceState::AffordanceState(const AffordanceState &other)
{
	initIdEnumMap(); //todo : should be static

	drc::affordance_t msg;
	other.toMsg(&msg);
	initHelper(&msg);
}

/**Constructs an affordance with emtpy state*/
AffordanceState::AffordanceState(const string &name) : _name(name)
{
	initIdEnumMap(); //todo : should be static
}

AffordanceState& AffordanceState::operator=( const AffordanceState& rhs )
{
	if (this == &rhs) // protect against invalid self-assignment
		return *this;

	//clear an object state
	_states.clear();
	_params.clear();
	_ptinds.clear();

	//convert to message and then run init
	drc::affordance_t msg;
	rhs.toMsg(&msg);
	initHelper(&msg);

	return *this;
}


/**used by constructor and copy constructor*/
void AffordanceState::initHelper(const drc::affordance_t *msg)
{
	if (_states.size() != 0 || _params.size() != 0 || _ptinds.size() != 0)
		throw ArgumentException("shouldn't call init if these fields aren't empty");

	_map_utime 	= msg->map_utime;
	_map_id 	= msg->map_id;
	_object_id 	= msg->object_id;
	_name 		= msg->name;
	_ptinds 	= msg->ptinds;


	//argument check
	if (idToEnum.find(msg->otdf_id) == idToEnum.end())
		throw InvalidOtdfID("not recognized");

	_otdf_id = idToEnum[msg->otdf_id];

	for(uint i = 0; i < msg->nstates; i++)
		_states[msg->state_names[i]] = msg->states[i];

	for (uint i = 0; i < msg->nparams; i++)
		_params[msg->param_names[i]] = msg->params[i];
}

/**initialize the idEnumMap*/
void AffordanceState::initIdEnumMap()
{
	//this should really be static.
	//SOME WEIRD C++ issues require setting these to variables first
	//before using as the key for the map.
	int16_t c 		 = drc::affordance_t::CYLINDER;
	int16_t lev 	 = drc::affordance_t ::LEVER;
	int16_t box		 = drc::affordance_t::BOX;
	int16_t sphere	 = drc::affordance_t::SPHERE;
	idToEnum[c]  	 = AffordanceState::CYLINDER;
	idToEnum[lev]  	 = AffordanceState::LEVER;
	idToEnum[box] 	 = AffordanceState::BOX;
	idToEnum[sphere] = AffordanceState::SPHERE;
}

AffordanceState::~AffordanceState()
{
	//
}

//------methods-------
/**convert this to a drc_affordacne_t lcm message*/
void AffordanceState::toMsg(drc::affordance_t *msg) const
{
	msg->map_utime 	= _map_utime;
	msg->map_id		= _map_id;
	msg->object_id 	= _object_id;
	msg->otdf_id	= _otdf_id;
	msg->name		= _name;


	unordered_map<string,double>::const_iterator iter;

	//params
	msg->nparams = _params.size();
	for(iter = _params.begin(); iter != _params.end(); ++iter)
	{
		msg->param_names.push_back(iter->first);
		msg->params.push_back(iter->second);
	}

	//states
	msg->nstates = _states.size();
	for(iter = _states.begin(); iter != _states.end(); ++iter)
	{
		msg->state_names.push_back(iter->first);
		msg->states.push_back(iter->second);
	}

	//ptinds
	msg->nptinds = _ptinds.size();
	for(int i = 0; i < _ptinds.size(); i++)
		msg->ptinds.push_back(_ptinds[i]);
}

string AffordanceState:: getName() const
{
  return _name;
}


/**@return x,y,z or throws an exception if any of those are not present*/
Vector3f AffordanceState::getXYZ() const
{
	//defensive checks
	assertContainsKey(_params, X_NAME);
	assertContainsKey(_params, Y_NAME);
	assertContainsKey(_params, Z_NAME);

	//using find method b/c operator[] isn't a const method
	return Vector3f(_params.find(X_NAME)->second,
					_params.find(Y_NAME)->second,
					_params.find(Z_NAME)->second);
}

/**@return true if we have roll/pitch/yaw parameters.  false otherwise*/
bool AffordanceState::hasRPY() const
{
	return (_params.find(ROLL_NAME) != _params.end() &&
			_params.find(PITCH_NAME) != _params.end() &&
			_params.find(YAW_NAME) != _params.end());
}

/**@return roll,pitch,yaw or throws an exception if any of those are not present*/
Vector3f AffordanceState::getRPY() const
{
	assertContainsKey(_params, ROLL_NAME);
	assertContainsKey(_params, PITCH_NAME);
	assertContainsKey(_params, YAW_NAME);

	//using find method b/c operator[] isn't a const method
	return Vector3f(_params.find(ROLL_NAME)->second,
					_params.find(PITCH_NAME)->second,
					_params.find(YAW_NAME)->second);
}

/**@return radius or throws exception if not present*/
double AffordanceState::radius() const
{
	assertContainsKey(_params, RADIUS_NAME);
	return _params.find(RADIUS_NAME)->second;
}

/**@return length or throws exception if not present*/
double AffordanceState::length() const
{
	assertContainsKey(_params, LENGTH_NAME);
	return _params.find(LENGTH_NAME)->second;
}

void AffordanceState::assertContainsKey(const unordered_map<string, double> &map,
					   	   	   	   	   	const string &key)
{
	if (map.find(key) == map.end())
		throw KeyNotFoundException("Key = " + key + " not found");
}

string AffordanceState::toStr(unordered_map<string,double> m)
{
	stringstream s;
	for(unordered_map<string,double>::const_iterator it = m.begin();
	    it != m.end(); ++it)
	{
		s << "(" << it->first << ", "
 		  << it->second << ")\n";
	}
	return s.str();
}


namespace affordance
{
	/**operator << */
	ostream& operator<<(ostream& out, const AffordanceState& other )
	{
		out << "=====Affordance " << other._name << "========" << endl;
		out << "(mapId, objectId, otdfId) = (" << other._map_id << ", "
			  << other._object_id << ", " << other._otdf_id << ")\n";
		out << "---params: \n" << AffordanceState::toStr(other._params) << endl;;
		out << "--states: \n" << AffordanceState::toStr(other._states) << endl;
		return out;
	}
} //namespace affordance

