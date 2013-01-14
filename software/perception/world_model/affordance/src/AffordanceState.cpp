/*
 * AffordanceState.cpp
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#include "AffordanceState.h"
#include "boost/assign.hpp"

using namespace boost;
using namespace std;

namespace affordance
{

/**Constructs an AffordanceState from an lcm message.*/
AffordanceState::AffordanceState(const drc::affordance_t *msg)
{
	initHelper(msg);
}

/**copy constructor by using toMsg and then the constructor*/
AffordanceState::AffordanceState(const AffordanceState &other)
{
	drc::affordance_t *msg;
	other.toMsg(msg);
	initHelper(msg);
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
	drc::affordance_t *msg;
	rhs.toMsg(msg);
	initHelper(msg);

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

	//this should really be static
	//SOME WEIRD C++ issues require setting these to variables
	int16_t c 		= drc::affordance_t::CYLINDER;
	int16_t lev 	= drc::affordance_t ::LEVER;
	idToEnum[c] 	= AffordanceState::CYLINDER;
	idToEnum[lev]  	=  AffordanceState::LEVER;

	//argument check
	if (idToEnum.find(msg->otdf_id) == idToEnum.end())
		throw InvalidOtdfID("not recognized");

	_otdf_id = idToEnum[msg->otdf_id];

	for(uint i = 0; i < msg->nstates; i++)
		_states[msg->state_names[i]] = msg->states[i];

	for (uint i = 0; i < msg->nparams; i++)
		_params[msg->param_names[i]] = msg->params[i];
}


AffordanceState::~AffordanceState()
{
	//
}

//------methods-------

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


} //namespace affordance
