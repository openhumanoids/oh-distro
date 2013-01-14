/*
 * AffordanceServer.cpp
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#include "AffordanceServer.h"
#include <string>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/date_time.hpp>

using namespace std;
using namespace boost;

//maps from: affordance.object_id --> affordance message
namespace affordance
{

const string AffordanceServer::AFF_SERVER_CHANNEL("AFF_SERVER_COLL");
const string AffordanceServer::AFFORDANCE_ADD_REPLACE_CHANNEL("AFFORDANCE");
const string AffordanceServer::AFF_COLLECTION_ADD_REPLACE_CHANNEL("AFFORDANCE_COLLECTION");


/**@param lcm shared lcm object
 * @param affordanceChannel channel on which to listen for affordance_t messages
 * @param affCollectionChannel channel on which to listen for affordance_collection_t messages */
AffordanceServer::AffordanceServer(shared_ptr<lcm::LCM> lcm)
	: _lcm(lcm), _mapIdToAffIdMaps(), _serverMutex()
{
	//subscribe
	lcm->subscribe(AFFORDANCE_ADD_REPLACE_CHANNEL, &AffordanceServer::handleAffordanceMsg, this);
	lcm->subscribe(AFF_COLLECTION_ADD_REPLACE_CHANNEL, &AffordanceServer::handleCollectionMsg, this);

	//start publishing thread
	_pubThread = boost::thread(&AffordanceServer::runPeriodicPublish, this);
}

AffordanceServer::~AffordanceServer()
{
	//nothing to do
}

//-------methods
void AffordanceServer::handleCollectionMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
										   const drc::affordance_collection_t *collection)
{
	//loop thru and call handle Affordance
	for(int i = 0; i < collection->naffs; i++)
		handle(&collection->affs[i]);
}
void AffordanceServer::handleAffordanceMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
										   const drc::affordance_t *affordance)
{
	handle(affordance);
}

void AffordanceServer::handle(const drc::affordance_t *aff)
{
	//copy the data into and AffordanceState
	AffPtr aptr(new AffordanceState(aff));

	_serverMutex.lock(); //=======lock

	//do we have an entry for this map?
	if (_mapIdToAffIdMaps.find(aptr->_map_id) == _mapIdToAffIdMaps.end())
		_mapIdToAffIdMaps[aptr->_map_id] = AffIdMap(new unordered_map<int32_t,AffPtr>());

	//get the affordance for a particular map
	AffIdMap scene(_mapIdToAffIdMaps[aptr->_map_id]);

	//set or modify the appropriate affordance
	(*scene)[aptr->_object_id] = aptr;

	_serverMutex.unlock(); //========unlock
}

void AffordanceServer::runPeriodicPublish()
{
	boost::posix_time::seconds sleepTime(1); //publish at 1 hz
	while(true)
	{
		boost::this_thread::sleep(sleepTime); //======sleep

		_serverMutex.lock(); //======lock

		drc::affordance_collection_t msg;
		for(unordered_map<int32_t, AffIdMap>::const_iterator iter = _mapIdToAffIdMaps.begin();
			iter != _mapIdToAffIdMaps.end(); ++iter)
		{
			AffIdMap nextMap = iter->second;
			for(unordered_map<int32_t, AffPtr>::const_iterator mIt = nextMap->begin();
				mIt != nextMap->end(); ++mIt)
			{
				shared_ptr<drc::affordance_t> a(new drc::affordance_t());
				mIt->second->toMsg(a.get());
				msg.affs.push_back(*a);
			}
		}

		msg.naffs 		= msg.affs.size();
		msg.map_id 		= -1;
		msg.map_utime 	= -1;
		msg.name 		= "affordanceServerPeriodicPublish";

		_lcm->publish(AFF_SERVER_CHANNEL, &msg); //====publish

		_serverMutex.unlock(); //=====unlock
	}
}

} //namespace affordance

