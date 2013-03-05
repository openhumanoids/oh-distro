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

//maps from: affordance.uid --> affordance message
namespace affordance
{

const string AffordanceServer::AFF_SERVER_CHANNEL("AFFORDANCE_COLLECTION");
const string AffordanceServer::AFFORDANCE_TRACK_CHANNEL("AFFORDANCE_TRACK");
const string AffordanceServer::AFFORDANCE_FIT_CHANNEL("AFFORDANCE_FIT");


/**@param lcm shared lcm object
 * @param affordanceChannel channel on which to listen for affordance_t messages
 * @param affCollectionChannel channel on which to listen for affordance_collection_t messages */
AffordanceServer::AffordanceServer(shared_ptr<lcm::LCM> lcm)
  : _lcm(lcm), _mapIdToAffIdMaps(), _nextObjectUID(0), _serverMutex()
{
	//subscribe
	lcm->subscribe(AFFORDANCE_TRACK_CHANNEL, &AffordanceServer::handleAffordanceTrackMsg, this);
	lcm->subscribe(AFFORDANCE_FIT_CHANNEL, &AffordanceServer::handleAffordanceFitMsg, this);

	//start publishing thread
	_pubThread = boost::thread(&AffordanceServer::runPeriodicPublish, this);
}

AffordanceServer::~AffordanceServer()
{
	//nothing to do
}

//-------methods
void AffordanceServer::handleAffordanceTrackMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
						const drc::affordance_t *affordance)
{
  if (affordance->aff_store_control != drc::affordance_t::UPDATE)
    {
      cout << "\n expected track messages to be UPDATE's" << endl;
      return; //todo : display system error
    }

	//copy the data into an AffordanceState
	AffPtr aptr(new AffordanceState(affordance));

	_serverMutex.lock(); //=======lock

	if (_mapIdToAffIdMaps.find(aptr->_map_id) == _mapIdToAffIdMaps.end())
      {
        cout << "\n how are we tracking an object we don't have a map for" << endl;
        return; //todo: display system error
      }

	//get the relevant map
	AffIdMap scene(_mapIdToAffIdMaps[aptr->_map_id]);

	if (scene->find(aptr->_uid) == scene->end())
      {
        cout << "\n how are we tracking an object that's not in the corresponding map" << endl;
        return; //todo: display system error
      }

    (*scene)[aptr->_uid] = aptr; //actually update

	_serverMutex.unlock(); //========unlock
}



void AffordanceServer::handleAffordanceFitMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                                              const drc::affordance_t *affordance)
{
	//copy the data into and AffordanceState
	AffPtr aptr(new AffordanceState(affordance));

	_serverMutex.lock(); //=======lock

	//do we have an entry for this map?
	if (_mapIdToAffIdMaps.find(aptr->_map_id) == _mapIdToAffIdMaps.end()) //no map 
      {
        if (affordance->aff_store_control != drc::affordance_t::NEW) //not new ?
          {
            cout << "\naffServer: non-NEW msg : no such map" << endl; //todo: system error
            return;
          }
        _mapIdToAffIdMaps[aptr->_map_id] = AffIdMap(new unordered_map<int32_t,AffPtr>()); //add map
      }

	//get the affordance for a particular map
	AffIdMap scene(_mapIdToAffIdMaps[aptr->_map_id]);

	if (scene->find(aptr->_uid) == scene->end())  //object not found
      {
        if (affordance->aff_store_control != drc::affordance_t::NEW) //new object?
          {
            cout << "\naffServer: non-NEW msg : no such object" << endl;
            return;
          }
        aptr->_uid = _nextObjectUID++; //set object id
      }

    
	//set or modify the appropriate affordance
	(*scene)[aptr->_uid] = aptr;

    //delete if that's what the update was
    if (affordance->aff_store_control == drc::affordance_t::DELETE)
        (*scene).erase(aptr->_uid);

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
		msg.utime 	= -1;
		msg.name 		= "affordanceServerPeriodicPublish";

		_lcm->publish(AFF_SERVER_CHANNEL, &msg); //====publish

		_serverMutex.unlock(); //=====unlock
	}
}

} //namespace affordance

