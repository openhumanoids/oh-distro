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
const string AffordanceServer::AFF_PLUS_SERVER_CHANNEL("AFFORDANCE_PLUS_COLLECTION");
const string AffordanceServer::AFFORDANCE_TRACK_CHANNEL("AFFORDANCE_TRACK");
const string AffordanceServer::AFFORDANCE_PLUS_TRACK_CHANNEL("AFFORDANCE_PLUS_TRACK");
const string AffordanceServer::AFFORDANCE_FIT_CHANNEL("AFFORDANCE_FIT");
const string AffordanceServer::AFFORDANCE_PLUS_BOT_OVERWRITE_CHANNEL("AFFORDANCE_PLUS_BOT_OVERWRITE");
const string AffordanceServer::AFFORDANCE_PLUS_BASE_OVERWRITE_CHANNEL("AFFORDANCE_PLUS_BASE_OVERWRITE");

/**@param lcm shared lcm object
 * @param affordanceChannel channel on which to listen for affordance_t messages
 * @param affCollectionChannel channel on which to listen for affordance_collection_t messages */
AffordanceServer::AffordanceServer(shared_ptr<lcm::LCM> lcm)
  : _lcm(lcm), _mapIdToAffIdMaps(),
    _nextObjectUID(1), //make the min id 1 
    _serverMutex(),
    _role("robot"),
    _latest_utime(-1)
    
{
	//subscribe
	lcm->subscribe(AFFORDANCE_TRACK_CHANNEL, 
                   &AffordanceServer::handleAffordanceTrackMsg, this);

    lcm->subscribe(AFFORDANCE_PLUS_TRACK_CHANNEL, 
                   &AffordanceServer::handleAffordancePlusTrackMsg, this);

	lcm->subscribe(AFFORDANCE_FIT_CHANNEL, 
                   &AffordanceServer::handleAffordanceFitMsg, this);

	lcm->subscribe(AFFORDANCE_PLUS_BOT_OVERWRITE_CHANNEL,
                   &AffordanceServer::overwriteAffordances, this);

	lcm->subscribe(AFFORDANCE_PLUS_BASE_OVERWRITE_CHANNEL,
                   &AffordanceServer::overwriteAffordances, this);


    lcm->subscribe("ROBOT_UTIME", 
                   &AffordanceServer::handleTimeUpdate, this);

	//start publishing threads
	_pubLightThread = boost::thread(&AffordanceServer::runPeriodicLightPublish, this);
	_pubPlusThread = boost::thread(&AffordanceServer::runPeriodicPlusPublish, this);
}

AffordanceServer::~AffordanceServer()
{
	//nothing to do
}

//-------methods
void AffordanceServer::setRole(const std::string& role)
{
  _role = role;
}

void AffordanceServer::handleAffordancePlusTrackMsg(const lcm::ReceiveBuffer* rbuf, 
                                                    const std::string& channel,
                                                    const drc::affordance_plus_t *affordance_plus)
{
  if (affordance_plus->aff.aff_store_control != drc::affordance_t::UPDATE)
    {
      sendErrorMsg("expected track messages to be UPDATE");
      return; 
    }

	//copy the data into an AffordanceState
	AffPlusPtr aptr(new AffordancePlusState(affordance_plus));

	boost::unique_lock<boost::mutex> lock(_serverMutex); //=======lock, will auto unlock

	if (_mapIdToAffIdMaps.find(aptr->aff->_map_id) == _mapIdToAffIdMaps.end())
      {
        sendErrorMsg("how are we tracking an object we don't have a map for");
        return; 
      }

	//get the relevant map
	AffIdMap scene(_mapIdToAffIdMaps[aptr->aff->_map_id]);

	if (scene->find(aptr->aff->_uid) == scene->end())
      {
        sendErrorMsg("how are we tracking an object that's not in the corresponding map");
        return; 
      }

    (*scene)[aptr->aff->_uid] = aptr; //actually update

	lock.unlock(); //========unlock

  publishPlusOneTime(); //publish this update
}



void AffordanceServer::handleAffordanceTrackMsg(const lcm::ReceiveBuffer* rbuf, 
                                                const std::string& channel,
                                                const drc::affordance_t *affordance)
{
  if (affordance->aff_store_control != drc::affordance_t::UPDATE)
    {
      sendErrorMsg("expected track messages to be UPDATE");
      return; 
    }

	//copy the data into an AffordanceState
	AffPtr aptr(new AffordanceState(affordance));

	boost::unique_lock<boost::mutex> lock(_serverMutex); //=======lock, will auto unlock

	if (_mapIdToAffIdMaps.find(aptr->_map_id) == _mapIdToAffIdMaps.end())
      {
        sendErrorMsg("how are we tracking an object we don't have a map for");
        return; 
      }

	//get the relevant map
	AffIdMap scene(_mapIdToAffIdMaps[aptr->_map_id]);

	if (scene->find(aptr->_uid) == scene->end())
      {
        sendErrorMsg("how are we tracking an object that's not in the corresponding map");
        return; 
      }

    (*scene)[aptr->_uid]->aff = aptr; //actually update
}

void AffordanceServer::overwriteAffordances(const lcm::ReceiveBuffer* rbuf, 
                                            const std::string& channel,
                                            const drc::affordance_plus_collection_t *affordance_plus_col)
{
  if ((channel == AFFORDANCE_PLUS_BOT_OVERWRITE_CHANNEL) && (_role.compare("robot") != 0)) return;
  if ((channel == AFFORDANCE_PLUS_BASE_OVERWRITE_CHANNEL) && (_role.compare("base") != 0)) return;

  boost::unique_lock<boost::mutex> lock(_serverMutex);

  // first update existing affordances
  for (int i = 0; i < affordance_plus_col->naffs; ++i) {
    AffPlusPtr aPlusPtr(new AffordancePlusState(&affordance_plus_col->affs_plus[i]));
    AffPtr aptr = aPlusPtr->aff;

    // add map for affordance if it des not exist
    bool added = false;
    if (_mapIdToAffIdMaps.find(aptr->_map_id) == _mapIdToAffIdMaps.end()) {
      _mapIdToAffIdMaps[aptr->_map_id] = AffIdMap(new unordered_map<int32_t,AffPlusPtr>());
      added = true;
    }
    AffIdMap scene(_mapIdToAffIdMaps[aptr->_map_id]);
    
    const drc::affordance_t* aff = &affordance_plus_col->affs_plus[i].aff;
    
    // replace affordance if it didn't exist before, or if it is labeled as 'new'
    if (added || aff->aff_store_control == drc::affordance_t::NEW) {
      (*scene)[aptr->_uid] = aPlusPtr;
      _nextObjectUID = max((int)_nextObjectUID, aptr->_uid);
      _nextObjectUID++;
    }
    
    // or just update it if it is labeled as 'update'
    else if (aff->aff_store_control == drc::affordance_t::UPDATE) {
      (*scene)[aptr->_uid]->aff = aptr;
    }
  }

  // next remove affordances that are not in the incoming list
  for(unordered_map<int32_t, AffIdMap>::const_iterator iter = _mapIdToAffIdMaps.begin();
      iter != _mapIdToAffIdMaps.end(); ++iter) {
    AffIdMap curMap = iter->second;
    unordered_map<int32_t,AffPlusPtr>::iterator it = curMap->begin();
    while (it != curMap->end()) {
      AffPlusPtr a = it->second;
      bool found = false;
      for (int i = 0; i < affordance_plus_col->naffs; ++i) {
        if (a->aff->_uid == affordance_plus_col->affs_plus[i].aff.uid) {
          found = true;
          break;
        }
      }
      if (!found) ++it;
      else it = curMap->erase(it);
    }
  }  
}

  void AffordanceServer::nukeAndOverwrite(const lcm::ReceiveBuffer* rbuf, 
                                          const std::string& channel,
                                          const drc::affordance_plus_collection_t *affordance_plus_col)
  {
	  boost::unique_lock<boost::mutex> lock(_serverMutex); //=======lock, will auto unlock

    _mapIdToAffIdMaps.clear(); //nuke

    _nextObjectUID = 0; //will set in this method
    
    //now overwrite
    for(int i = 0; i < affordance_plus_col->naffs; i++)
      {
        AffPlusPtr aPlusPtr(new AffordancePlusState(&affordance_plus_col->affs_plus[i]));
        AffPtr aptr = aPlusPtr->aff;
       
        //---add an entry for the corresponding map if needed
        if (_mapIdToAffIdMaps.find(aptr->_map_id) == _mapIdToAffIdMaps.end()) //no map 
          {
            _mapIdToAffIdMaps[aptr->_map_id] = AffIdMap(new unordered_map<int32_t,AffPlusPtr>()); //add map
          }

        //get the map
        AffIdMap scene(_mapIdToAffIdMaps[aptr->_map_id]);

        //set 
        (*scene)[aptr->_uid] = aPlusPtr;

        //need to increase _nextObjectUID ?
        _nextObjectUID = max((int)_nextObjectUID, aptr->_uid);
      }

    _nextObjectUID++; //make this 1 more than the largest uid seen
  }



void AffordanceServer::handleAffordanceFitMsg(const lcm::ReceiveBuffer* rbuf, 
                                              const std::string& channel,
                                              const drc::affordance_plus_t *affordance_plus)
{
	//copy the data into and AffordanceState
    AffPlusPtr aPlusPtr(new AffordancePlusState(affordance_plus));
	AffPtr aptr = aPlusPtr->aff;

	boost::unique_lock<boost::mutex> lock(_serverMutex); //=======lock, will auto unlock

	//do we have an entry for this map?
	if (_mapIdToAffIdMaps.find(aptr->_map_id) == _mapIdToAffIdMaps.end()) //no map 
      {
        if (affordance_plus->aff.aff_store_control != drc::affordance_t::NEW) //not new ?
          {
            sendErrorMsg("affServer: non-NEW msg : no such map");
            return;
          }
        _mapIdToAffIdMaps[aptr->_map_id] = AffIdMap(new unordered_map<int32_t,AffPlusPtr>()); //add map
      }

	//get the affordance for a particular map
	AffIdMap scene(_mapIdToAffIdMaps[aptr->_map_id]);

	if (scene->find(aptr->_uid) == scene->end())  //object not found
      {
        if (affordance_plus->aff.aff_store_control != drc::affordance_t::NEW) //new object?
          {
            sendErrorMsg("affServer: non-NEW msg : no such object");
            return;
          }
        aptr->_uid = _nextObjectUID++; //set object id
      }
    
	//set or modify the appropriate affordance
	(*scene)[aptr->_uid] = aPlusPtr;

    //delete if that's what the update was
    if (affordance_plus->aff.aff_store_control == drc::affordance_t::DELETE)
        (*scene).erase(aptr->_uid);
}

  void AffordanceServer::handleTimeUpdate(const lcm::ReceiveBuffer*rbuf,
                                          const std::string &channel,
                                          const drc::utime_t *new_time)
  {
    _latest_utime = new_time->utime;
  }

  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //=======================================
  //======================publishing

void AffordanceServer::runPeriodicLightPublish()
{
	boost::posix_time::milliseconds sleepTime(50); //publish at 20 hz
	while(true)
	{
		boost::this_thread::sleep(sleepTime); //======sleep

	  boost::unique_lock<boost::mutex> lock(_serverMutex); //=======lock, will auto unlock

		drc::affordance_collection_t msg;
		for(unordered_map<int32_t, AffIdMap>::const_iterator iter = _mapIdToAffIdMaps.begin();
			iter != _mapIdToAffIdMaps.end(); ++iter)
		{
			AffIdMap nextMap = iter->second;
			for(unordered_map<int32_t, AffPlusPtr>::const_iterator mIt = nextMap->begin();
				mIt != nextMap->end(); ++mIt)
			{
				shared_ptr<drc::affordance_t> a(new drc::affordance_t());
				mIt->second->aff->toMsg(a.get());
                a->utime = _latest_utime;
				msg.affs.push_back(*a);
			}
		}

		msg.naffs 		= msg.affs.size();
		msg.map_id 		= -1;
		msg.utime 	    = _latest_utime;
		msg.name 		= "affordanceServerPeriodicLightPublish";

		_lcm->publish(AFF_SERVER_CHANNEL, &msg); //====publish
	} //lock goes out of scope
}


void AffordanceServer::runPeriodicPlusPublish()
{
	boost::posix_time::seconds sleepTime(1); //publish at 1 hz
	while(true)
	{
		boost::this_thread::sleep(sleepTime); //======sleep
        publishPlusOneTime();
	}
}


void AffordanceServer::publishPlusOneTime()
{

	boost::unique_lock<boost::mutex> lock(_serverMutex); //=======lock, will auto unlock
  
  drc::affordance_plus_collection_t msg;
  for(unordered_map<int32_t, AffIdMap>::const_iterator iter = _mapIdToAffIdMaps.begin();
      iter != _mapIdToAffIdMaps.end(); ++iter)
    {
      AffIdMap nextMap = iter->second;
      for(unordered_map<int32_t, AffPlusPtr>::const_iterator mIt = nextMap->begin();
          mIt != nextMap->end(); ++mIt)
        {
          shared_ptr<drc::affordance_plus_t> a(new drc::affordance_plus_t());
          mIt->second->toMsg(a.get());
          a->aff.utime = _latest_utime;
          msg.affs_plus.push_back(*a);
        }
    }
  
  msg.naffs 		= msg.affs_plus.size();
  msg.map_id 		= -1;
  msg.utime 	    = _latest_utime;
  msg.name 		= "affordanceServerPeriodicPlusPublish";
  
  _lcm->publish(AFF_PLUS_SERVER_CHANNEL, &msg); //====publish
}

void AffordanceServer::sendErrorMsg(const string &s) const
{
  //todo 
  cout << "\n\n===ERROR:\n" << s << "\n========\n\n" << endl;
}

} //namespace affordance

