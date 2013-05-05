/*
 * AffordanceServer.h
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#ifndef AFFORDANCESERVER_H_
#define AFFORDANCESERVER_H_

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/thread.hpp>

#include "affordance/AffordancePlusState.h"

namespace affordance
{

//maps from : affordance.objectId --> that affordance
typedef boost::shared_ptr<boost::unordered_map<int32_t, AffPlusPtr> > AffIdMap;

class AffordanceServer {

	//----fields
public:
	/**Do not publish to this channel.  This channel is reserved for this server
	 * to periodically publish the server state in a light format: 
     affordance_collection_t.*/
	const static std::string AFF_SERVER_CHANNEL;

	/**Do not publish to this channel.  This channel is reserved for this server
	 * to periodically publish the entire server state as 
     affordance_plus_collection_t.*/
	const static std::string AFF_PLUS_SERVER_CHANNEL;


	/**Channel for affordance updates from tracking
	   an affordance_t message received on this channel will be added to the server 
       and  will replace any existing affordance with the same map/objectId.*/
	const static std::string AFFORDANCE_TRACK_CHANNEL;

	/**Channel for affordance updates from tracking
	   an affordance_t message received on this channel will be added to the server 
       and  will replace any existing affordance with the same map/objectId.*/
	const static std::string AFFORDANCE_PLUS_TRACK_CHANNEL;

	/**Channel for new affordances from fitting
	   an affordance_t message received on this channel will be added to the
	 * server and will replace anY existing affordance with the same map/objectId.
	 */
	const static std::string AFFORDANCE_FIT_CHANNEL;

    /**if a message is received on this channel, will completely 
       overwrite the state of the affordance server*/
	const static std::string AFFORDANCE_PLUS_BOT_OVERWRITE_CHANNEL;

    /**if a message is received on this channel, will completely 
     overwrite the state of the affordance server*/
	const static std::string AFFORDANCE_PLUS_BASE_OVERWRITE_CHANNEL;


private:
	/**shared lcm object */
	boost::shared_ptr<lcm::LCM> _lcm;

	/**map id --> {affordanceObjectId --> affordance}*/
	boost::unordered_map<int32_t, AffIdMap> _mapIdToAffIdMaps;

	/**uid that will be assigned to the next object added*/
	uint _nextObjectUID;

	/**periodic publishing thread of affordance collection*/
	boost::thread _pubLightThread;

	/**periodic publishing thread of all plus objects*/
	boost::thread _pubPlusThread;

    /**period publishing thread of affordance_plus_collection*/

	/**mutex for _mapIdToAffIdMaps*/
	boost::mutex _serverMutex;

	/** role of this instance (robot or base) */
	std::string _role;

    /**most recently received utime on ROBOT_UTIME channel*/
    int64_t _latest_utime;

	//--------constructor/destructor
public:
	AffordanceServer(const boost::shared_ptr<lcm::LCM> lcm);
	virtual ~AffordanceServer();

	void setRole(const std::string& role);

	//---------message handling
private:
	void handleAffordanceTrackMsg(const lcm::ReceiveBuffer* rbuf, 
                                  const std::string& channel,
                                  const drc::affordance_t *affordance);


	void handleAffordancePlusTrackMsg(const lcm::ReceiveBuffer* rbuf, 
                                      const std::string& channel,
                                      const drc::affordance_plus_t *affordance_plus);

	void handleAffordanceFitMsg(const lcm::ReceiveBuffer* rbuf, 
                                const std::string& channel,
                                const drc::affordance_plus_t *affordance_plus);

	void overwriteAffordances(const lcm::ReceiveBuffer* rbuf, 
                              const std::string& channel,
                              const drc::affordance_plus_collection_t *affordance_plus);

    void nukeAndOverwrite(const lcm::ReceiveBuffer* rbuf, 
                          const std::string& channel,
                          const drc::affordance_plus_collection_t *affordance_plus);

    void handleTimeUpdate(const lcm::ReceiveBuffer* rbuf, 
                         const std::string& channel,
                         const drc::utime_t *time);

	void runPeriodicLightPublish(void);
	void runPeriodicPlusPublish(void);

    void publishPlusOneTime(void);

    void sendErrorMsg(const std::string &) const;
};

} //namespace affordance

#endif /* AFFORDANCESERVER_H_ */
