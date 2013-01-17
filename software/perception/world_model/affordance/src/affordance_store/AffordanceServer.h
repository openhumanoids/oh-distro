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

#include "affordance/AffordanceState.h"

namespace affordance
{

//maps from : affordance.objectId --> that affordance
typedef boost::shared_ptr<boost::unordered_map<int32_t, AffPtr> > AffIdMap;

class AffordanceServer {

	//----fields
public:
	/**Do not publish to this channel.  This channel is reserved for this server
	 * to periodically publish the server state.*/
	const static std::string AFF_SERVER_CHANNEL;

	/**an affordance_t message received on this channel will be added to the server and
	 * will replace and existing affordances with the same map/objectId.*/
	const static std::string AFFORDANCE_ADD_REPLACE_CHANNEL;

	/**an affordance_collection_t message received on this channel will be added to the
	 * server and will replace and existing affordances with the same map/objectId.*/
		const static std::string AFF_COLLECTION_ADD_REPLACE_CHANNEL;

private:
	/**shared lcm object */
	boost::shared_ptr<lcm::LCM> _lcm;

	/**map id --> {affordanceObjectId --> affordance}*/
	boost::unordered_map<int32_t, AffIdMap> _mapIdToAffIdMaps;

	/**periodic publishing thread*/
	boost::thread _pubThread;

	/**mutex for _mapIdToAffIdMaps*/
	boost::mutex _serverMutex;

	//--------constructor/destructor
public:
	AffordanceServer(const boost::shared_ptr<lcm::LCM> lcm);
	virtual ~AffordanceServer();

	//---------message handling
private:
	void handleCollectionMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
						  	 const drc::affordance_collection_t *collection);

	void handleAffordanceMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
						  	 const drc::affordance_t *affordance);

	void handle(const drc::affordance_t *aff);

	void runPeriodicPublish(void);
};

} //namespace affordance

#endif /* AFFORDANCESERVER_H_ */
