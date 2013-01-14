/*
 * AffordanceUpWrapper.h
 *
 *  Created on: Jan 14, 2013
 *      Author: mfleder
 */

#ifndef AFFORDANCEUPWRAPPER_H_
#define AFFORDANCEUPWRAPPER_H_

#include "AffordanceServer.h"
#include <vector>

namespace affordance {

class AffordanceUpWrapper
{
//--------fields
private:
	/**shared lcm object */
	boost::shared_ptr<lcm::LCM> _lcm;

	/**affordances in the server*/
	std::vector<AffPtr> _affordances;

	/**mutex for accessing _affordances*/
	boost::mutex _accessMutex;

	//-------constructor/destructor
public:
	AffordanceUpWrapper(const boost::shared_ptr<lcm::LCM> lcm);
	virtual ~AffordanceUpWrapper();

	//-----------accessors
	void getAllAffordances(std::vector<AffPtr> &affs);

	//-----------mutators
	void addOrReplace(const AffordanceState &aff);

private:
	void handleCollectionMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
							 const drc::affordance_collection_t *collection);

};

} /* namespace affordance */
#endif /* AFFORDANCEUPWRAPPER_H_ */
