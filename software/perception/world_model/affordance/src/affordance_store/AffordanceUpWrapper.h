/*
 * AffordanceUpWrapper.h
 *
 *  Created on: Jan 14, 2013
 *      Author: mfleder
 */

#ifndef AFFORDANCEUPWRAPPER_H_
#define AFFORDANCEUPWRAPPER_H_

#include "affordance/AffordanceServer.h"
#include <vector>

namespace affordance {

class AffordanceUpWrapper
{
//--------fields
private:
	/**shared lcm object */
	const boost::shared_ptr<lcm::LCM> _lcm;

	/**affordances in the server*/
	std::vector<AffConstPtr> _affordances;

        /**affordance plus objects in the server*/
        std::vector<AffPlusPtr> _affordances_plus;

	/**mutex for accessing _affordances*/
	boost::mutex _accessMutex;

	//-------constructor/destructor
public:
	AffordanceUpWrapper(const boost::shared_ptr<lcm::LCM> lcm);
	virtual ~AffordanceUpWrapper();

	//-----------accessors
	void getAllAffordances(std::vector<AffConstPtr> &affs);
	void getAllAffordancesPlus(std::vector<AffPlusPtr> &affs);

	//--observers
	std::string toString();

	//-----------mutators
	void addNewlyFittedAffordance(const AffordanceState &aff);
	void addNewlyFittedAffordance(const AffordancePlusState &affPlus);
	void deleteAffordance(const AffordanceState &aff);
	void updateTrackedAffordance(const AffordanceState &aff);
	void updateTrackedAffordancePlus(const AffordancePlusState &aff);

private:
	void handleCollectionMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
							 const drc::affordance_collection_t *collection);
	void handlePlusCollectionMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
							 const drc::affordance_plus_collection_t *collection);


};

std::ostream& operator<<( std::ostream& out, AffordanceUpWrapper& other );

} /* namespace affordance */
#endif /* AFFORDANCEUPWRAPPER_H_ */
