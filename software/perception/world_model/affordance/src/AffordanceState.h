/*
 * AffordanceState.h
 *
 *  Created on: Jan 13, 2013
 *      Author: mfleder
 */

#ifndef AFFORDANCESTATE_H_
#define AFFORDANCESTATE_H_

#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/unordered_map.hpp>

namespace affordance
{
	class ArgumentException : public std::runtime_error {public: ArgumentException(const std::string &msg) : std::runtime_error(msg){}};
	class InvalidOtdfID : public std::runtime_error { public: InvalidOtdfID(const std::string &msg) : std::runtime_error(msg){}};

	class AffordanceState
	{
	//------------fields
	public:
		enum OTDF_TYPE  {CYLINDER = drc::affordance_t::CYLINDER,
						LEVER 	 = drc::affordance_t::LEVER};
	private:
		boost::unordered_map<int16_t, OTDF_TYPE> idToEnum;

	public: //should make get / private set methods for these
		//mimicking lcm
		int64_t    _map_utime;
		int32_t    _map_id;

		/**which object in the scene?*/
		int32_t    _object_id;

		/**type of object*/
		OTDF_TYPE  _otdf_id;

		/**informal name for the affordance*/
		std::string _name;

		/**{name --> value} maps*/
		boost::unordered_map<std::string, double> _params, //geometrical properties
												  _states;
		std::vector< int32_t > _ptinds;

	//-----------constructor/destructor
	public:
		AffordanceState(const drc::affordance_t *affordanceMsg);
		AffordanceState(const AffordanceState &other);
		AffordanceState& operator=( const AffordanceState& rhs );

		void initHelper(const drc::affordance_t *msg);
		virtual ~AffordanceState();

		//methods
	public:
		void toMsg(drc::affordance_t *affordanceMsg) const;
	};

} //namespace affordance

#endif /* AFFORDANCESTATE_H_ */
