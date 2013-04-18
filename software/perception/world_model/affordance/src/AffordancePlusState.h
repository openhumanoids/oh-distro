/*
 * AffordancePlusState.h
 *
 *  Created on: April 3, 2013
 *      Author: mfleder
 */

#ifndef AFFORDANCE_PLUS_STATE_H
#define AFFORDANCE_PLUS_STATE_H

#include <affordance/AffordanceState.h>

namespace affordance
{

  /**Mutable class representing the state of an affordance*/
  class AffordancePlusState 
  {
    //------------fields
  public: 
    //mimicking lcm
    /**underlying affordance*/
    AffPtr aff;

    //points on object - relative to affordance xyz rpy
    std::vector<Eigen::Vector3f> points;

    std::vector<Eigen::Vector3i> triangles;
    
    //-----------constructor/destructor
  public:
    AffordancePlusState(void);
    AffordancePlusState(const drc::affordance_plus_t *affordancePlusMsg);
    AffordancePlusState& operator=(const AffordancePlusState& rhs);
    virtual ~AffordancePlusState();    

  private:
    void initHelper(const drc::affordance_plus_t *msg);

    //mutators
  public:
    void fromMsg(const drc::affordance_plus_t *msg);
    void clear();

    //observers
  public:
    void toMsg(drc::affordance_plus_t *affordanceMsg) const;
  }; //class AffordancePlusState
  

  typedef boost::shared_ptr<AffordancePlusState> AffPlusPtr;
  typedef boost::shared_ptr<const AffordancePlusState> AffPlusConstPtr;  

} //namespace affordance

#endif /* AFFORDANCE_PLUS_STATE_H */
