/*
 * ManipulatorState.h
 *
 *  Created on: Jan 23, 2013
 *      Author: mfleder
 */

#ifndef MANIPULATOR_STATE_H
#define MANIPULATOR_STATE_H

#include "affordance/ModelState.h"

namespace affordance
{

  /**Represents that state of a manipulator*/
  class ManipulatorState : public ModelState
  {
    //-------------fields----
  private: 

    //-----------constructor/destructor
  public:
    virtual ManipulatorState(const ManipulatorState &other) = 0;
    virtual ManipulatorState& operator=( const ManipulatorState& rhs ) = 0;
    virtual ~ManipulatorState() = 0;
    
    //-------------------observers
  public:
    //interface
    virtual GlobalUID getGlobalUniqueId() const;
    virtual std::string getName() const;
    virtual Eigen::Vector3f getColor() const;
    virtual void getFrame(KDL::Frame &frame) const;

  };
  
  std::ostream& operator<<( std::ostream& out, const ManipulatorState& other );
  
  typedef boost::shared_ptr<ManipulatorState> ManipulatorStatePtr;
  typedef boost::shared_ptr<const ManipulatorState> ManipulatorStateConstPtr;
  
} //namespace affordance

#endif /* MANIPULATOR_STATE_H */
