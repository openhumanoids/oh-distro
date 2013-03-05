/**
 * @file    state_gfe_arm.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to represent the state of the GFE Arm
 */

#ifndef STATE_STATE_GFE_ARM_H
#define STATE_STATE_GFE_ARM_H

#include <iostream>
#include <vector>

#include <state/state.h>
#include <state/state_gfe_hand.h>
#include <state/state_gfe_joint.h>

namespace state {
  typedef enum {
    STATE_GFE_ARM_USY_JOINT,
    STATE_GFE_ARM_SHX_JOINT,
    STATE_GFE_ARM_ELY_JOINT,
    STATE_GFE_ARM_ELX_JOINT,
    STATE_GFE_ARM_UWY_JOINT,
    STATE_GFE_ARM_MWX_JOINT,
    NUM_STATE_GFE_ARM_JOINTS
  } state_gfe_arm_joint_t;

  class State_GFE_Arm : public State {
  public:
    State_GFE_Arm( std::string jointIdPrefix = "l_arm_" );
    ~State_GFE_Arm();
    State_GFE_Arm( const State_GFE_Arm& other );
    State_GFE_Arm& operator=( const State_GFE_Arm& other );

    static State_GFE_Arm interpolate( const State_GFE_Arm& first, const State_GFE_Arm& second, unsigned long long time );    

    virtual void set_time( unsigned long long time );

    State_GFE_Joint& joint( state_gfe_arm_joint_t joint );
    const State_GFE_Joint& joint( state_gfe_arm_joint_t joint )const;

  protected:
    State_GFE_Joint _joints[ NUM_STATE_GFE_ARM_JOINTS ];
  private:

  };
  std::ostream& operator<<( std::ostream& out, const State_GFE_Arm& other );
}

#endif /* STATE_STATE_GFE_ARM_H */
