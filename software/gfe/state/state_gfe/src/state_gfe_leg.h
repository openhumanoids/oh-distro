/**
 * @file    state_gfe_leg.h
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The interface for a class used to represent the state of the GFE Leg
 */

#ifndef STATE_STATE_GFE_LEG_H
#define STATE_STATE_GFE_LEG_H

#include <iostream>
#include <vector>

#include <state/state.h>
#include <state/state_gfe_joint.h>

namespace state {
  typedef enum {
    STATE_GFE_LEG_UHZ_JOINT,
    STATE_GFE_LEG_MHX_JOINT,
    STATE_GFE_LEG_LHY_JOINT,
    STATE_GFE_LEG_KNY_JOINT,
    STATE_GFE_LEG_UAY_JOINT,
    STATE_GFE_LEG_LAX_JOINT,
    NUM_STATE_GFE_LEG_JOINTS
  } state_gfe_leg_joint_t; 

  class State_GFE_Leg : public State {
  public:
    State_GFE_Leg( std::string jointIdPrefix = "l_leg_" );
    ~State_GFE_Leg();
    State_GFE_Leg( const State_GFE_Leg& other );
    State_GFE_Leg& operator=( const State_GFE_Leg& other );

    static State_GFE_Leg interpolate( const State_GFE_Leg& first, const State_GFE_Leg& second, unsigned long long time );

    virtual void set_time( unsigned long long time );

    State_GFE_Joint& joint( state_gfe_leg_joint_t joint );
    const State_GFE_Joint& joint( state_gfe_leg_joint_t joint )const;

  protected:
    State_GFE_Joint _joints[ NUM_STATE_GFE_LEG_JOINTS ];
  private:

  };
  std::ostream& operator<<( std::ostream& out, const State_GFE_Leg& other );
}

#endif /* STATE_STATE_GFE_LEG_H */
