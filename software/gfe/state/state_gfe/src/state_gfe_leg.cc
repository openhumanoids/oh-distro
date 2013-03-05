/**
 * @file    state_gfe_leg.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to represent the state of the GFE Leg
 */

#include "state/state_gfe_leg.h"

using namespace std;
using namespace state;

State_GFE_Leg::
State_GFE_Leg( string jointIdPrefix ) : State(),
                                        _joints(){
  _joints[ STATE_GFE_LEG_UHZ_JOINT ].set_id( jointIdPrefix + "uhz" );
  _joints[ STATE_GFE_LEG_MHX_JOINT ].set_id( jointIdPrefix + "mhx" );
  _joints[ STATE_GFE_LEG_LHY_JOINT ].set_id( jointIdPrefix + "lhy" );
  _joints[ STATE_GFE_LEG_KNY_JOINT ].set_id( jointIdPrefix + "kny" );
  _joints[ STATE_GFE_LEG_UAY_JOINT ].set_id( jointIdPrefix + "uay" );
  _joints[ STATE_GFE_LEG_LAX_JOINT ].set_id( jointIdPrefix + "lax" );
}

State_GFE_Leg::
~State_GFE_Leg() {

}

State_GFE_Leg::
State_GFE_Leg( const State_GFE_Leg& other ) : State( other ),
                                              _joints(){
  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    _joints[ i ] = other._joints[ i ];
  }
}

State_GFE_Leg&
State_GFE_Leg::
operator=( const State_GFE_Leg& other ) {
  _id = other._id;
  _time = other._time;
  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    _joints[ i ] = other._joints[ i ];
  }
  return (*this);
}

State_GFE_Leg
State_GFE_Leg::
interpolate( const State_GFE_Leg& first,
              const State_GFE_Leg& second,
              unsigned long long time ){
  State_GFE_Leg state;
  state.set_id( first.id() );
  state.set_time( time );
  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    state.joint( ( state_gfe_leg_joint_t )( i ) ) = State_GFE_Joint::interpolate( first.joint( ( state_gfe_leg_joint_t )( i ) ), second.joint( ( state_gfe_leg_joint_t )( i ) ), time );
  }
  return state;
}

void
State_GFE_Leg::
set_time( unsigned long long time ){
  _time = time;
  for( unsigned int i = 0; i < NUM_STATE_GFE_LEG_JOINTS; i++ ){
    _joints[ i ].set_time( time );
  }
}

State_GFE_Joint& 
State_GFE_Leg::
joint( state_gfe_leg_joint_t joint ){
  return _joints[ joint ];
}

const State_GFE_Joint& 
State_GFE_Leg::
joint( state_gfe_leg_joint_t joint )const{
  return _joints[ joint ];
}

namespace state {
  ostream&
  operator<<( ostream& out,
              const State_GFE_Leg& other ) {
    return out;
  }
}
