/**
 * @file    state_gfe_arm.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a class used to represent the state of the GFE Arm
 */

#include "state/state_gfe_arm.h"

using namespace std;
using namespace state;

State_GFE_Arm::
State_GFE_Arm( string jointIdPrefix ) : State(),
                                        _joints() {
  _joints[ STATE_GFE_ARM_USY_JOINT ].set_id( jointIdPrefix + "usy" );
  _joints[ STATE_GFE_ARM_SHX_JOINT ].set_id( jointIdPrefix + "shx" );
  _joints[ STATE_GFE_ARM_ELY_JOINT ].set_id( jointIdPrefix + "ely" );
  _joints[ STATE_GFE_ARM_ELX_JOINT ].set_id( jointIdPrefix + "elx" );
  _joints[ STATE_GFE_ARM_UWY_JOINT ].set_id( jointIdPrefix + "uwy" );
  _joints[ STATE_GFE_ARM_MWX_JOINT ].set_id( jointIdPrefix + "mwx" );
}

State_GFE_Arm::
~State_GFE_Arm() {

}

State_GFE_Arm::
State_GFE_Arm( const State_GFE_Arm& other ) : State( other ),
                                              _joints(){
  for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
    _joints[ i ] = other._joints[ i ];
  }
}

State_GFE_Arm&
State_GFE_Arm::
operator=( const State_GFE_Arm& other ) {
  _id = other._id;
  _time = other._time;
  for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
    _joints[ i ] = other._joints[ i ];
  }
  return (*this);
}

State_GFE_Arm
State_GFE_Arm::
interpolate( const State_GFE_Arm& first,
              const State_GFE_Arm& second,
              unsigned long long time ){
  State_GFE_Arm state;
  state.set_id( first.id() );
  state.set_time( time );
  for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
    state.joint( ( state_gfe_arm_joint_t )( i ) ) = State_GFE_Joint::interpolate( first.joint( ( state_gfe_arm_joint_t )( i ) ), second.joint( ( state_gfe_arm_joint_t )( i ) ), time );
  }
  return state;
}

void
State_GFE_Arm::
set_time( unsigned long long time ){
  _time = time;
  for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
    _joints[ i ].set_time( time ) ;
  }
}

State_GFE_Joint&
State_GFE_Arm::
joint( state_gfe_arm_joint_t joint ){
  return _joints[ joint ];
}

const State_GFE_Joint&
State_GFE_Arm::
joint( state_gfe_arm_joint_t joint )const{
  return _joints[ joint ];
} 

namespace state {
  ostream&
  operator<<( ostream& out,
              const State_GFE_Arm& other ) {
    out << endl;
    for( unsigned int i = 0; i < NUM_STATE_GFE_ARM_JOINTS; i++ ){
      out << "  " << other.joint( ( state_gfe_arm_joint_t )( i ) ) << endl;
    }
    return out;
  }

}
