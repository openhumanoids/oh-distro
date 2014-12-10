#include "state/state_gfe_finger.h"

using namespace std;
using namespace state;

State_GFE_Finger::
State_GFE_Finger( string jointIdPrefix ) : State() {
  _joints[ STATE_GFE_FINGER_JOINT_0 ].set_id( jointIdPrefix + "j0" );
  _joints[ STATE_GFE_FINGER_JOINT_1 ].set_id( jointIdPrefix + "j1" );
  _joints[ STATE_GFE_FINGER_JOINT_2 ].set_id( jointIdPrefix + "j2" );
}

State_GFE_Finger::
~State_GFE_Finger() {

}

State_GFE_Finger::
State_GFE_Finger( const State_GFE_Finger& other ) : State( other ),
                                                    _joints(){
  for( unsigned int i = 0; i < NUM_STATE_GFE_FINGER_JOINTS; i++ ){
    _joints[ i ] = other._joints[ i ];
  }
}

State_GFE_Finger&
State_GFE_Finger::
operator=( const State_GFE_Finger& other ) {
  _id = other._id;
  _time = other._time;
  for( unsigned int i = 0; i < NUM_STATE_GFE_FINGER_JOINTS; i++ ){
    _joints[ i ] = other._joints[ i ];
  }
  return (*this);
}

State_GFE_Finger
State_GFE_Finger::
interpolate( const State_GFE_Finger& first,
              const State_GFE_Finger& second,
              unsigned long long time ){
  State_GFE_Finger state;
  state.set_id( first.id() );
  state.set_time( time );
  for( unsigned int i = 0; i < NUM_STATE_GFE_FINGER_JOINTS; i++ ){
    state.joint( ( state_gfe_finger_joint_t )( i ) ) = State_GFE_Joint::interpolate( first.joint( ( state_gfe_finger_joint_t )( i ) ), second.joint( ( state_gfe_finger_joint_t )( i ) ), time );
  }
  return state;
}

void
State_GFE_Finger::
set_id( string id ){
  _id = id;
  _joints[ STATE_GFE_FINGER_JOINT_0 ].set_id( id + "j0" );
  _joints[ STATE_GFE_FINGER_JOINT_1 ].set_id( id + "j1" );
  _joints[ STATE_GFE_FINGER_JOINT_2 ].set_id( id + "j2" );
  return;
}

void
State_GFE_Finger::
set_time( unsigned long long time ){
  _time = time;
  for( unsigned int i = 0; i < NUM_STATE_GFE_FINGER_JOINTS; i++ ){
    _joints[ i ].set_time( time ) ;
  }
}

State_GFE_Joint&
State_GFE_Finger::
joint( state_gfe_finger_joint_t joint ){
  return _joints[ joint ];
}

const State_GFE_Joint&
State_GFE_Finger::
joint( state_gfe_finger_joint_t joint )const{
  return _joints[ joint ];
}

namespace state {
  ostream&
  operator<<( ostream& out,
              const State_GFE_Finger& other ) {
    for( unsigned int i = 0; i < NUM_STATE_GFE_FINGER_JOINTS; i++ ){
      out << "  " << other.joint( ( state_gfe_finger_joint_t )( i ) ) << endl;
    }
    return out;
  }
}
