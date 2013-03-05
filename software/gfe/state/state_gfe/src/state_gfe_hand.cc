#include "state/state_gfe_hand.h"

using namespace std;
using namespace state;

State_GFE_Hand::
State_GFE_Hand( string jointIdPrefix ) : State() {
  _fingers[ STATE_GFE_HAND_FINGER_0 ].set_id( jointIdPrefix + "f0_" );
  _fingers[ STATE_GFE_HAND_FINGER_1 ].set_id( jointIdPrefix + "f1_" );
  _fingers[ STATE_GFE_HAND_FINGER_2 ].set_id( jointIdPrefix + "f2_" );
  _fingers[ STATE_GFE_HAND_FINGER_3 ].set_id( jointIdPrefix + "f3_" );
}

State_GFE_Hand::
~State_GFE_Hand() {

}

State_GFE_Hand::
State_GFE_Hand( const State_GFE_Hand& other ) : State( other ),
                                                _fingers() {
  for( unsigned int i = 0; i < NUM_STATE_GFE_HAND_FINGERS; i++ ){
    _fingers[ i ] = other._fingers[ i ];
  }
}

State_GFE_Hand&
State_GFE_Hand::
operator=( const State_GFE_Hand& other ) {
  _id = other._id;
  _time = other._time;
  for( unsigned int i = 0; i < NUM_STATE_GFE_HAND_FINGERS; i++ ){
    _fingers[ i ] = other._fingers[ i ];
  }
  return (*this);
}

State_GFE_Hand
State_GFE_Hand::
interpolate( const State_GFE_Hand& first,
              const State_GFE_Hand& second,
              unsigned long long time ){
  State_GFE_Hand state;
  state.set_id( first.id() );
  state.set_time( time );
  for( unsigned int i = 0; i < NUM_STATE_GFE_HAND_FINGERS; i++ ){
    state.finger( ( state_gfe_finger_t )( i ) ) = State_GFE_Finger::interpolate( first.finger( ( state_gfe_finger_t )( i ) ), second.finger( ( state_gfe_finger_t )( i ) ), time );
  }
  return state;
}

void
State_GFE_Hand::
set_time( unsigned long long time ){
  _time = time;
  for( unsigned int i = 0; i < NUM_STATE_GFE_HAND_FINGERS; i++ ){
    _fingers[ i ].set_time( time ) ;
  }
}

State_GFE_Finger&
State_GFE_Hand::
finger( state_gfe_finger_t finger ){
  return _fingers[ finger ];
}

const State_GFE_Finger&
State_GFE_Hand::
finger( state_gfe_finger_t finger )const{
  return _fingers[ finger ];
} 

namespace state {
  ostream&
  operator<<( ostream& out,
              const State_GFE_Hand& other ) {
    out << endl;
    for( unsigned int i = 0; i < NUM_STATE_GFE_HAND_FINGERS; i++ ){
      out << "  " << other.finger( ( state_gfe_finger_t )( i ) ) << endl;
    }
    return out;
  }

}
