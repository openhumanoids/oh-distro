#include <state/state_gfe.h>

using namespace std;
using namespace KDL;
using namespace drc;
using namespace state;

State_GFE_Joint::
State_GFE_Joint( string id,
                  unsigned long long time,
                  double position,
                  double velocity,
                  double effort ) : State( id, time ),
                                            _position( position ),
                                            _velocity( velocity ),
                                            _effort( effort ) {

}

State_GFE_Joint::
~State_GFE_Joint() {

}

State_GFE_Joint::
State_GFE_Joint( const State_GFE_Joint& other ) : State( other ),
                                                  _position( other._position ),
                                                  _velocity( other._velocity ),
                                                  _effort( other._effort ){
  
}

State_GFE_Joint&
State_GFE_Joint::
operator=( const State_GFE_Joint& other ){
  _id = other._id;
  _time = other._time;
  _position = other._position;
  _velocity = other._velocity;
  _effort = other._effort;
  return ( *this );
}

State_GFE_Joint 
State_GFE_Joint::
interpolate( const State_GFE_Joint& first, 
              const State_GFE_Joint& second, 
              unsigned long long time ){
  State_GFE_Joint state( first.id(), time );
  double ifactor = ( double )( time - first.time() ) / ( double )( second.time() - first.time() );
  state.set_position( first.position() + ifactor * ( second.position() - first.position() ) );
  state.set_velocity( first.velocity() + ifactor * ( second.velocity() - first.velocity() ) );
  state.set_effort( first.effort() + ifactor * ( second.effort() - first.effort() ) );
  return state;
}


void
State_GFE_Joint::
set_position( double position ){
  _position = position;
  return;
}

void
State_GFE_Joint::
set_velocity( double velocity ){
  _velocity = velocity;
  return;
}

void
State_GFE_Joint::
set_effort( double effort ){
  _effort = effort;
  return;
}

double
State_GFE_Joint::
position( void )const{
  return _position;
}

double
State_GFE_Joint::
velocity( void )const{
  return _velocity;
}

double
State_GFE_Joint::
effort( void )const{
  return _effort;
}

namespace state {
  ostream&
  operator<<( ostream& out,
              const State_GFE_Joint& other ){
    out << "id:{" << other.id() << "} ";
    out << "time:{" << other.time() << "} ";
    out << "pos:{" << other.position() << "} ";
    out << "vel:{" << other.velocity() << "} ";
    out << "me:{" << other.effort() << "}";
    return out;
  }
}
