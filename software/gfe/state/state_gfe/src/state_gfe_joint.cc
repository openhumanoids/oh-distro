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
                  double measuredEffort ) : State( id, time ),
                                            _position( position ),
                                            _velocity( velocity ),
                                            _measured_effort( measuredEffort ) {

}

State_GFE_Joint::
~State_GFE_Joint() {

}

State_GFE_Joint::
State_GFE_Joint( const State_GFE_Joint& other ) : State( other ),
                                                  _position( other._position ),
                                                  _velocity( other._velocity ),
                                                  _measured_effort( other._measured_effort ){
  
}

State_GFE_Joint&
State_GFE_Joint::
operator=( const State_GFE_Joint& other ){
  _id = other._id;
  _time = other._time;
  _position = other._position;
  _velocity = other._velocity;
  _measured_effort = other._measured_effort;
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
  state.set_measured_effort( first.measured_effort() + ifactor * ( second.measured_effort() - first.measured_effort() ) );
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
set_measured_effort( double measuredEffort ){
  _measured_effort = measuredEffort;
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
measured_effort( void )const{
  return _measured_effort;
}

namespace state {
  ostream&
  operator<<( ostream& out,
              const State_GFE_Joint& other ){
    out << "id:{" << other.id() << "} ";
    out << "time:{" << other.time() << "} ";
    out << "pos:{" << other.position() << "} ";
    out << "vel:{" << other.velocity() << "} ";
    out << "me:{" << other.measured_effort() << "}";
    return out;
  }
}
