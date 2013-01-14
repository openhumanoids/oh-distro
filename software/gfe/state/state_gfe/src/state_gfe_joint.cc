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
                  double measuredEffort ) : _id( id ),
                                            _time( time ),
                                            _position( position ),
                                            _velocity( velocity ),
                                            _measured_effort( measuredEffort ) {

}

State_GFE_Joint::
~State_GFE_Joint() {

}

State_GFE_Joint::
State_GFE_Joint( const State_GFE_Joint& other ) : _id( other._id ),
                                                  _time( other._time ),
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

void
State_GFE_Joint::
set_id( string id ){
  _id = id;
  return;
}

void
State_GFE_Joint::
set_time( unsigned long long time ){
  _time = time;
  return;
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

string
State_GFE_Joint::
id( void )const{
  return _id;
}

unsigned long long
State_GFE_Joint::
time( void )const{
  return _time;
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
