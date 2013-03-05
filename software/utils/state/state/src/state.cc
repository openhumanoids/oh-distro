/**
 * @file    state.cc
 * @author  Thomas M. Howard (tmhoward@csail.mit.edu)
 * @version 1.0
 *
 * @section LICENSE
 *
 * TBD
 *
 * @section DESCRIPTION
 *
 * The implementation of a base class for the state of an object
 */

#include "state/state.h"

using namespace std;
using namespace state;

State::
State( string id,
        unsigned long long time ) : _id( id ),
                                    _time( time ){

}

State::
~State() {

}

State::
State( const State& other ) : _id( other._id ),
                              _time( other._time ){

}

State&
State::
operator=( const State& other ) {
  _id = other._id;
  _time = other._time;
  return (*this);
}

void
State::
set_id( string id ){
  _id = id;
  return;
}

void
State::
set_time( unsigned long long time ){
  _time = time;
  return;
}

string 
State::
id( void )const{
  return _id;
}

unsigned long long
State::
time( void )const{
  return _time;
}

namespace state {
  ostream&
  operator<<( ostream& out,
              const State& other ) {
    out << "id:{" << other.id() << "} time:{" << other.time() << "}";
    return out;
  }

}
