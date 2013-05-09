#include "authoring/constraint.h"

using namespace std;
using namespace drc;
using namespace authoring;

Constraint::
Constraint( const string& id,
            bool active,
            double start,
            double end ) : _id( id ),
                            _active( active ),
                            _start( start ),
                            _end( end ) {

}

Constraint::
~Constraint() {

}

Constraint::
Constraint( const Constraint& other ) : _id( other._id ),
                                        _active( other._active ),
                                        _start( other._start ),
                                        _end( other._end ){

}

Constraint&
Constraint::
operator=( const Constraint& other ) {
  _id = other._id;
  _active = other._active;
  _start = other._start;
  _end = other._end;
  return (*this);
}

string
Constraint::
constraint_type_t_to_std_string( constraint_type_t constraintType ){
  switch( constraintType ){
  case ( CONSTRAINT_TASK_SPACE_REGION_TYPE ):
    return "TASK_SPACE_REGION";
    break;
  case ( CONSTRAINT_CONFIGURATION_TYPE ):
    return "CONFIGURATION";
    break;
  case ( CONSTRAINT_UNKNOWN_TYPE ):
  default:
    return "UNKNOWN";
    break;
  }
}

void
Constraint::
add_to_drc_action_sequence_t( action_sequence_t& actionSequence ){
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Constraint& other ) {
    out << "id: " << other.id() << " ";
    if( other.active() ){
      out << "active: T ";
    } else {
      out << "active: F ";
    }
    out << "start: " << other.start() << " ";
    out << "end: " << other.end() << " ";
    return out;
  }

}
