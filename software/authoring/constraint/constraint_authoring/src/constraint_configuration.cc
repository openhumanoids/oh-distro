#include "authoring/constraint_configuration.h"

using namespace std;
using namespace drc;
using namespace state;
using namespace authoring;

Constraint_Configuration::
Constraint_Configuration( const std::string& id,
                          bool active,
                          double start,
                          double end,
                          const State_GFE& q ) : Constraint( id, active, start, end ),
                                                  _q( q ){

}

Constraint_Configuration::
~Constraint_Configuration() {

}

Constraint_Configuration::
Constraint_Configuration( const Constraint_Configuration& other ) : Constraint( other ) {

}

Constraint_Configuration&
Constraint_Configuration::
operator=( const Constraint_Configuration& other ) {
  _id = other._id;
  _active = other._active;
  _start = other._start;
  _end = other._end;
  _q = other._q;
  return (*this);
}

void
Constraint_Configuration::
to_xml( ofstream& out,
        unsigned int indent )const{
  return;
}

void 
Constraint_Configuration::
add_to_drc_action_sequence_t( action_sequence_t& actionSequence ){
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Constraint_Configuration& other ) {
    return out;
  }

}
