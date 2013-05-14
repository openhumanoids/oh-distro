#include "authoring/constraint_sequence.h"

using namespace std;
using namespace authoring;

Constraint_Sequence::
Constraint_Sequence() {

}

Constraint_Sequence::
~Constraint_Sequence() {

}

Constraint_Sequence::
Constraint_Sequence( const Constraint_Sequence& other ) {

}

Constraint_Sequence&
Constraint_Sequence::
operator=( const Constraint_Sequence& other ) {

  return (*this);
}

void
Constraint_Sequence::
load( string filename ){
  return;
}

void
Constraint_Sequence::
save( string filename ){
  return;
}

namespace authoring {
  ostream&
  operator<<( ostream& out,
              const Constraint_Sequence& other ) {
    return out;
  }

}
