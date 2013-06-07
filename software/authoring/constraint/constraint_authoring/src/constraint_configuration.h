#ifndef AUTHORING_CONSTRAINT_CONFIGURATION_H
#define AUTHORING_CONSTRAINT_CONFIGURATION_H

#include <iostream>

#include <state/state_gfe.h>

#include "authoring/constraint.h"

namespace authoring {
  class Constraint_Configuration: public Constraint {
  public:
    Constraint_Configuration( const std::string& id = "N/A", bool active = true, double start = 0.0, double end = 0.0, const state::State_GFE& q = state::State_GFE() );
    ~Constraint_Configuration();
    Constraint_Configuration( const Constraint_Configuration& other );
    Constraint_Configuration& operator=( const Constraint_Configuration& other );

    virtual void from_xml( xmlNodePtr root );
    virtual void to_xml( std::ofstream& out, unsigned int indent = 0 )const;

    virtual void add_to_drc_action_sequence_t( drc::action_sequence_t& actionSequence );

    virtual inline constraint_type_t type( void )const{ return CONSTRAINT_CONFIGURATION_TYPE; };

  protected:
    state::State_GFE _q;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Constraint_Configuration& other );
}

#endif /* AUTHORING_CONSTRAINT_CONFIGURATION_H */
