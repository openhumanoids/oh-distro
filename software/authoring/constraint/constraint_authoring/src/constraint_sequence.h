#ifndef AUTHORING_CONSTRAINT_SEQUENCE_H
#define AUTHORING_CONSTRAINT_SEQUENCE_H

#include <iostream>
#include <vector>

#include <state/state_gfe.h>
#include <affordance/AffordanceState.h>

#include <authoring/constraint_task_space_region.h>

namespace authoring {
  class Constraint_Sequence {
  public:
    Constraint_Sequence( unsigned int numConstraints = 128 );
    ~Constraint_Sequence();
    Constraint_Sequence( const Constraint_Sequence& other );
    Constraint_Sequence& operator=( const Constraint_Sequence& other );

    void from_xml( const std::string& filename );
    void from_xml( xmlNodePtr root ); 
    void to_xml( const std::string& filename )const;
    void to_xml( std::ofstream& out, unsigned int indent = 0 )const;
    void to_msg( drc::action_sequence_t& msg, std::vector< affordance::AffordanceState >& affordanceCollection );
    static void print_msg( const drc::action_sequence_t& msg );

    inline std::vector< Constraint_Task_Space_Region >& constraints( void ){ return _constraints; };
    inline const std::vector< Constraint_Task_Space_Region >& constraints( void )const{ return _constraints; };
    inline state::State_GFE& q0( void ){ return _q0; };
    inline const state::State_GFE& q0( void )const{ return _q0; };

  protected:
    std::vector< Constraint_Task_Space_Region > _constraints;
    state::State_GFE _q0;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Constraint_Sequence& other );
}

#endif /* AUTHORING_CONSTRAINT_SEQUENCE_H */
