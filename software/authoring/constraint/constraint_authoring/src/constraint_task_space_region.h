#ifndef AUTHORING_CONSTRAINT_TASK_SPACE_REGION_H
#define AUTHORING_CONSTRAINT_TASK_SPACE_REGION_H

#include <iostream>
#include <vector>
#include <kdl/frames.hpp>

#include <urdf/model.h>
#include <affordance/AffordanceState.h>

#include "authoring/constraint.h"

namespace authoring {
  class Constraint_Task_Space_Region: public Constraint {
  public:
    Constraint_Task_Space_Region( const std::string& id = "N/A", double start = 0.0, double end = 0.0, const std::pair< boost::shared_ptr< urdf::Link >, std::string >& parent = std::pair< boost::shared_ptr< urdf::Link >, std::string >(), affordance::AffordanceState* child = NULL );
    ~Constraint_Task_Space_Region();
    Constraint_Task_Space_Region( const Constraint_Task_Space_Region& other );
    Constraint_Task_Space_Region& operator=( const Constraint_Task_Space_Region& other );

    virtual void add_to_drc_action_sequence_t( drc::action_sequence_t& actionSequence );    

    virtual inline constraint_type_t type( void ){ return CONSTRAINT_TASK_SPACE_REGION_TYPE; };
    inline std::vector< std::pair< double, double > >& ranges( void ){ return _ranges; };
    inline const std::vector< std::pair< double, double > >& ranges( void )const{ return _ranges; };
    inline std::pair< boost::shared_ptr< urdf::Link >, std::string >& parent( void ){ return _parent; };
    inline const std::pair< boost::shared_ptr< urdf::Link >, std::string >& parent( void )const{ return _parent; };
    inline affordance::AffordanceState*& child( void ){ return _child; };
    inline const affordance::AffordanceState* child( void )const{ return _child; };
    inline KDL::Frame& parent_to_constraint( void ){ return _parent_to_constraint; };
    inline const KDL::Frame& parent_to_constraint( void )const{ return _parent_to_constraint; };
    inline KDL::Frame& child_to_constraint( void ){ return _child_to_constraint; };
    inline const KDL::Frame& child_to_constraint( void )const{ return _child_to_constraint; };

  protected:
    std::vector< std::pair< double, double > > _ranges;
    std::pair< boost::shared_ptr< urdf::Link >, std::string > _parent;
    affordance::AffordanceState* _child;
    KDL::Frame _parent_to_constraint;
    KDL::Frame _child_to_constraint;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Constraint_Task_Space_Region& other );
}

#endif /* AUTHORING_CONSTRAINT_TASK_SPACE_REGION_H */
