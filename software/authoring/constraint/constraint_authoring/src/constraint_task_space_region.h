#ifndef AUTHORING_CONSTRAINT_TASK_SPACE_REGION_H
#define AUTHORING_CONSTRAINT_TASK_SPACE_REGION_H

#include <iostream>
#include <vector>
#include <kdl/frames.hpp>

#include <urdf/model.h>
#include <affordance/AffordanceState.h>

#include "authoring/constraint.h"

namespace authoring {

  typedef enum {
    CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE,
    CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE,
    CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE,
    CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE,
    CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE,
    CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE,
    NUM_CONSTRAINT_TASK_SPACE_REGION_RANGES
  } range_index_t;

  typedef enum {
    CONSTRAINT_TASK_SPACE_REGION_WITHIN_REGION_CONTACT_TYPE,
    CONSTRAINT_TASK_SPACE_REGION_SUPPORTED_WITHIN_REGION_CONTACT_TYPE,
    NUM_CONSTRAINT_TASK_SPACE_REGION_CONTACT_TYPES
  } contact_type_t;

  class Constraint_Task_Space_Region: public Constraint {
  public:
    Constraint_Task_Space_Region( const std::string& id = "N/A", bool active = true, double start = 0.1, double end = 1.0, const std::string& parent = std::string( "N/A" ), affordance::AffordanceState* child = NULL, contact_type_t contactType = CONSTRAINT_TASK_SPACE_REGION_WITHIN_REGION_CONTACT_TYPE );
    ~Constraint_Task_Space_Region();
    Constraint_Task_Space_Region( const Constraint_Task_Space_Region& other );
    Constraint_Task_Space_Region& operator=( const Constraint_Task_Space_Region& other );

    virtual void to_xml( std::ofstream& out, unsigned int indent = 0 )const;

    virtual void add_to_drc_action_sequence_t( drc::action_sequence_t& actionSequence );    
    
    static std::string contact_type_t_to_std_string( contact_type_t contactType );

    virtual inline constraint_type_t type( void )const{ return CONSTRAINT_TASK_SPACE_REGION_TYPE; };
    virtual std::string description( void )const;
    inline contact_type_t& contact_type( void ) { return _contact_type; }
    inline const contact_type_t& contact_type( void )const{ return _contact_type; };
    inline std::vector< std::pair< bool, double > >& ranges( void ){ return _ranges; };
    inline const std::vector< std::pair< bool, double > >& ranges( void )const{ return _ranges; };
    inline std::vector< std::string >& parents( void ){ return _parents; };
    inline const std::vector< std::string >& parents( void )const{ return _parents; };
    inline affordance::AffordanceState*& child( void ){ return _child; };
    inline const affordance::AffordanceState* child( void )const{ return _child; };
    inline KDL::Frame& offset( void ){ return _offset; };
    inline const KDL::Frame& offset( void )const{ return _offset; };

  protected:
    contact_type_t _contact_type;
    std::vector< std::pair< bool, double > > _ranges;
    std::vector< std::string > _parents;
    affordance::AffordanceState* _child;
    KDL::Frame _offset;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Constraint_Task_Space_Region& other );
}

#endif /* AUTHORING_CONSTRAINT_TASK_SPACE_REGION_H */

