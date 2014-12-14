#ifndef AUTHORING_CONSTRAINT_H
#define AUTHORING_CONSTRAINT_H

#include <iostream>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <libxml/tree.h>
#include <affordance/AffordanceState.h>

namespace authoring {
  typedef enum {
    CONSTRAINT_UNKNOWN_TYPE,
    CONSTRAINT_TASK_SPACE_REGION_TYPE,
    CONSTRAINT_CONFIGURATION_TYPE,
    NUM_CONSTRAINT_TYPES 
  } constraint_type_t;

  class Constraint {
  public:
    Constraint( const std::string& id = "NA", bool active = true, bool visible = true, double start = 0.1, double end = 1.0, const std::string& metadata = "NA" );
    ~Constraint();
    Constraint( const Constraint& other );
    Constraint& operator=( const Constraint& other );
 
    static std::string constraint_type_t_to_std_string( constraint_type_t constraintType );

    virtual void from_xml( xmlNodePtr root ) = 0;
    virtual void to_xml( std::ofstream& out, unsigned int indent = 0 )const = 0;

    virtual void add_to_drc_action_sequence_t( drc::action_sequence_t& actionSequence, std::vector< affordance::AffordanceState >& affordanceCollection );

    virtual inline constraint_type_t type( void )const{ return CONSTRAINT_UNKNOWN_TYPE; };
    virtual inline std::string description( void )const{ return "NA"; };
    inline std::string& id( void ){ return _id; };
    inline const std::string& id( void )const{ return _id; };
    inline bool& active( void ){ return _active; };
    inline const bool& active( void )const{ return _active; };
    inline bool& visible( void ){ return _visible; };
    inline const bool& visible( void )const{ return _visible; };
    inline double& start( void ){ return _start; };
    inline const double& start( void )const{ return _start; };
    inline double& end( void ){ return _end; };
    inline const double& end( void )const{ return _end; };
    inline std::string& metadata( void ){ return _metadata; };
    inline const std::string& metadata( void )const{ return _metadata; };

  protected:
    std::string _id;
    bool _active;
    bool _visible;
    double _start;
    double _end;
    std::string _metadata;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Constraint& other );
}

#endif /* AUTHORING_CONSTRAINT_H */
