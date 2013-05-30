#ifndef AUTHORING_CONSTRAINT_H
#define AUTHORING_CONSTRAINT_H

#include <iostream>
#include <lcmtypes/drc_lcmtypes.hpp>

namespace authoring {
  typedef enum {
    CONSTRAINT_UNKNOWN_TYPE,
    CONSTRAINT_TASK_SPACE_REGION_TYPE,
    CONSTRAINT_CONFIGURATION_TYPE,
    NUM_CONSTRAINT_TYPES 
  } constraint_type_t;

  class Constraint {
  public:
    Constraint( const std::string& id = "N/A", bool active = true, double start = 0.1, double end = 1.0 );
    ~Constraint();
    Constraint( const Constraint& other );
    Constraint& operator=( const Constraint& other );
 
    static std::string constraint_type_t_to_std_string( constraint_type_t constraintType );

    virtual void add_to_drc_action_sequence_t( drc::action_sequence_t& actionSequence );

    virtual inline constraint_type_t type( void )const{ return CONSTRAINT_UNKNOWN_TYPE; };
    virtual inline std::string description( void )const{ return "N/A"; };
    inline std::string& id( void ){ return _id; };
    inline const std::string& id( void )const{ return _id; };
    inline bool& active( void ){ return _active; };
    inline const bool& active( void )const{ return _active; };
    inline double& start( void ){ return _start; };
    inline const double& start( void )const{ return _start; };
    inline double& end( void ){ return _end; };
    inline const double& end( void )const{ return _end; };

  protected:
    std::string _id;
    bool _active;
    double _start;
    double _end;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Constraint& other );
}

#endif /* AUTHORING_CONSTRAINT_H */
