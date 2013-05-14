#ifndef AUTHORING_CONSTRAINT_SEQUENCE_H
#define AUTHORING_CONSTRAINT_SEQUENCE_H

#include <iostream>
#include <vector>

#include <authoring/constraint.h>

namespace authoring {
  class Constraint_Sequence {
  public:
    Constraint_Sequence();
    ~Constraint_Sequence();
    Constraint_Sequence( const Constraint_Sequence& other );
    Constraint_Sequence& operator=( const Constraint_Sequence& other );

    void save( std::string filename );
    void load( std::string filename );

    inline std::vector< Constraint* >& constraints( void ){ return _constraints; };
    inline const std::vector< Constraint* >& constraints( void )const{ return _constraints; };

  protected:
    std::vector< Constraint* > _constraints;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Constraint_Sequence& other );
}

#endif /* AUTHORING_CONSTRAINT_SEQUENCE_H */
