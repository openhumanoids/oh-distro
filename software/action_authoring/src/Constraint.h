#pragma once
#include <string>
#include <vector>  
#include "Constraint.h"
#include "AffordanceRelation.h"
#include <boost/unordered_map.hpp>

namespace action_authoring {

  class InvalidMethodCallForContraintTypeException : public std::runtime_error {
      public: InvalidMethodCallForContraintTypeException(const std::string &msg) : std::runtime_error(msg){};
    };
  
  class Constraint {
   public:
    typedef enum {
      ATOMIC,
      SEQUENTIAL,
      UNORDERED} ConstraintType;

   protected:
    char* m_name;
    std::vector<Constraint*> m_constraints;
    ConstraintType m_constraintType;
    AffordanceRelation* m_affordanceRelation;

   public:
    // Constructors
    Constraint(char* name, ConstraintType constraintType);
    Constraint(char* name, AffordanceRelation* affordanceRelation);

    //Accessors
    char* getName() { return m_name; };
    ConstraintType getConstraintType() { return m_constraintType; };

    //Available for non ATOMIC constraints only
    std::vector<Constraint*> getConstraints();
    void addConstraint(Constraint* constraint);

    //Available for ATOMIC constraints only
    AffordanceRelation* getAffordanceRelation();
  };
}
