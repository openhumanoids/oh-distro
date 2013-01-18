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
  
  /**todo: add comment / description*/
  class Constraint 
  {
    
    //----------enumerations
  public:
    typedef enum 
    {
      ATOMIC, //todo: add small comment
      SEQUENTIAL, //to each of 
      UNORDERED} //these lines.
    ConstraintType;
    
  //------------fields--------
  protected:
    std::string m_name;
    std::vector<boost::shared_ptr<Constraint> > m_constraints;
    ConstraintType m_constraintType;
    AffRelationConstPtr m_affordanceRelation;

    //-------Constructors--
  public:
    Constraint(const std::string &name, const ConstraintType &constraintType);
    Constraint(const std::string &name, AffRelationConstPtr affordanceRelation);
    
    //Accessors
    std::string getName() const { return m_name; };
    ConstraintType getConstraintType() const { return m_constraintType; };
    
    //Available for non-ATOMIC constraints only
    std::vector<boost::shared_ptr<Constraint> > getConstraints() const;
    void addConstraint(boost::shared_ptr<const Constraint> constraint);
    
    //Available for ATOMIC constraints only
    AfforRelationConstPr getAffordance_Relation() const;
  }; //class Constraint

} //namespace action_authoring
