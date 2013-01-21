#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <string>
#include <vector>  
#include <boost/unordered_map.hpp>

#include "Constraint.h"
#include "AffordanceRelation.h"

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
    std::string _name;
    std::vector<boost::shared_ptr<Constraint> > _constraints;
    ConstraintType _constraintType;
    AffRelationPtr _affordanceRelation;

    //-------Constructors--
  public:
    Constraint(const std::string &name, ConstraintType constraintType);
    Constraint(const std::string &name, AffRelationPtr affordanceRelation);
    
    //Accessors
    std::string getName() { return _name; };
    void setName(std::string name) { _name = name; };
    ConstraintType getConstraintType() { return _constraintType; };
    
    //Available for non-ATOMIC constraints only
    void getConstraints(std::vector<
			boost::shared_ptr<Constraint> > &constraints);

    void addConstraint(boost::shared_ptr<Constraint> constraint);
    
    //Available for ATOMIC constraints only
    AffRelationPtr getAffordanceRelation();
    void setAffordanceRelation(AffRelationPtr affordanceRelation);
  }; //class Constraint

  typedef boost::shared_ptr<Constraint> ConstraintPtr;
  typedef boost::shared_ptr<const Constraint> ConstraintConstPtr;
} //namespace action_authoring


#endif //CONSTRAINT_H
