#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <string>
#include <vector>  
#include <boost/unordered_map.hpp>

#include "ConstraintMacro.h"
#include "AtomicConstraint.h"

namespace action_authoring {

  class InvalidMethodCallForContraintTypeException : public std::runtime_error {
  public: InvalidMethodCallForContraintTypeException(const std::string &msg) : std::runtime_error(msg){};
  };
  
  /**todo: add comment / description*/
  class ConstraintMacro 
  {
    
    //----------enumerations
  public:
    typedef enum 
    {
      ATOMIC, //todo: add small comment
      SEQUENTIAL, //to each of 
      UNORDERED} //these lines.
    ConstraintMacroType;
    
  //------------fields--------
  private:
    std::string _name;
    std::vector<boost::shared_ptr<ConstraintMacro> > _constraints;
    ConstraintMacroType _constraintType;
    AtomicConstraintPtr _atomicConstraint;

    //-------Constructors--
  public:
    ConstraintMacro(const std::string &name, ConstraintMacroType constraintType);
    ConstraintMacro(const std::string &name, AtomicConstraintPtr atomicConstraint);
    
    //Accessors
    std::string getName() { return _name; };
    void setName(std::string name) { _name = name; };
    ConstraintMacroType getConstraintMacroType() { return _constraintType; };
    
    //Available for non-ATOMIC constraints only
    void getConstraintMacros(std::vector<
			boost::shared_ptr<ConstraintMacro> > &constraints);

    void addConstraintMacro(boost::shared_ptr<ConstraintMacro> constraint);
    
    //Available for ATOMIC constraints only
    AtomicConstraintPtr getAtomicConstraint();
    void setAtomicConstraint(AtomicConstraintPtr atomicConstraint);
  }; //class ConstraintMacro

  typedef boost::shared_ptr<ConstraintMacro> ConstraintMacroPtr;
  typedef boost::shared_ptr<const ConstraintMacro> ConstraintMacroConstPtr;
} //namespace action_authoring


#endif //CONSTRAINT_H
