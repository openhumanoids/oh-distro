/*
 * ManipulatorState.h
 *
 *  Created on: Jan 23, 2013
 *      Author: mfleder
 */

#ifndef MANIPULATOR_STATE_H
#define MANIPULATOR_STATE_H

#include "affordance/ModelState.h"

namespace affordance
{

  /**Represents that state of a manipulator*/
  class ManipulatorState : public ModelState<ManipulatorState>
  {
    //-------------fields----
  private: 
    std::string _name;

    //-----------constructor/destructor
  public:
    ManipulatorState(std::string name);
    ManipulatorState(const ManipulatorState &other);
    ManipulatorState& operator=( const ManipulatorState& rhs );
    virtual ~ManipulatorState();
    
    //-------------------observers
  public:
    //interface
    virtual GlobalUID getGlobalUniqueId() const;
    virtual std::string getName() const;

    virtual Eigen::Vector3f getColor() const;

    virtual Eigen::Vector3f getXYZ() const;
    virtual Eigen::Vector3f getRPY() const; 

    virtual bool isAffordance() const ;
    virtual bool isManipulator() const;
    virtual bool hasChildren() const; //any
    virtual bool hasParent() const; //1 or more
    virtual void getChildren(std::vector<boost::shared_ptr<const ManipulatorState> > &children) const;
    virtual void getParents(std::vector<boost::shared_ptr<const ManipulatorState> > &children) const;
    virtual void getParents(std::vector<boost::shared_ptr<const ModelState> > &children) const;
    virtual void getCopy(ManipulatorState &copy) const;

  };
  
  std::ostream& operator<<( std::ostream& out, const ManipulatorState& other );
  
  typedef boost::shared_ptr<ManipulatorState> ManipulatorStatePtr;
  typedef boost::shared_ptr<const ManipulatorState> ManipulatorStateConstPtr;
  
} //namespace affordance

#endif /* MANIPULATOR_STATE_H */
