/*
 * ManipulatorState.h
 *
 *  Created on: Jan 23, 2013
 *      Author: mfleder
 */

#ifndef MANIPULATOR_STATE_H
#define MANIPULATOR_STATE_H

#include <affordance/ToString.h>
#include <affordance/ModelState.h>
#include "urdf/model.h"

namespace affordance
{

  /**Represents that state of a manipulator*/
  class ManipulatorState : public ModelState
  {
    //-------------fields----
  private: 
    const std::string _name;
    const GlobalUID _guid;
    const boost::shared_ptr<const urdf::Link> _link;
    KDL::Frame _link_frame;
    std::string _link_name;
    std::string _contact_group_name;

    //-----------constructor/destructor
  public:
    ManipulatorState(boost::shared_ptr<const urdf::Link> link, 
                     const std::string &contact_group_name,
                     KDL::Frame link_frame,
                     const GlobalUID &uid);
    ManipulatorState(const std::string &s, const GlobalUID &uid);

    //ManipulatorState(const ManipulatorState &other); //todo
    //ManipulatorState& operator=( const ManipulatorState& rhs ); //todo
    virtual ~ManipulatorState();
    
    //-------------------observers
  public:
    //ModelState interface
    virtual GlobalUID getGlobalUniqueId() const;
    virtual std::string getName() const;

    virtual Eigen::Vector3f getColor() const;

    virtual Eigen::Vector3f getXYZ() const;
    virtual Eigen::Vector3f getRPY() const; 

    virtual bool isAffordance() const ;
    virtual bool isManipulator() const;
    virtual bool hasChildren() const; //any
    virtual bool hasParent() const; //1 or more
    virtual void getChildren(std::vector<ModelStateConstPtr> &children) const;
    virtual void getParents(std::vector<ModelStateConstPtr> &parents) const; 
    virtual void getCopy(ModelState &copy) const;
     
    //specific to this class
  public:
    boost::shared_ptr<const urdf::Link> getLink() const;
    std::string getLinkName() const { return _link_name; }
    std::string getGUIDAsString()  const;
    KDL::Frame getLinkFrame() const { return _link_frame; }
    void getCollisionContactPoints(std::vector<KDL::Frame> &pts) const;
    std::string getContactGroupName() const { return _contact_group_name; }
    // void setContactGroupName(std::string contact_group_name);

 private:
    typedef boost::shared_ptr<std::vector<boost::shared_ptr<urdf::Collision > > > CollisionGroupPtr;  
  };
  
  std::ostream& operator<<( std::ostream& out, const ManipulatorState& other );
  
  typedef boost::shared_ptr<ManipulatorState> ManipulatorStatePtr;
  typedef boost::shared_ptr<const ManipulatorState> ManipulatorStateConstPtr;
  
} //namespace affordance

#endif /* MANIPULATOR_STATE_H */
