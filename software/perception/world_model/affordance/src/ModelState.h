/*
 * ModelState.h
 *
 *  Created on: Jan 23, 2013
 *      Author: mfleder
 */

#ifndef MODEL_STATE_H
#define MODEL_STATE_H

#include <iostream>
#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <kdl/frames.hpp>
#include <stdexcept>
#include <vector>
#include <affordance/ToString.h>

namespace affordance
{

  class ArgumentException : public std::runtime_error {public: ArgumentException(const std::string &msg) : std::runtime_error(msg){}};
  class InvalidOtdfID : public std::runtime_error { public: InvalidOtdfID(const std::string &msg) : std::runtime_error(msg){}};
  class KeyNotFoundException : public std::runtime_error { public: KeyNotFoundException(const std::string &msg) : std::runtime_error(msg){}};
  class NotImplementedException : public std::runtime_error { public: NotImplementedException(const std::string &msg) : std::runtime_error(msg){}};

  typedef std::pair<const int32_t, const int32_t> GlobalUID;
  
  /**Base Class representing the state of a some modeled object 
     like an affordance or a manipulator.  The derived class is of type T.
     All ModelStates have a position and orientation.*/
  class ModelState
  {

    //fields:
    //not storing state in here right now
  public: 

    //-----------constructor/destructor
  public:
    //none
    
    //-------------------observers
  public:

    //--interface
    //identification
    virtual GlobalUID getGlobalUniqueId() const = 0;
    std::string getGUIDAsString() const
      {
	return ToString::toStr(getGlobalUniqueId().first) + "," 
	  + ToString::toStr(getGlobalUniqueId().second);
      };
    virtual std::string getName() const = 0;
    
    //location and appearance
    virtual Eigen::Vector3f getColor() const = 0;
    virtual Eigen::Vector3f getXYZ() const = 0;
    virtual Eigen::Vector3f getRPY() const = 0; 

    //type
    virtual bool isAffordance() const = 0;
    virtual bool isManipulator() const = 0;

    //structure
    virtual bool hasChildren() const = 0; //any
    virtual bool hasParent() const = 0; //1 or more
    virtual void getChildren(std::vector<boost::shared_ptr<const ModelState> > &children) const = 0;
    virtual void getParents(std::vector<boost::shared_ptr<const ModelState> > &parents) const = 0; 

    //copying
    virtual void getCopy(ModelState &copy) const = 0;


    //==========derived properties
    //can't put this in a separate cpp file: http://www.parashift.com/c++-faq-lite/separate-template-fn-defn-from-decl.html
    virtual KDL::Frame getFrame() const //derived method
    {
      Eigen::Vector3f xyz = getXYZ();
      Eigen::Vector3f rpy = getRPY();
  
      //frame will be used by everything -- only compute once
      return KDL::Frame(KDL::Rotation::RPY(rpy[0],rpy[1],rpy[2]),
			KDL::Vector(xyz[0], xyz[1], xyz[2]));
    }

    virtual KDL::Rotation getRotation() const
      {
	Eigen::Vector3f rpy = getRPY();
	return KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]);
      }

    virtual Eigen::Vector4f getQuaternion() const
      {
	double x,y,z,w;
	getRotation().GetQuaternion(x,y,z,w);
	return Eigen::Vector4f(x,y,z,w);
      }

    //useful methods
    static Eigen::Vector4f extractQuaternion(const KDL::Frame &f) 
    {
      	double x,y,z,w;
	f.M.GetQuaternion(x,y,z,w);
	return Eigen::Vector4f(x,y,z,w);
    }

    static Eigen::Vector3f extractXYZ(const KDL::Frame &f) 
    {
      return Eigen::Vector3f(f.p.x(),
			     f.p.y(),
			     f.p.z());
    }
  };
  
  std::ostream& operator<<( std::ostream& out, const ModelState& other );  
  typedef boost::shared_ptr<ModelState> ModelStatePtr;
  typedef boost::shared_ptr<const ModelState> ModelStateConstPtr;  
} //namespace affordance

#endif /* MODEL_STATE_H */
