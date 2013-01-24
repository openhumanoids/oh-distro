/*
 * ModelState.h
 *
 *  Created on: Jan 23, 2013
 *      Author: mfleder
 */

#ifndef MODEL_STATE_H
#define MODEL_STATE_H

#include <lcmtypes/drc_lcmtypes.hpp>
#include <boost/unordered_map.hpp>
#include <Eigen/Core>
#include <kdl/frames.hpp>
#include <boost/tuple/tuple.hpp>

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
  template <class T>
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
    virtual std::string getName() const = 0;
    
    //location and appearance
    virtual Eigen::Vector3f getColor() const = 0;
    virtual void getFrame(KDL::Frame &frame) const = 0;

    //type
    virtual bool isAffordance() const = 0;
    virtual bool isManipulator() const = 0;

    //structure
    virtual bool hasChildren() const = 0; //any
    virtual bool hasParent() const = 0; //1 or more
    virtual void getChildren(std::vector<boost::shared_ptr<const T> > &children) const = 0;
    virtual void getParents(std::vector<boost::shared_ptr<const T> > &children) const = 0; 

    //copying
    virtual void getCopy(T &copy) const = 0;
  };
  
  template <class T> std::ostream& operator<<( std::ostream& out, const ModelState<T>& other );  
} //namespace affordance

#endif /* MODEL_STATE_H */
