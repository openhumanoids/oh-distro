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


  typedef std::pair<const int32_t, const int32_t> GlobalUID;
  
  /**Base Class representing the state of a some modeled object 
     like an affordance or a manipulator.
     All ModelStates have a position and orientation*/
  class ModelState
  {

    //fields:
    //not storing state in here right now
  public: 

    //-----------constructor/destructor
  public:
    //virtual ModelState(const ModelState &other) = 0;
    //virtual ModelState& operator=( const ModelState& rhs ) = 0;
    //virtual ~ModelState() = 0;
    
    //-------------------observers
  public:
    //interface
    virtual GlobalUID getGlobalUniqueId() const = 0;
    virtual std::string getName() const = 0;
    virtual void getFrame(KDL::Frame &frame) const = 0;
    virtual Eigen::Vector3f getColor() const = 0;

    //derived properties
    Eigen::Vector3f getXYZ() const; 
    Eigen::Vector3f getRPY() const; 
    Eigen::Vector4f getQuaternion() const;
  };
  
  std::ostream& operator<<( std::ostream& out, const ModelState& other );
  
  typedef boost::shared_ptr<ModelState> ModelStatePtr;
  typedef boost::shared_ptr<const ModelState> ModelStateConstPtr;
  
} //namespace affordance

#endif /* MODEL_STATE_H */
