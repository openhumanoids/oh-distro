/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

// Modified from ROS's URDF_DOM library Written by Josh Faust and Wim Meeussen

#ifndef OTDF_INTERFACE_JOINT_H
#define OTDF_INTERFACE_JOINT_H

#include <string>
#include <vector>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include "expression_parsing.h"
#include "pose.h"

namespace otdf{

  
class Link;

class JointDynamics
{
public:
  JointDynamics() { this->clear(); };
  double damping;
  double friction;
  
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;

  void clear()
  {
    local_expressions.clear();
    expression_flags.clear(); 
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("0",expression);
    local_expressions.push_back(expression);// initialise local_expressions
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    damping = 0;
    friction = 0;
  };
  bool initXml(TiXmlElement* config, ParamTable_t &symbol_table);
  void update()
  {
    if(expression_flags[0]){
	this->damping = this->local_expressions[0].value();
    }
    if(expression_flags[1]){
	this->friction = this->local_expressions[1].value();
    }
  };
};

class JointLimits
{
public:
  JointLimits() { this->clear(); };
  double lower;
  double upper;
  double effort;
  double velocity;

  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;
  void clear()
  {
    local_expressions.clear();
    expression_flags.clear(); 
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression); // initialise local_expressions
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    lower = 0;
    upper = 0;
    effort = 0;
    velocity = 0;
  };
  bool initXml(TiXmlElement* config, ParamTable_t &symbol_table);
  void update()
  {
    if(expression_flags[0]){
	this->lower = this->local_expressions[0].value();
    }
    
    if(expression_flags[1]){
	this->upper = this->local_expressions[1].value();
    }
    
    if(expression_flags[2]){
	this->effort = this->local_expressions[2].value();
    }
    
    if(expression_flags[3]){
	this->velocity = this->local_expressions[3].value();
    }
  };
};

/// \brief Parameters for Joint Safety Controllers
class JointSafety
{
public:
  /// clear variables on construction
  JointSafety() { this->clear(); };

  /// 
  /// IMPORTANT:  The safety controller support is very much PR2 specific, not intended for generic usage.
  /// 
  /// Basic safety controller operation is as follows
  /// 
  /// current safety controllers will take effect on joints outside the position range below:
  ///
  /// position range: [JointSafety::soft_lower_limit  + JointLimits::velocity / JointSafety::k_position, 
  ///                  JointSafety::soft_uppper_limit - JointLimits::velocity / JointSafety::k_position]
  ///
  /// if (joint_position is outside of the position range above)
  ///     velocity_limit_min = -JointLimits::velocity + JointSafety::k_position * (joint_position - JointSafety::soft_lower_limit)
  ///     velocity_limit_max =  JointLimits::velocity + JointSafety::k_position * (joint_position - JointSafety::soft_upper_limit)
  /// else
  ///     velocity_limit_min = -JointLimits::velocity
  ///     velocity_limit_max =  JointLimits::velocity
  ///
  /// velocity range: [velocity_limit_min + JointLimits::effort / JointSafety::k_velocity,
  ///                  velocity_limit_max - JointLimits::effort / JointSafety::k_velocity]
  ///
  /// if (joint_velocity is outside of the velocity range above)
  ///     effort_limit_min = -JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_min)
  ///     effort_limit_max =  JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_max)
  /// else
  ///     effort_limit_min = -JointLimits::effort
  ///     effort_limit_max =  JointLimits::effort
  ///
  /// Final effort command sent to the joint is saturated by [effort_limit_min,effort_limit_max]
  ///
  /// Please see wiki for more details: http://www.ros.org/wiki/pr2_controller_manager/safety_limits
  /// 
  double soft_lower_limit;
  double soft_upper_limit;
  double k_position;
  double k_velocity;
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;
  
  void clear()
  {
    local_expressions.clear();
    expression_flags.clear(); 
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression); // initialise local_expressions
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);    
    soft_lower_limit = 0;
    soft_upper_limit = 0;
    k_position = 0;
    k_velocity = 0;
  };
  bool initXml(TiXmlElement* config, ParamTable_t &symbol_table);
  void update()
  {
    if(expression_flags[0]){
	this->soft_lower_limit = this->local_expressions[0].value();
    }
    
    if(expression_flags[1]){
	this->soft_upper_limit = this->local_expressions[1].value();
    }
    
    if(expression_flags[2]){
	this->k_position = this->local_expressions[2].value();
    }
    
    if(expression_flags[3]){
	this->k_velocity = this->local_expressions[3].value();
    }
  };
};

class JointCalibration
{
public:
  JointCalibration() { this->clear(); };
  double reference_position;
  boost::shared_ptr<double> rising, falling;
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;
  void clear()
  {
    local_expressions.clear();
    expression_flags.clear(); 
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression); // initialise local_expressions
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    reference_position = 0;
  };
  bool initXml(TiXmlElement* config, ParamTable_t &symbol_table);
  void update()
  {
    if(expression_flags[0]){
       double temp =this->local_expressions[0].value();
	this->rising.reset(new double(temp));	
    }
    
    if(expression_flags[1]){
      double temp =this->local_expressions[1].value();
      this->falling.reset(new double(temp));	
    }
  };
};

class JointMimic
{
public:
  JointMimic() { this->clear(); };  
  double multiplier;
  double offset;
  std::string joint_name;
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;
  void clear()
  {
    local_expressions.clear();
    expression_flags.clear(); 
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression); // initialise local_expressions
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);    
    multiplier = 0.0;
    offset = 0.0;
    joint_name.clear();
  };
  bool initXml(TiXmlElement* config, ParamTable_t &symbol_table);
  void update()
  {
    if(expression_flags[0]){
	this->multiplier = this->local_expressions[0].value();
    }
    
    if(expression_flags[1]){
	this->offset = this->local_expressions[1].value();
    }
  };
};


class Joint
{
public:

  Joint() { this->clear(); };

  std::string name;
  
  // state (p,v) of the joint (new addition in otdf)
  double position; 
  double velocity;
  enum
  {
    UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
  } type;

  /// \brief     type_       meaning of axis_
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation axis
  ///            PRISMATIC   translation axis
  ///            FLOATING    N/A
  ///            PLANAR      plane normal axis
  ///            FIXED       N/A
  Vector3 axis;

  /// child Link element
  ///   child link frame is the same as the Joint frame
  std::string child_link_name;
  std::string child_type;
  /// parent Link element
  ///   origin specifies the transform from Parent Link to Joint Frame
  std::string parent_link_name;
  std::string parent_type;
  /// transform from Parent Link frame to Joint frame
  Pose  parent_to_joint_origin_transform;

  /// Joint Dynamics
  boost::shared_ptr<JointDynamics> dynamics;

  /// Joint Limits
  boost::shared_ptr<JointLimits> limits;

  /// Unsupported Hidden Feature
  boost::shared_ptr<JointSafety> safety;

  /// Unsupported Hidden Feature
  boost::shared_ptr<JointCalibration> calibration;

  /// Option to Mimic another Joint
  boost::shared_ptr<JointMimic> mimic;

  bool initXml(TiXmlElement* xml, ParamTable_t &symbol_table);
  void clear()
  {
    this->axis.clear();
    this->child_link_name.clear();
    this->parent_link_name.clear();
    this->parent_to_joint_origin_transform.clear();
    this->dynamics.reset(new JointDynamics);
    this->limits.reset(new JointLimits);
    this->safety.reset(new JointSafety);
    this->calibration.reset(new JointCalibration);
    this->mimic.reset(new JointMimic);
    this->type = UNKNOWN;
    this->position = 0;
    this->velocity = 0;
  };
  
  void update()
  {
    this->axis.update();
    this->parent_to_joint_origin_transform.update();
    this->dynamics->update();
    this->limits->update();
    this->safety->update();
    this->calibration->update();
    this->mimic->update();
  };
};

}

#endif
