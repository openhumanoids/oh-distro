/*********************************************************************
* Software Ligcense Agreement (BSD License)
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

/* Author: John Hsu */
// Modified from ROS's URDF_DOM library Written by John Hsu, Wim Meeussen and  Josh Faust
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <otdf_interface/exceptions.h>
#include <otdf_interface/joint.h>

namespace otdf{

bool JointDynamics::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{
  this->clear();

  // Get joint damping
  const char* damping_str = config->Attribute("damping");
  if (damping_str == NULL){
    //ROS_DEBUG("joint dynamics: no damping, defaults to 0");
    this->damping = 0;
  }
  else
  {

   this->damping  = lexicalCastOrExpressionParsing("damping", config, symbol_table,this->local_expressions[0],this->expression_flags[0]);

  }

  // Get joint friction
  const char* friction_str = config->Attribute("friction");
  if (friction_str == NULL){
    //ROS_DEBUG("joint dynamics: no friction, defaults to 0");
    this->friction = 0;
  }
  else
  {

   this->friction  = lexicalCastOrExpressionParsing("friction", config, symbol_table,this->local_expressions[1],this->expression_flags[1]);
    
  }

  if (damping_str == NULL && friction_str == NULL)
  {
    //ROS_ERROR("joint dynamics element specified with no damping and no friction");
    return false;
  }
  else{
    //ROS_DEBUG("joint dynamics: damping %f and friction %f", damping, friction);
    return true;
  }
}

bool JointLimits::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{
  this->clear();

  // Get lower joint limit
  const char* lower_str = config->Attribute("lower");
  if (lower_str == NULL){
    //ROS_DEBUG("joint limit: no lower, defaults to 0");
    this->lower = 0;
  }
  else
  {

   this->lower  = lexicalCastOrExpressionParsing("lower", config, symbol_table,this->local_expressions[0],this->expression_flags[0]);

  }

  // Get upper joint limit
  const char* upper_str = config->Attribute("upper");
  if (upper_str == NULL){
    //ROS_DEBUG("joint limit: no upper, , defaults to 0");
    this->upper = 0;
  }
  else
  {

    this->upper  = lexicalCastOrExpressionParsing("upper", config, symbol_table,this->local_expressions[1],this->expression_flags[1]);

  }

  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str == NULL){
    //ROS_ERROR("joint limit: no effort");
    return false;
  }
  else
  {

    this->effort  = lexicalCastOrExpressionParsing("effort", config, symbol_table,this->local_expressions[2],this->expression_flags[2]);

  }

  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL){
    //ROS_ERROR("joint limit: no velocity");
    return false;
  }
  else
  {

    this->velocity  = lexicalCastOrExpressionParsing("velocity", config, symbol_table,this->local_expressions[3],this->expression_flags[3]);

  }

  return true;
}


bool JointSafety::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{
  this->clear();

  // Get soft_lower_limit joint limit
  const char* soft_lower_limit_str = config->Attribute("soft_lower_limit");
  if (soft_lower_limit_str == NULL)
  {
    //ROS_DEBUG("joint safety: no soft_lower_limit, using default value");
    this->soft_lower_limit = 0;
  }
  else
  {

    this->soft_lower_limit  = lexicalCastOrExpressionParsing("soft_lower_limit", config, symbol_table,this->local_expressions[0],this->expression_flags[0]);

  }

  // Get soft_upper_limit joint limit
  const char* soft_upper_limit_str = config->Attribute("soft_upper_limit");
  if (soft_upper_limit_str == NULL)
  {
    //ROS_DEBUG("joint safety: no soft_upper_limit, using default value");
    this->soft_upper_limit = 0;
  }
  else
  {

    this->soft_upper_limit  = lexicalCastOrExpressionParsing("soft_upper_limit", config, symbol_table,this->local_expressions[1],this->expression_flags[1]);

  }

  // Get k_position_ safety "position" gain - not exactly position gain
  const char* k_position_str = config->Attribute("k_position");
  if (k_position_str == NULL)
  {
    //ROS_DEBUG("joint safety: no k_position, using default value");
    this->k_position = 0;
  }
  else
  {

    this->k_position  = lexicalCastOrExpressionParsing("k_position", config, symbol_table,this->local_expressions[2],this->expression_flags[2]);

  }
  // Get k_velocity_ safety velocity gain
  const char* k_velocity_str = config->Attribute("k_velocity");
  if (k_velocity_str == NULL)
  {
    //ROS_ERROR("joint safety: no k_velocity");
    return false;
  }
  else
  {

    this->k_velocity  = lexicalCastOrExpressionParsing("k_velocity", config, symbol_table,this->local_expressions[3],this->expression_flags[3]);

  }

  return true;
}

bool JointCalibration::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{
  this->clear();

  // Get rising edge position
  const char* rising_position_str = config->Attribute("rising");
  if (rising_position_str == NULL)
  {
    //ROS_DEBUG("joint calibration: no rising, using default value");
    this->rising.reset();
  }
  else
  {

    double temp = lexicalCastOrExpressionParsing("rising", config, symbol_table,this->local_expressions[0],this->expression_flags[0]);
    this->rising.reset(new double(temp));

  }

  // Get falling edge position
  const char* falling_position_str = config->Attribute("falling");
  if (falling_position_str == NULL)
  {
    //ROS_DEBUG("joint calibration: no falling, using default value");
    this->falling.reset();
  }
  else
  {

    double temp = lexicalCastOrExpressionParsing("falling", config, symbol_table,this->local_expressions[1],this->expression_flags[1]);
    this->falling.reset(new double(temp));

  }

  return true;
}

bool JointMimic::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{
  this->clear();

  // Get name of joint to mimic
  const char* joint_name_str = config->Attribute("joint");

  if (joint_name_str == NULL)
  {
    //ROS_ERROR("joint mimic: no mimic joint specified");
    //return false;
  }
  else
     this->joint_name = joint_name_str;
  
  // Get mimic multiplier
  const char* multiplier_str = config->Attribute("multiplier");

  if (multiplier_str == NULL)
  {
    //ROS_DEBUG("joint mimic: no multiplier, using default value of 1");
    this->multiplier = 1;    
  }
  else
  {

    this->multiplier  = lexicalCastOrExpressionParsing("multiplier", config, symbol_table,this->local_expressions[0],this->expression_flags[0]);

  }

  
  // Get mimic offset
  const char* offset_str = config->Attribute("offset");
  if (offset_str == NULL)
  {
    //ROS_DEBUG("joint mimic: no offset, using default value of 0");
    this->offset = 0;
  }
  else
  {

    this->offset  = lexicalCastOrExpressionParsing("offset", config, symbol_table,this->local_expressions[1],this->expression_flags[1]);

  }

  return true;
}

bool Joint::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{
  this->clear();

  // Get Joint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    //ROS_ERROR("unnamed joint found");
    return false;
  }
  this->name = name;
   
  // Get transform from Parent Link to Joint Frame
  TiXmlElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    //ROS_DEBUG("Joint '%s' missing origin tag under parent describing transform from Parent Link to Joint Frame, (using Identity transform).", this->name.c_str());
    this->parent_to_joint_origin_transform.clear();
  }
  else
  {
    try {
      this->parent_to_joint_origin_transform.initXml(origin_xml,symbol_table);
    }
    catch (ParseError &e) {
      this->parent_to_joint_origin_transform.clear();
      std::stringstream stm;
      stm << "Malformed parent origin element for joint [" << this->name << "]";
      throw ParseError(stm.str());
    }
  }

  // Get Parent Link
  TiXmlElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("link");
    if (!pname)
    {
      //ROS_INFO("no parent link name specified for Joint link '%s'. this might be the root?", this->name.c_str());
    }
    else
    {
      this->parent_link_name = std::string(pname);
    }
    const char *ptype = parent_xml->Attribute("type");
    if (!ptype)
    {
      //ROS_INFO("no parent link name specified for Joint link '%s'. this might be the root?", this->name.c_str());
    }
    else
    {
      this->parent_type = std::string(ptype);
    }
  }

  // Get Child Link
  TiXmlElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
    {
      //ROS_INFO("no child link name specified for Joint link '%s'.", this->name.c_str());
    }
    else
    {
      this->child_link_name = std::string(pname);

    }
    const char *ptype = child_xml->Attribute("type");
    if (!ptype)
    {
      //ROS_INFO("no child link name specified for Joint link '%s'. this might be the root?", this->name.c_str());
    }
    else
    {
      this->child_type = std::string(ptype);
    }
  }

  // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
    //ROS_ERROR("joint '%s' has no type, check to see if it's a reference.", this->name.c_str());
    return false;
  }
  std::string type_str = type_char;
  if (type_str == "planar")
  {
    type = PLANAR;
    //ROS_WARN("Planar joints are deprecated in the URDF!\n");
  }
  else if (type_str == "floating")
  {
    type = FLOATING;
    //ROS_WARN("Floating joints are deprecated in the URDF!\n");
  }
  else if (type_str == "revolute")
    type = REVOLUTE;
  else if (type_str == "continuous")
    type = CONTINUOUS;
  else if (type_str == "prismatic")
    type = PRISMATIC;
  else if (type_str == "fixed")
    type = FIXED;
  else
  {
    //ROS_ERROR("Joint '%s' has no known type '%s'", this->name.c_str(), type_str.c_str());
    return false;
  }

  // Get Joint Axis
  if (this->type != FLOATING && this->type != FIXED)
  {
    // axis
    TiXmlElement *axis_xml = config->FirstChildElement("axis");
    if (!axis_xml){
      //ROS_DEBUG("no axis elemement for Joint link '%s', defaulting to (1,0,0) axis", this->name.c_str());
      this->axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (axis_xml->Attribute("xyz")){
        try {
          this->axis.init(axis_xml->Attribute("xyz"),symbol_table);
        }
        catch (ParseError &e) {
          this->axis.clear();
          std::stringstream stm;
          stm << "Malformed axis element for joint ["<< this->name.c_str() << "]";
          throw ParseError(stm.str());
        }
      }
    }
  }

  // Get limit
  TiXmlElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
    limits.reset(new JointLimits);
    if (!limits->initXml(limit_xml,symbol_table))
    {
      //ROS_ERROR("Could not parse limit element for joint '%s'", this->name.c_str());
      limits.reset();
      return false;
    }
    
  }
  else if (this->type == REVOLUTE)
  {
    //ROS_ERROR("Joint '%s' is of type REVOLUTE but it does not specify limits", this->name.c_str());
    return false;
  }
  else if (this->type == PRISMATIC)
  {
    //ROS_INFO("Joint '%s' is of type PRISMATIC without limits", this->name.c_str());
    limits.reset();
  }

  // Get safety
  TiXmlElement *safety_xml = config->FirstChildElement("safety_controller");
  if (safety_xml)
  {
    safety.reset(new JointSafety);
    if (!safety->initXml(safety_xml,symbol_table))
    {
      //ROS_ERROR("Could not parse safety element for joint '%s'", this->name.c_str());
      safety.reset();
      return false;
    }
 
  }

  // Get calibration
  TiXmlElement *calibration_xml = config->FirstChildElement("calibration");
  if (calibration_xml)
  {
    calibration.reset(new JointCalibration);
    if (!calibration->initXml(calibration_xml,symbol_table))
    {
      //ROS_ERROR("Could not parse calibration element for joint  '%s'", this->name.c_str());
      calibration.reset();
      return false;
    }

  }

  // Get Joint Mimic
  TiXmlElement *mimic_xml = config->FirstChildElement("mimic");
  if (mimic_xml)
  {
    mimic.reset(new JointMimic);
    if (!mimic->initXml(mimic_xml,symbol_table))
    {
      //ROS_ERROR("Could not parse mimic element for joint  '%s'", this->name.c_str());
      mimic.reset();
      return false;
    }

  }

  // Get Dynamics
  TiXmlElement *prop_xml = config->FirstChildElement("dynamics");
  if (prop_xml)
  {
    dynamics.reset(new JointDynamics);
    if (!dynamics->initXml(prop_xml,symbol_table))
    {
      //ROS_ERROR("Could not parse joint_dynamics element for joint '%s'", this->name.c_str());
      dynamics.reset();
      return false;
    }

  }


  return true;
}



}
