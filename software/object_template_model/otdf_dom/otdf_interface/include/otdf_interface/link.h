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
#ifndef OTDF_INTERFACE_LINK_H
#define OTDF_INTERFACE_LINK_H

#include <string>
#include <vector>
#include <map>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "expression_parsing.h"
#include "joint.h"
#include "color.h"

namespace otdf{

  class BaseEntity
  {
  public:
    BaseEntity(){}

  std::string name;
  
  virtual std::string getEntityType() const{}
  
  virtual void initXml(TiXmlElement* config ,ParamTable_t &symbol_table){}  
  virtual void clear(){} 
  virtual void update(){} 
  virtual void setParent(boost::shared_ptr<BaseEntity> parent){} 
  virtual void setParentJoint(boost::shared_ptr<Joint> parent){}  
  virtual void addChild(boost::shared_ptr<BaseEntity> child){}   
  virtual void addChildJoint(boost::shared_ptr<Joint> child){}  
  virtual void clearChildEntities(){}   
  virtual void clearChildJoints(){}  
  virtual boost::shared_ptr<BaseEntity> getParent(){}
  
//   void setParent(boost::shared_ptr<BaseEntity> parent)
//   {
//     this->parent_link_ = parent;
//     //ROS_DEBUG("set parent Link '%s' for Link '%s'", parent->name.c_str(), this->name.c_str());
//   };
//   boost::shared_ptr<BaseEntity> getParent() const
//   {return parent_link_.lock();};
//   
//   void setParentJoint(boost::shared_ptr<Joint> parent)
//   {
//     this->parent_joint = parent;
//     //ROS_DEBUG("set parent joint '%s' to Link '%s'",  parent->name.c_str(), this->name.c_str());
//   };
// 
//   void addChild(boost::shared_ptr<BaseEntity> child)
//   {
//     this->child_links.push_back(child);
//     //ROS_DEBUG("added child Link '%s' to Link '%s'",  child->name.c_str(), this->name.c_str());
//   };
// 
//   void addChildJoint(boost::shared_ptr<Joint> child)
//   {
//     this->child_joints.push_back(child);
//     //ROS_DEBUG("added child Joint '%s' to Link '%s'", child->name.c_str(), this->name.c_str());
//   };
//     /// Parent Joint element
//   ///   explicitly stating "parent" because we want directional-ness for tree structure
//   ///   every link can have one parent
//   boost::shared_ptr<Joint> parent_joint;
// 
//   std::vector<boost::shared_ptr<Joint> > child_joints;
//   std::vector<boost::shared_ptr<BaseEntity> > child_links;
//   
//   boost::weak_ptr<BaseEntity> parent_link_;
  };  
  
class Geometry
{
public:
  enum {SPHERE, BOX, CYLINDER, MESH, TORUS} type;

  virtual void initXml(TiXmlElement *, ParamTable_t &symbol_table) = 0;
  virtual void update(){}
};

class Sphere : public Geometry
{
public:
  Sphere() { this->clear(); };
  double radius;

  
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;

  void clear()
  {
    local_expressions.clear();
    expression_flags.clear();        
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression);// initialise local_expressions
    expression_flags.push_back(false);
    radius = 0;
  };
  
  void update()
  {
    if(expression_flags[0])
       this->radius = this->local_expressions[0].value();
  };
  void initXml(TiXmlElement *, ParamTable_t &symbol_table);
};

class Box : public Geometry
{
public:
  Box() { this->clear(); };
  Vector3 dim;

  void clear()
  {
    this->dim.clear();
  };
  
  void update()
  {
    this->dim.update();
  };
  void initXml(TiXmlElement *, ParamTable_t &symbol_table);
};

class Cylinder : public Geometry
{
public:
  Cylinder() { this->clear(); };
  double length;
  double radius;
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;

  void clear()
  {
    local_expressions.clear();
    expression_flags.clear();    
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression);// initialise local_expressions
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    length = 0;
    radius = 0;
  };
  void update()
  {
    if(expression_flags[0]){
	this->length = this->local_expressions[0].value();
    }
    
    if(expression_flags[1]){
	this->radius = this->local_expressions[1].value();
    }
  };
  void initXml(TiXmlElement *, ParamTable_t &symbol_table);
};

class Mesh : public Geometry
{
public:
  Mesh() { this->clear(); };
  std::string filename;
  Vector3 scale;

  void clear()
  {
    filename.clear();
    // default scale
    scale.x = 1;
    scale.y = 1;
    scale.z = 1;
  };
  void update()
  {
    this->scale.update();
  };
  void initXml(TiXmlElement *, ParamTable_t &symbol_table);
  bool fileExists(std::string filename);
};

class Torus : public Geometry
{
public:
  Torus() { this->clear(); };
  //see http://en.wikipedia.org/wiki/Torus for parametric equation.
  double radius; // distance from the center of the tube to the center of the torus,
  double tube_radius;
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;

  void clear()
  {
    local_expressions.clear();
    expression_flags.clear();
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression);// initialise local_expressions
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    radius = 0;
    tube_radius = 0;
  };
  void update()
  {
    if(expression_flags[0]){
	this->radius = this->local_expressions[0].value();
    }
    
    if(expression_flags[1]){
	this->tube_radius = this->local_expressions[1].value();
    }
  };
  void initXml(TiXmlElement *, ParamTable_t &symbol_table);
};

class Material
{
public:
  Material() { this->clear(); };
  std::string name;
  std::string texture_filename;
  Color color;

  void clear()
  {
    color.clear();
    texture_filename.clear();
    name.clear();
  };
  void update()
  {
    this->color.update();
  };
  void initXml(TiXmlElement* config ,ParamTable_t &symbol_table);
};

class Inertial
{
public:
  Inertial() { this->clear(); };
  Pose origin;
  double mass;
  double ixx,ixy,ixz,iyy,iyz,izz;
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;
  void clear()
  {
    local_expressions.clear();
    expression_flags.clear();
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression);// initialise local_expressions
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    origin.clear();
    mass = 0;
    ixx = ixy = ixz = iyy = iyz = izz = 0;
  };
  void update()
  {
    this->origin.update();
    if(expression_flags[0]){
	this->mass = this->local_expressions[0].value();
    }
    
    if(expression_flags[1]){
	this->ixx = this->local_expressions[1].value();
    }
    if(expression_flags[2]){
	this->ixy = this->local_expressions[2].value();
    }    
    if(expression_flags[3]){
	this->ixz = this->local_expressions[5].value();
    }
    if(expression_flags[4]){
	this->iyy = this->local_expressions[4].value();
    }    
    if(expression_flags[5]){
	this->iyz = this->local_expressions[5].value();
    }
    if(expression_flags[6]){
	this->izz = this->local_expressions[6].value();
    }
  };
  void initXml(TiXmlElement* config ,ParamTable_t &symbol_table);
};

class Visual
{
public:
  Visual() { this->clear(); };
  Pose origin;
  boost::shared_ptr<Geometry> geometry;

  std::string material_name;
  boost::shared_ptr<Material> material;

  void clear()
  {
    origin.clear();
    material_name.clear();
    material.reset(new Material);
    geometry.reset(); // can't set it to a object as initXml is a pure virtual.
    this->group_name.clear();
  };
  void update()
  {
    this->origin.update();
    this->material->update();
    if(geometry) // if not empty then update
     this->geometry->update();
  };
  void initXml(TiXmlElement* config ,ParamTable_t &symbol_table);
  std::string group_name;
};

class Collision
{
public:
  Collision() { this->clear(); };
  Pose origin;
  boost::shared_ptr<Geometry> geometry;

  void clear()
  {
    origin.clear();
    geometry.reset();
    this->group_name.clear();
  };
  
  void update()
  {
    this->origin.update();
    if(geometry) // if not empty then update
      this->geometry->update();
  };
  void initXml(TiXmlElement* config ,ParamTable_t &symbol_table);
  std::string group_name;
};


class Link : public BaseEntity
{
public:
  Link() { this->clear(); };

  /// inertial element
  boost::shared_ptr<Inertial> inertial;

  /// visual element
  boost::shared_ptr<Visual> visual;

  /// collision element
  boost::shared_ptr<Collision> collision;

  /// a collection of visual elements, keyed by a string tag called "group"
  std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<Visual> > > > visual_groups;

  /// a collection of collision elements, keyed by a string tag called "group"
  std::map<std::string, boost::shared_ptr<std::vector<boost::shared_ptr<Collision> > > > collision_groups;

  /// Parent Joint element
  ///   explicitly stating "parent" because we want directional-ness for tree structure
  ///   every link can have one parent
  boost::shared_ptr<Joint> parent_joint;

  std::vector<boost::shared_ptr<Joint> > child_joints;
  std::vector<boost::shared_ptr<BaseEntity> > child_links;

  void initXml(TiXmlElement* config ,ParamTable_t &symbol_table);

  boost::shared_ptr<BaseEntity> getParent() const
  {return parent_link_.lock();};

  void setParent(boost::shared_ptr<BaseEntity> parent);

  void clear()
  {
    this->inertial.reset(new Inertial);
    this->visual.reset(new Visual);
    this->collision.reset(new Collision);
    this->collision_groups.clear();
  };
 void update()
  {
 
    this->inertial->update();
    
    if(this->visual)
      this->visual->update();
    
    if(this->collision)
      this->collision->update();
  };
  std::string getEntityType() const{
    std::string str = "Link";
    return str;};
  void setParentJoint(boost::shared_ptr<Joint> child);
  void addChild(boost::shared_ptr<BaseEntity> child);
  void addChildJoint(boost::shared_ptr<Joint> child);
  void clearChildEntities(){
    this->child_links.clear();    
  };   
  void clearChildJoints(){
    this->child_joints.clear();    
  };   
  void addVisual(std::string group_name, boost::shared_ptr<Visual> visual);
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > getVisuals(const std::string& group_name) const;
  void addCollision(std::string group_name, boost::shared_ptr<Collision> collision);
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > getCollisions(const std::string& group_name) const;
private:
  boost::weak_ptr<BaseEntity> parent_link_;

};




}

#endif
