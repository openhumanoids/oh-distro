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
#include <iostream>

#include <fstream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <otdf_interface/exceptions.h>
#include <otdf_interface/link.h>
#include <otdf_interface/otdf_addendum.h>

namespace otdf{

boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *g, ParamTable_t &symbol_table)
{
  boost::shared_ptr<Geometry> geom;
  if (!g) return geom;

  TiXmlElement *shape = g->FirstChildElement();
  if (!shape)
  {
    //ROS_ERROR("Geometry tag contains no child element.");
    std::cerr << "ERROR: Geometry tag contains no child element" << std::endl;
    return geom;
  }

  std::string type_name = shape->ValueStr();
  if (type_name == "sphere")
    geom.reset(new Sphere);
  else if (type_name == "box")
    geom.reset(new Box);
  else if (type_name == "cylinder")
    geom.reset(new Cylinder);
  else if (type_name == "mesh")
    geom.reset(new Mesh);
  else if (type_name == "torus")
    geom.reset(new Torus);
  else
  {
    //ROS_ERROR("Unknown geometry type '%s'", type_name.c_str());
    std::cerr << "ERROR: Unknown geometry type "<< type_name << std::endl;
    return geom;
  }

  // clear geom object when fails to initialize
  try {
    geom->initXml(shape,symbol_table);
  }
  catch (ParseError &e) {
    geom.reset();
    throw e.addMessage("failed to parse Geometry from shape");
  }

  return geom;
}

void Material::initXml(TiXmlElement *config, ParamTable_t &symbol_table)
{
  bool has_rgb = false;
  bool has_filename = false;

  this->clear();

  try {
    config->Attribute("name");
  }
  catch (ParseError &e) {
    throw e.addMessage("Material must contain a name attribute");
  }

  this->name = config->Attribute("name");

  // texture
  TiXmlElement *t = config->FirstChildElement("texture");
  if (t)
  {
    if (t->Attribute("filename"))
    {
      this->texture_filename = t->Attribute("filename");
      has_filename = true;
    }
  }

  // color
  TiXmlElement *c = config->FirstChildElement("color");
  if (c)
  {
    if (c->Attribute("rgba")) {

      try {
        this->color.init(c->Attribute("rgba"),symbol_table);
      }
      catch (ParseError &e) {
        this->color.clear();
        throw e.addMessage("Material ["+this->name+"] has malformed color rgba values.");
      }

      has_rgb = true;
    }
  }

  if (!has_rgb && !has_filename) {
    ParseError e;
    if (!has_rgb) e.addMessage("Material ["+this->name+"] color has no rgba");
    if (!has_filename) e.addMessage("Material ["+this->name+"] not defined in file");
    throw e;
  }
}

void Inertial::initXml(TiXmlElement *config, ParamTable_t &symbol_table)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    try {
      this->origin.initXml(o,symbol_table);
    }
    catch (ParseError &e) {
      this->origin.clear();
      throw e.addMessage("Inertial has a malformed origin tag");
    }
  }

  TiXmlElement *mass_xml = config->FirstChildElement("mass");
  if (!mass_xml)
  {
    throw ParseError("Inertial element must have a mass element");
  }
  if (!mass_xml->Attribute("value"))
  {
    throw ParseError("Inertial: mass element must have value attributes");
  }


  this->mass  = lexicalCastOrExpressionParsing("value",  mass_xml , symbol_table,this->local_expressions[0],this->expression_flags[0]);
  
//  std::cout << "ok Inertial  "<< " flag: "<<this->expression_flags[0] << std::endl;
//   std::cout << "mass: " <<  this->local_expressions[0].value()<< std::endl;
  
  TiXmlElement *inertia_xml = config->FirstChildElement("inertia");
  if (!inertia_xml)
  {
    throw ParseError("Inertial element must have inertia element");
  }
  if (!(inertia_xml->Attribute("ixx") && inertia_xml->Attribute("ixy") && inertia_xml->Attribute("ixz") &&
        inertia_xml->Attribute("iyy") && inertia_xml->Attribute("iyz") &&
        inertia_xml->Attribute("izz")))
  {
    throw ParseError("Inertial: inertia element must have ixx,ixy,ixz,iyy,iyz,izz attributes");
  }
  this->ixx  = lexicalCastOrExpressionParsing("ixx", inertia_xml, symbol_table,this->local_expressions[1],this->expression_flags[1]);
  this->ixy  = lexicalCastOrExpressionParsing("ixy", inertia_xml, symbol_table,this->local_expressions[2],this->expression_flags[2]);
  this->ixz  = lexicalCastOrExpressionParsing("ixz", inertia_xml, symbol_table,this->local_expressions[3],this->expression_flags[3]);
  this->iyy  = lexicalCastOrExpressionParsing("iyy", inertia_xml, symbol_table,this->local_expressions[4],this->expression_flags[4]);
  this->iyz  = lexicalCastOrExpressionParsing("iyz", inertia_xml, symbol_table,this->local_expressions[5],this->expression_flags[5]);
  this->izz  = lexicalCastOrExpressionParsing("izz", inertia_xml, symbol_table,this->local_expressions[6],this->expression_flags[6]);

}

void Visual::initXml(TiXmlElement *config, ParamTable_t &symbol_table)
{
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    try {
      this->origin.initXml(o,symbol_table);
    }
    catch (ParseError &e) {
      this->origin.clear();
      throw e.addMessage("Visual has a malformed origin tag");
    }
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  this->geometry = parseGeometry(geom, symbol_table);
  if (!this->geometry) {
    throw ParseError("Malformed geometry for Visual element");
  }

  // Material
  TiXmlElement *mat = config->FirstChildElement("material");
  if (mat) {
    // get material name
    if (!mat->Attribute("name")) {
      throw ParseError("Visual material must contain a name attribute");
    }
    this->material_name = mat->Attribute("name");

    // try to parse material element in place
    this->material.reset(new Material);
    try {
      this->material->initXml(mat,symbol_table);
    }
    catch (ParseError &e) {
      this->material.reset();
      //e.addMessage("INFO: Could not parse material element in Visual block, maybe defined outside.  resetting material, but not throwing.");
    }
  }

  // Group Tag (optional)
  // collision blocks without a group tag are designated to the "default" group
  const char *group_name_char = config->Attribute("group");
  if (!group_name_char)
    group_name = std::string("default");
  else
    group_name = std::string(group_name_char);
}

void Collision::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{  
  this->clear();

  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    try {
      this->origin.initXml(o,symbol_table);
    }
    catch (ParseError &e)
    {
      this->origin.clear();
      throw e.addMessage("Collision has a malformed origin tag");
    }
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  this->geometry = parseGeometry(geom, symbol_table);
  if (!this->geometry)
  {
    throw ParseError("Malformed geometry for Collision element");
  }

  // Group Tag (optional)
  // collision blocks without a group tag are designated to the "default" group
  const char *group_name_char = config->Attribute("group");
  if (!group_name_char)
    group_name = std::string("default");
  else
    group_name = std::string(group_name_char);
}

void Sphere::initXml(TiXmlElement *c, ParamTable_t &symbol_table)
{
  this->clear();

  this->type = SPHERE;
  if (!c->Attribute("radius"))
  {
    throw ParseError("Sphere shape must have a radius attribute");
  }

  this->radius  = lexicalCastOrExpressionParsing("radius", c, symbol_table,this->local_expressions[0],this->expression_flags[0]);

}

void Box::initXml(TiXmlElement *c, ParamTable_t &symbol_table)
{
  this->clear();

  this->type = BOX;
  if (!c->Attribute("size"))
  {
    throw ParseError("Box shape has no size attribute");
  }
  try
  {
    this->dim.init(c->Attribute("size"),symbol_table);
  }
  catch (ParseError &e)
  {
    this->dim.clear();
    std::stringstream stm;
    stm << "Box shape has malformed size attribute ["
        << c->Attribute("size")
        << "].";
    throw e.addMessage(stm.str());
  }
}

void Cylinder::initXml(TiXmlElement *c, ParamTable_t &symbol_table)
{
  this->clear();

  this->type = CYLINDER;
  if (!c->Attribute("length") ||
      !c->Attribute("radius"))
  {
    throw ParseError("Cylinder shape must have both length and radius attributes");
  }

  this->length  = lexicalCastOrExpressionParsing("length", c, symbol_table,this->local_expressions[0],this->expression_flags[0]);

  this->radius  = lexicalCastOrExpressionParsing("radius", c, symbol_table,this->local_expressions[1],this->expression_flags[1]);

}

void Torus::initXml(TiXmlElement *c, ParamTable_t &symbol_table)
{
  this->clear();

  this->type = TORUS;
  if (!c->Attribute("radius") ||
      !c->Attribute("tube_radius"))
  {
    throw ParseError("Torus shape must have both radius and tube_radius attributes");
  }
  
  this->radius  = lexicalCastOrExpressionParsing("radius", c, symbol_table,this->local_expressions[0],this->expression_flags[0]);

  this->tube_radius = lexicalCastOrExpressionParsing("tube_radius", c, symbol_table,this->local_expressions[1],this->expression_flags[1]);

}

void Mesh::initXml(TiXmlElement *c, ParamTable_t &symbol_table)
{
  this->clear();

  this->type = MESH;
  if (!c->Attribute("filename")) {
    throw ParseError("Mesh must contain a filename attribute");
  }

  filename = c->Attribute("filename");

  if (c->Attribute("scale")) {
    try {
      this->scale.init(c->Attribute("scale"),symbol_table);
    }
    catch (ParseError &e) {
      this->scale.clear();
      throw e.addMessage("Mesh scale was specified, but could not be parsed");
    }
  }
  else
  {
    //("Mesh scale was not specified, default to (1,1,1)");
  }
}


void Link::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{

  this->clear();
  
  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    throw ParseError("No name given for the link.");
  }
  name = std::string(name_char);

  // Inertial (optional)
  TiXmlElement *i = config->FirstChildElement("inertial");
  if (i)
  {
    inertial.reset(new Inertial);
    try { 
      inertial->initXml(i,symbol_table);    
    }
    catch (ParseError &e) {
      std::stringstream stm;
      stm << "Could not parse inertial element for Link [" << this->name << "]";
      throw e.addMessage(stm.str());
    }
  }

  // Multiple Visuals (optional)
  for (TiXmlElement* vis_xml = config->FirstChildElement("visual"); vis_xml; vis_xml = vis_xml->NextSiblingElement("visual"))
  {

    boost::shared_ptr<Visual> vis;
    vis.reset(new Visual);

    try {
      vis->initXml(vis_xml,symbol_table);
      boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > viss = this->getVisuals(vis->group_name);
      if (!viss)
      {
        // group does not exist, create one and add to map
        viss.reset(new std::vector<boost::shared_ptr<Visual > >);
        // new group name, create vector, add vector to map and add Visual to the vector
        this->visual_groups.insert(make_pair(vis->group_name,viss));
        //ROS_DEBUG("successfully added a new visual group name '%s'",vis->group_name.c_str());
      }

      // group exists, add Visual to the vector in the map
      viss->push_back(vis);
      //ROS_DEBUG("successfully added a new visual under group name '%s'",vis->group_name.c_str());
    }
    catch (ParseError &e) {
      vis.reset();
      std::stringstream stm;
      stm << "Could not parse visual element for Link [" << this->name << "]";
      throw e.addMessage(stm.str());
    }
  }

  // Visual (optional)
  // Assign one single default visual pointer from the visual_groups map
  this->visual.reset();
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > default_visual = this->getVisuals("default");
  if (!default_visual)
  {
    //("No 'default' visual group for Link '%s'", this->name.c_str());
  }
  else if (default_visual->empty())
  {
    //("'default' visual group is empty for Link '%s'", this->name.c_str());
  }
  else
  {
    if (default_visual->size() > 1)
    {
      //("'default' visual group has %d visuals for Link '%s', taking the first one as default",(int)default_visual->size(), this->name.c_str());
    }
    this->visual = (*default_visual->begin());
  }


  // Multiple Collisions (optional)
  for (TiXmlElement* col_xml = config->FirstChildElement("collision"); col_xml; col_xml = col_xml->NextSiblingElement("collision"))
  {
    boost::shared_ptr<Collision> col;
    col.reset(new Collision);
    try {
      col->initXml(col_xml,symbol_table);

      boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > cols = this->getCollisions(col->group_name);  
      
      if (!cols)
      {
        // group does not exist, create one and add to map
        cols.reset(new std::vector<boost::shared_ptr<Collision > >);
        // new group name, create vector, add vector to map and add Collision to the vector
        this->collision_groups.insert(make_pair(col->group_name,cols));
        //ROS_DEBUG("successfully added a new collision group name '%s'",col->group_name.c_str());
      }

      // group exists, add Collision to the vector in the map
      cols->push_back(col);
      //ROS_DEBUG("successfully added a new collision under group name '%s'",col->group_name.c_str());
    }
    catch (ParseError &e) {
      col.reset();
      std::stringstream stm;
      stm << "Could not parse collision element for Link [" << this->name << "]";
      throw ParseError(stm.str());
    }
  }

  // Collision (optional)
  // Assign one single default collision pointer from the collision_groups map
  this->collision.reset();
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > default_collision = this->getCollisions("default");

  if (!default_collision)
  {
    //ROS_DEBUG("No 'default' collision group for Link '%s'", this->name.c_str());
  }
  else if (default_collision->empty())
  {
    //ROS_DEBUG("'default' collision group is empty for Link '%s'", this->name.c_str());
  }
  else
  {
    if (default_collision->size() > 1)
    {
      //ROS_WARN("'default' collision group has %d collisions for Link '%s', taking the first one as default",(int)default_collision->size(), this->name.c_str());
    }
    this->collision = (*default_collision->begin());
  }
}

void Link::addVisual(std::string group_name, boost::shared_ptr<Visual> visual)
{
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > viss = this->getVisuals(group_name);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<boost::shared_ptr<Visual > >);
    // new group name, create vector, add vector to map and add Visual to the vector
    this->visual_groups.insert(make_pair(group_name,viss));
    //ROS_DEBUG("successfully added a new visual group name '%s'",group_name.c_str());
  }

  // group exists, add Visual to the vector in the map
  std::vector<boost::shared_ptr<Visual > >::iterator vis_it = find(viss->begin(),viss->end(),visual);
  if (vis_it != viss->end())
  {
    //ROS_WARN("attempted to add a visual that already exists under group name '%s', skipping.",group_name.c_str());
  }
  else
    viss->push_back(visual);
  //ROS_DEBUG("successfully added a new visual under group name '%s'",group_name.c_str());

}

boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > Link::getVisuals(const std::string& group_name) const
{
  boost::shared_ptr<std::vector<boost::shared_ptr<Visual > > > ptr;
  if (this->visual_groups.find(group_name) == this->visual_groups.end())
    ptr.reset();
  else
    ptr = this->visual_groups.find(group_name)->second;
  return ptr;
}


void Link::addCollision(std::string group_name, boost::shared_ptr<Collision> collision)
{
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > viss = this->getCollisions(group_name);
  if (!viss)
  {
    // group does not exist, create one and add to map
    viss.reset(new std::vector<boost::shared_ptr<Collision > >);
    // new group name, create vector, add vector to map and add Collision to the vector
    this->collision_groups.insert(make_pair(group_name,viss));
    //ROS_DEBUG("successfully added a new collision group name '%s'",group_name.c_str());
  }

  // group exists, add Collision to the vector in the map
  std::vector<boost::shared_ptr<Collision > >::iterator vis_it = find(viss->begin(),viss->end(),collision);
  if (vis_it != viss->end())
  {
    //ROS_WARN("attempted to add a collision that already exists under group name '%s', skipping.",group_name.c_str());
  }
  else
    viss->push_back(collision);
  //ROS_DEBUG("successfully added a new collision under group name '%s'",group_name.c_str());

}

boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > Link::getCollisions(const std::string& group_name) const
{
  boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > ptr;
  if (this->collision_groups.find(group_name) == this->collision_groups.end())
    ptr.reset();
  else
    ptr = this->collision_groups.find(group_name)->second;
  return ptr;
}

void Link::setParent(boost::shared_ptr<BaseEntity> parent)
{
  this->parent_link_ = parent;
  //ROS_DEBUG("set parent Link '%s' for Link '%s'", parent->name.c_str(), this->name.c_str());
}

void Link::setParentJoint(boost::shared_ptr<Joint> parent)
{
  this->parent_joint = parent;
 // ROS_DEBUG("set parent joint '%s' to Link '%s'",  parent->name.c_str(), this->name.c_str());
}

void Link::addChild(boost::shared_ptr<BaseEntity> child)
{
  this->child_links.push_back(child);
  //ROS_DEBUG("added child Link '%s' to Link '%s'",  child->name.c_str(), this->name.c_str());
}

void Link::addChildJoint(boost::shared_ptr<Joint> child)
{
  this->child_joints.push_back(child);
  //ROS_DEBUG("added child Joint '%s' to Link '%s'", child->name.c_str(), this->name.c_str());
}


void Bounding_volume::initXml(TiXmlElement *config, ParamTable_t &symbol_table)
{
  this->clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    throw ParseError("No name given for the bounding volume.");
  }
  name = std::string(name_char);
  
  // Origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o) {
    try {
      this->origin.initXml(o,symbol_table);
    }
    catch (ParseError &e) {
      this->origin.clear();
      throw e.addMessage("Visual has a malformed origin tag");
    }
  }

  // Geometry
  TiXmlElement *geom = config->FirstChildElement("geometry");
  this->geometry = parseGeometry(geom, symbol_table);
  if (!this->geometry) {
    throw ParseError("Malformed geometry for Visual element");
  }
}

void Bounding_volume::setParent(boost::shared_ptr<BaseEntity> parent)
{
  this->parent_link_ = parent;
  //ROS_DEBUG("set parent Link '%s' for Link '%s'", parent->name.c_str(), this->name.c_str());
}

void Bounding_volume::setParentJoint(boost::shared_ptr<Joint> parent)
{
  this->parent_joint = parent;
  //ROS_DEBUG("set parent joint '%s' to Link '%s'",  parent->name.c_str(), this->name.c_str());
}

void Bounding_volume::addChild(boost::shared_ptr<BaseEntity> child)
{
  this->child_links.push_back(child);
  //ROS_DEBUG("added child Link '%s' to Link '%s'",  child->name.c_str(), this->name.c_str());
}

void Bounding_volume::addChildJoint(boost::shared_ptr<Joint> child)
{
  this->child_joints.push_back(child);
  //ROS_DEBUG("added child Joint '%s' to Link '%s'", child->name.c_str(), this->name.c_str());
}


void Link_pattern::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{
  
  this->clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    throw ParseError("No name given for the Link_pattern.");
  }
  name = std::string(name_char);

  // parse noofrepetitions config->Attribute("noofrepetitions");
  this->noofrepetitions  = lexicalCastOrExpressionParsing("noofrepetitions",  config , symbol_table,this->local_expressions[0],this->expression_flags[0]);

  try { 
       this->link_template->initXml(config,symbol_table); 
      }
  catch (ParseError &e) {
      std::stringstream stm;
      stm << "Could not parse link_template element for Link Pattern [" << this->name << "]";
      throw e.addMessage(stm.str());
      }

   for  (unsigned int i=0; i < noofrepetitions; i++){
     boost::shared_ptr<Link> temp; 
     temp.reset(new Link(*link_template));
     std::ostringstream stm;   
     stm << name << "_" << i; // append ID to pattern name
     temp->name =stm.str();    
     this->link_set.push_back(temp);
   }

} // end link_pattern init



bool Joint_pattern::initXml(TiXmlElement* config, ParamTable_t &symbol_table)
{
  
  this->clear();

  // Get Joint Pattern Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    //ROS_ERROR("unnamed joint found");
    return false;
  }
  this->name = name;
  
 // Get Joint pattern type
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
     std::cerr << " ERROR: joint pattern "<< this->name <<" has no known type"<< type_str << std::endl;
     return false;
  } 
  
    this->noofrepetitions  = lexicalCastOrExpressionParsing("noofrepetitions",  config , symbol_table,this->local_expressions[0],this->expression_flags[0]);
  

  // Get transform of the base origin
  TiXmlElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    //ROS_DEBUG("Joint '%s' missing origin tag under parent describing transform from Parent Link to Joint Frame, (using Identity transform).", this->name.c_str());
    this->origin.clear();
  }
  else
  {
    try {
      this->origin.initXml(origin_xml,symbol_table);
    }
    catch (ParseError &e) {
      this->origin.clear();
      std::stringstream stm;
      stm << "Malformed parent origin element for joint pattern [" << this->name << "]";
      throw ParseError(stm.str());
    }
  }
  
    // Get pattern offset
  TiXmlElement *offset_xml = config->FirstChildElement("pattern_offset");
  if (!offset_xml)
  {
    this->pattern_offset.clear();
  }
  else
  {
    try {
      this->pattern_offset.initXml(offset_xml,symbol_table);
    }
    catch (ParseError &e) {
      this->pattern_offset.clear();
      std::stringstream stm;
      stm << "Malformed pattern_offset element for joint pattern [" << this->name << "]";
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
  
    // Get Child Link Pattern
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
      this->child_link_pattern_name = std::string(pname);

    }
    const char *ptype = child_xml->Attribute("type");
    if (!ptype)
    {
      //ROS_INFO("no child link name specified for Joint link '%s'. this might be the root?", this->name.c_str());
    }
    else
    {
      this->child_link_pattern_type = std::string(ptype);
    }
  }

  try { 
       this->joint_template->initXml(config,symbol_table); 
      }
  catch (ParseError &e) {
      std::stringstream stm;
      stm << "Could not parse joint_template element for Link Pattern [" << this->name << "]";
      throw e.addMessage(stm.str());
      }
      
   for  (unsigned int i=0; i < noofrepetitions; i++){
     boost::shared_ptr<Joint> temp; 
     temp.reset(new Joint(*joint_template));
     std::ostringstream stm;   
     stm << name << "_" << i; // append ID to pattern name
     temp->name =stm.str();
     stm.clear();
     stm.str("");
     stm << temp->child_link_name << "_" << i; // append ID to pattern name
     temp->child_link_name= stm.str();
     if(i==0)
       temp->parent_to_joint_origin_transform = origin;     
     else{
       temp->parent_to_joint_origin_transform.position =  this->pattern_offset.position + this->joint_set[i-1]->parent_to_joint_origin_transform.position;
       temp->parent_to_joint_origin_transform.rotation =  this->pattern_offset.rotation*(this->joint_set[i-1]->parent_to_joint_origin_transform.rotation);
     }   
     this->joint_set.push_back(temp);
   }
  return true;
}

}// end namespace

