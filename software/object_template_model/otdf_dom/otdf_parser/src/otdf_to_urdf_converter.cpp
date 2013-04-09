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
#include <cmath>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "otdf_parser/otdf_parser.h"
#include "otdf_parser/otdf_urdf_converter.h"
#include "otdf_interface/exceptions.h"



namespace otdf{
  enum
  { UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED }; 
  
 void addGeometry(boost::shared_ptr<const Geometry> geom,TiXmlElement* element, bool compliant)
{
  //enum {SPHERE, BOX, CYLINDER, MESH, TORUS, DYNAMIC_MESH};
  TiXmlElement * geometry = new TiXmlElement( "geometry" );
  //if(geom->type!=(const int) TORUS)
    element->LinkEndChild(geometry); // not recognised in urdf

  if(geom->type== (const int) otdf::Geometry::SPHERE){
   boost::shared_ptr<const Sphere> downcasted_geom(boost::shared_dynamic_cast<const Sphere>(geom)); 
   TiXmlElement * sphere = new TiXmlElement( "sphere" );
   sphere->SetDoubleAttribute("radius", downcasted_geom->radius);
   geometry->LinkEndChild(sphere); 
  }
  else if (geom->type==(const int) otdf::Geometry::BOX){
    boost::shared_ptr<const Box> downcasted_geom(boost::shared_dynamic_cast<const Box>(geom)); 
    TiXmlElement * box = new TiXmlElement( "box" );
    std::ostringstream stm; 
    stm << downcasted_geom->dim.x << " " << downcasted_geom->dim.y << " " << downcasted_geom->dim.z << " ";
    box->SetAttribute("size", stm.str());
    geometry->LinkEndChild(box);  
  }
   else if (geom->type==(const int) otdf::Geometry::CYLINDER){
  boost::shared_ptr<const Cylinder> downcasted_geom(boost::shared_dynamic_cast<const Cylinder>(geom)); 
   TiXmlElement * cyl = new TiXmlElement( "cylinder" );
   cyl->SetDoubleAttribute("length",downcasted_geom->length);
   cyl->SetDoubleAttribute("radius",downcasted_geom->radius);
   geometry->LinkEndChild(cyl); 
  }
  else if (geom->type==(const int) otdf::Geometry::MESH){
   boost::shared_ptr<const Mesh> downcasted_geom(boost::shared_dynamic_cast<const Mesh>(geom)); 
   TiXmlElement * mesh = new TiXmlElement( "mesh" );
   mesh->SetAttribute("filename",downcasted_geom->filename);
   std::ostringstream stm; 
   stm << downcasted_geom->scale.x << " " << downcasted_geom->scale.y << " " << downcasted_geom->scale.z << " ";
   mesh->SetAttribute("scale", stm.str());
   geometry->LinkEndChild(mesh); 
  }
  else if (geom->type==(const int) otdf::Geometry::TORUS){
   boost::shared_ptr<const Torus> downcasted_geom(boost::shared_dynamic_cast<const Torus>(geom)); 
     if(compliant==false){
     TiXmlElement * torus = new TiXmlElement( "torus" );
     torus->SetDoubleAttribute("radius", downcasted_geom->radius);
     torus->SetDoubleAttribute("tube_radius", downcasted_geom->tube_radius);
     geometry->LinkEndChild(torus);
     }
     else{
     TiXmlElement * torus = new TiXmlElement( "cylinder" );
     torus->SetDoubleAttribute("radius", downcasted_geom->radius);
     torus->SetDoubleAttribute("length", downcasted_geom->tube_radius);
     geometry->LinkEndChild(torus);
    }    
  }
  else if (geom->type==(const int) otdf::Geometry::DYNAMIC_MESH){
   boost::shared_ptr<const DynamicMesh> downcasted_geom(boost::shared_dynamic_cast<const DynamicMesh>(geom)); 
     if(compliant==false){
     TiXmlElement * dynamic_mesh = new TiXmlElement( "dynamic_mesh" );
     geometry->LinkEndChild(dynamic_mesh);
     }
     else{
     TiXmlElement * mesh = new TiXmlElement( "mesh" );
     std::string empty_filename= " ";
     mesh->SetAttribute("filename",empty_filename);
     geometry->LinkEndChild(mesh); 
    }    
  }
}

void addVisual(boost::shared_ptr<const Link> downcasted_entity,TiXmlElement* element, bool compliant)
{
   TiXmlElement * visual = new TiXmlElement( "visual" );

   TiXmlElement * origin = new TiXmlElement( "origin" );
   visual->LinkEndChild(origin);
    std::ostringstream stm; 
    stm << downcasted_entity->visual->origin.position.x << " "
	<< downcasted_entity->visual->origin.position.y << " "
	<< downcasted_entity->visual->origin.position.z << " ";
   origin->SetAttribute("xyz", stm.str());
     stm.clear();
    stm.str("");
    double r,p,y;
   downcasted_entity->visual->origin.rotation.getRPY(r,p,y);
   stm << r << " " << p << " " << y << " ";
   origin->SetAttribute("rpy", stm.str());   
 
   if(downcasted_entity->visual->geometry){
   addGeometry(downcasted_entity->visual->geometry,visual,compliant);   
   }
   element->LinkEndChild(visual);
}

void addInertial(boost::shared_ptr<const Link> downcasted_entity,TiXmlElement* element)
{
   TiXmlElement * inertial = new TiXmlElement( "inertial" );

    TiXmlElement * origin = new TiXmlElement( "origin" );
    inertial->LinkEndChild(origin);
      std::ostringstream stm; 
      stm << downcasted_entity->inertial->origin.position.x << " "
	  << downcasted_entity->inertial->origin.position.y << " "
	  << downcasted_entity->inertial->origin.position.z << " ";
    origin->SetAttribute("xyz", stm.str());
      stm.clear();
      stm.str("");
      double r,p,y;
      downcasted_entity->inertial->origin.rotation.getRPY(r,p,y);
      stm << r << " " << p << " " << y << " ";
    origin->SetAttribute("rpy", stm.str());   
  
 
   TiXmlElement * mass = new TiXmlElement( "mass" );
   mass->SetDoubleAttribute("value", downcasted_entity->inertial->mass);
   
   inertial->LinkEndChild(mass);
   TiXmlElement * inertia = new TiXmlElement( "inertia" );
   inertia->SetDoubleAttribute("ixx", downcasted_entity->inertial->ixx); 
   inertia->SetDoubleAttribute("ixy", downcasted_entity->inertial->ixy); 
   inertia->SetDoubleAttribute("ixz", downcasted_entity->inertial->ixz); 
   inertia->SetDoubleAttribute("iyy", downcasted_entity->inertial->iyy); 
   inertia->SetDoubleAttribute("iyz", downcasted_entity->inertial->iyz); 
   inertia->SetDoubleAttribute("izz", downcasted_entity->inertial->izz); 
   inertial->LinkEndChild(inertia);
   element->LinkEndChild(inertial);
}

void addDummyInertial(double x, double y,double z,double r,double p,double yaw,TiXmlElement* element)
{
   TiXmlElement * inertial = new TiXmlElement( "inertial" );

    TiXmlElement * origin = new TiXmlElement( "origin" );
    inertial->LinkEndChild(origin);
      std::ostringstream stm; 
      stm << x << " "
	  << y << " "
	  << z << " ";
    origin->SetAttribute("xyz", stm.str());
      stm.clear();
      stm.str("");
      stm << r << " " << p << " " << yaw << " ";
    origin->SetAttribute("rpy", stm.str());   
  
 
   TiXmlElement * mass = new TiXmlElement( "mass" );
   mass->SetDoubleAttribute("value", 0.01);
   
   inertial->LinkEndChild(mass);
   TiXmlElement * inertia = new TiXmlElement( "inertia" );
   inertia->SetDoubleAttribute("ixx", 0.0001); 
   inertia->SetDoubleAttribute("ixy", 0.0); 
   inertia->SetDoubleAttribute("ixz", 0.0); 
   inertia->SetDoubleAttribute("iyy", 0.0001); 
   inertia->SetDoubleAttribute("iyz", 0.0); 
   inertia->SetDoubleAttribute("izz", 0.0001); 
   inertial->LinkEndChild(inertia);
   element->LinkEndChild(inertial);
}

void addCollision(boost::shared_ptr<const Link> downcasted_entity,TiXmlElement* element, bool compliant)
{
   TiXmlElement * collision = new TiXmlElement( "collision" );
  
    TiXmlElement * origin = new TiXmlElement( "origin" );
    collision->LinkEndChild(origin);
      std::ostringstream stm; 
      stm << downcasted_entity->collision->origin.position.x << " "
	  << downcasted_entity->collision->origin.position.y << " "
	  << downcasted_entity->collision->origin.position.z << " ";
      origin->SetAttribute("xyz", stm.str());
      stm.clear();
      stm.str("");
      double r,p,y;
    downcasted_entity->collision->origin.rotation.getRPY(r,p,y);
    stm << r << " " << p << " " << y << " ";
    origin->SetAttribute("rpy", stm.str());   

   addGeometry(downcasted_entity->collision->geometry,collision,compliant); 
   element->LinkEndChild(collision);
}

void addChildEntities(boost::shared_ptr<const BaseEntity> entity,TiXmlElement* robot, bool compliant)
{
  
  std::string type = entity->getEntityType();
  if (type == "Link"){
    boost::shared_ptr<const Link> downcasted_entity(boost::shared_dynamic_cast<const Link>(entity)); 
  
    TiXmlElement * link_element = new TiXmlElement( "link" );
     link_element->SetAttribute("name", entity->name);
    if(entity->name !="world"){
     if(downcasted_entity->visual)	{
	addVisual(downcasted_entity,link_element,compliant);
     }
     if(downcasted_entity->inertial)     {
	addInertial(downcasted_entity,link_element);	
      }
     if(downcasted_entity->collision)	{
     	addCollision(downcasted_entity,link_element,compliant);	
      }
    }
    robot->LinkEndChild( link_element );
    
  for (std::vector<boost::shared_ptr<BaseEntity> >::const_iterator child = downcasted_entity->child_links.begin(); child != downcasted_entity->child_links.end(); child++)
    addChildEntities(*child,robot,compliant);
    
  }
  else if (type == "Bounding_volume"){
  boost::shared_ptr<const Bounding_volume> downcasted_entity(boost::shared_dynamic_cast<const Bounding_volume>(entity));  
   
//   TiXmlElement * link_element = new TiXmlElement( "bounding_volume" );
   TiXmlElement * link_element = new TiXmlElement( "link" );
   link_element->SetAttribute("name", entity->name);
   double x,y,z;
   x = downcasted_entity->origin.position.x;
   y = downcasted_entity->origin.position.y;
   z = downcasted_entity->origin.position.z;
   double r,p,yaw;
   downcasted_entity->origin.rotation.getRPY(r,p,yaw);
   addDummyInertial(x,y,z,r,p,yaw,link_element);
   robot->LinkEndChild( link_element );
   
  for (std::vector<boost::shared_ptr<BaseEntity> >::const_iterator child = downcasted_entity->child_links.begin(); child != downcasted_entity->child_links.end(); child++)
    addChildEntities(*child,robot,compliant);
  }
}

void addParentJoint(boost::shared_ptr<const BaseEntity> entity, boost::shared_ptr<const BaseEntity> child, double &r, double &p, double &y,TiXmlElement* robot)
{
  if((child)->getEntityType() == "Link") {
    boost::shared_ptr<const Link> downcasted_child(boost::shared_dynamic_cast<const Link>((child))); 
    TiXmlElement * joint_element = new TiXmlElement( "joint" );
    joint_element->SetAttribute("name", (downcasted_child)->parent_joint->name);
    int type =  (downcasted_child)->parent_joint->type;
    if(type==FIXED)
     joint_element->SetAttribute("type","fixed");
    else if (type==PLANAR)
         joint_element->SetAttribute("type","planar");
    else if (type==FLOATING)
         joint_element->SetAttribute("type","floating");
    else if (type==PRISMATIC)
         joint_element->SetAttribute("type","prismatic");
    else if (type==CONTINUOUS)
         joint_element->SetAttribute("type","continuous");
    else if (type==REVOLUTE)
         joint_element->SetAttribute("type","revolute");

    TiXmlElement * parent = new TiXmlElement( "parent" );
    joint_element->LinkEndChild(parent);
    parent->SetAttribute("link", (downcasted_child)->parent_joint->parent_link_name);	
    TiXmlElement * child = new TiXmlElement( "child" );
    joint_element->LinkEndChild(child);
    child->SetAttribute("link", (downcasted_child)->parent_joint->child_link_name);
    
    TiXmlElement * origin = new TiXmlElement( "origin" );
    joint_element->LinkEndChild(origin);
    std::ostringstream stm; 
	stm << downcasted_child->parent_joint->parent_to_joint_origin_transform.position.x << " "
	    << downcasted_child->parent_joint->parent_to_joint_origin_transform.position.y << " "
	    << downcasted_child->parent_joint->parent_to_joint_origin_transform.position.z << " ";
	origin->SetAttribute("xyz", stm.str());
	stm.clear();
	stm.str("");
    double r,p,y;
    downcasted_child->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(r,p,y);
	stm << r << " " << p << " " << y << " ";
	origin->SetAttribute("rpy", stm.str()); 
    
  if(type!=FIXED) {  
    TiXmlElement * axis = new TiXmlElement( "axis" );
    joint_element->LinkEndChild(axis);
    stm.clear();
	  stm.str("");
    stm << downcasted_child->parent_joint->axis.x << " "
      << downcasted_child->parent_joint->axis.y << " "
      << downcasted_child->parent_joint->axis.z << " ";
    axis->SetAttribute("xyz", stm.str());
   }
   
   if((type!=FIXED)&&(type!=CONTINUOUS)) {  
    TiXmlElement * limit = new TiXmlElement( "limit" );
    joint_element->LinkEndChild(limit);
    stm.clear(); stm.str("");
    stm << downcasted_child->parent_joint->limits->effort;
    limit->SetAttribute("effort", stm.str());
    stm.clear(); stm.str("");
    stm << downcasted_child->parent_joint->limits->lower;
    limit->SetAttribute("lower", stm.str());
    stm.clear(); stm.str("");
    stm << downcasted_child->parent_joint->limits->upper;
    limit->SetAttribute("upper", stm.str());
    stm.clear(); stm.str("");
    stm << downcasted_child->parent_joint->limits->velocity;
    limit->SetAttribute("velocity", stm.str());
   }

    robot->LinkEndChild( joint_element );
    
    
    
  }
  else if ((child)->getEntityType() == "Bounding_volume") {
    boost::shared_ptr<const Bounding_volume> downcasted_child(boost::shared_dynamic_cast<const Bounding_volume>((child))); 
    TiXmlElement * joint_element = new TiXmlElement( "joint" );
    joint_element->SetAttribute("name", (downcasted_child)->parent_joint->name);
    int type =  (downcasted_child)->parent_joint->type;
    if(type==FIXED)
     joint_element->SetAttribute("type","fixed");
    else if (type==PLANAR)
         joint_element->SetAttribute("type","planar");
    else if (type==FLOATING)
         joint_element->SetAttribute("type","floating");
    else if (type==PRISMATIC)
         joint_element->SetAttribute("type","prismatic");
    else if (type==CONTINUOUS)
         joint_element->SetAttribute("type","continuous");
    else if (type==REVOLUTE)
         joint_element->SetAttribute("type","revolute");
    
    
    TiXmlElement * parent = new TiXmlElement( "parent" );
    joint_element->LinkEndChild(parent);
    parent->SetAttribute("link", (downcasted_child)->parent_joint->parent_link_name);	
    TiXmlElement * child = new TiXmlElement( "child" );
    joint_element->LinkEndChild(child);
    child->SetAttribute("link", (downcasted_child)->parent_joint->child_link_name);
    
    TiXmlElement * origin = new TiXmlElement( "origin" );
    joint_element->LinkEndChild(origin);
    std::ostringstream stm; 
	stm << downcasted_child->parent_joint->parent_to_joint_origin_transform.position.x << " "
	    << downcasted_child->parent_joint->parent_to_joint_origin_transform.position.y << " "
	    << downcasted_child->parent_joint->parent_to_joint_origin_transform.position.z << " ";
	origin->SetAttribute("xyz", stm.str());
	stm.clear();
	stm.str("");
    double r,p,y;
    downcasted_child->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(r,p,y);
	stm << r << " " << p << " " << y << " ";
	origin->SetAttribute("rpy", stm.str()); 
    robot->LinkEndChild( joint_element );	  
  }
}

void addChildJoints(boost::shared_ptr<const BaseEntity> entity,TiXmlElement* robot)
{  
  
  double r, p, y;
  std::string type = entity->getEntityType();
  if (type == "Link"){
   boost::shared_ptr<const Link> downcasted_entity(boost::shared_dynamic_cast<const Link>(entity)); 
	
      for (std::vector<boost::shared_ptr<BaseEntity> >::const_iterator child = downcasted_entity->child_links.begin(); child != downcasted_entity->child_links.end(); child++)
      {
	addParentJoint(entity,(*child),r,p,y,robot);
	addChildJoints(*child,robot);
      }
  }
  else if (type == "Bounding_volume"){
    boost::shared_ptr<const Bounding_volume> downcasted_entity(boost::shared_dynamic_cast<const Bounding_volume>(entity));  
      
      for (std::vector<boost::shared_ptr<BaseEntity> >::const_iterator child = downcasted_entity->child_links.begin(); child != downcasted_entity->child_links.end(); child++)
      {
	  addParentJoint(entity,(*child),r,p,y,robot);
	  addChildJoints(*child,robot);
      }
  }
  
}


void convertObjectInstanceToURDFfile(boost::shared_ptr<ModelInterface> object){
   

  std::string output_filename = object->getName();
  TiXmlDocument urdf_doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  urdf_doc.LinkEndChild( decl );
	
  TiXmlElement * element = new TiXmlElement( "robot" );
  urdf_doc.LinkEndChild( element );
  element->SetAttribute("name", object->getName());
  
  boost::shared_ptr<const BaseEntity> root=object->getRoot();
  bool compliant = false;
  addChildEntities(root,element,compliant); // adds all links
  addChildJoints(root,element);
  urdf_doc.SaveFile(  output_filename +".urdf");
   
}
  
std::string convertObjectInstanceToURDFstring(boost::shared_ptr<ModelInterface> object)
{
 std::string output_filename = object->getName();
  TiXmlDocument urdf_doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  urdf_doc.LinkEndChild( decl );
	
  TiXmlElement * element = new TiXmlElement( "robot" );
  urdf_doc.LinkEndChild( element );
  element->SetAttribute("name", object->getName());
  
  boost::shared_ptr<const BaseEntity> root=object->getRoot();
 bool compliant = false;
  addChildEntities(root,element,compliant); // adds all links
  addChildJoints(root,element);
  
  // urdf_doc.Print();
    std::string urdf_xml_string;
      urdf_xml_string << urdf_doc;
//cout<< urdf_xml_string << endl;
    return urdf_xml_string;    
}

std::string convertObjectInstanceToCompliantURDFstring(boost::shared_ptr<ModelInterface> object)
{
 std::string output_filename = object->getName();
  TiXmlDocument urdf_doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  urdf_doc.LinkEndChild( decl );
	
  TiXmlElement * element = new TiXmlElement( "robot" );
  urdf_doc.LinkEndChild( element );
  element->SetAttribute("name", object->getName());
  
  boost::shared_ptr<const BaseEntity> root=object->getRoot();
  bool compliant = true;
  addChildEntities(root,element,compliant); // adds all links
  addChildJoints(root,element);
  
  // urdf_doc.Print();
    std::string urdf_xml_string;
      urdf_xml_string << urdf_doc;
//cout<< urdf_xml_string << endl;
    return urdf_xml_string;
  
}

}// end namespace

