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
#ifndef OTDF_INTERFACE_MODEL_H
#define OTDF_INTERFACE_MODEL_H

#include <string>
#include <map>
#include <boost/function.hpp>

#include "link.h"
#include "joint.h"
#include "expression_parsing.h"
#include "otdf_addendum.h"

namespace otdf {

class ModelInterface
{
public:
  boost::shared_ptr<const BaseEntity> getRoot(void) const{return this->root_link_;};
  boost::shared_ptr<const Bounding_volume> getBoundingVolume(const std::string& name) const
  {
    boost::shared_ptr<const Bounding_volume> ptr;
    if (this->bounding_volumes_.find(name) == this->bounding_volumes_.end())
      ptr.reset();
    else
      ptr = this->bounding_volumes_.find(name)->second;
    return ptr;
  };
  boost::shared_ptr<const Link> getLink(const std::string& name) const
  {
    boost::shared_ptr<const Link> ptr;
    if (this->links_.find(name) == this->links_.end())
      ptr.reset();
    else
      ptr = this->links_.find(name)->second;
    return ptr;
  };
  boost::shared_ptr<const BaseEntity> getEntity(const std::string& name) const
  {
    boost::shared_ptr<const BaseEntity> ptr;
    if (this->entities_.find(name) == this->entities_.end())
      ptr.reset();
    else
      ptr = this->entities_.find(name)->second;
    return ptr;
  };

  boost::shared_ptr<const Joint> getJoint(const std::string& name) const
  {
    boost::shared_ptr<const Joint> ptr;
    if (this->joints_.find(name) == this->joints_.end())
      ptr.reset();
    else
      ptr = this->joints_.find(name)->second;
    return ptr;
  };


  const std::string& getName() const {return name_;};
  void getLinks(std::vector<boost::shared_ptr<Link> >& links) const
  {
    for (std::map<std::string,boost::shared_ptr<Link> >::const_iterator link = this->links_.begin();link != this->links_.end(); link++)
    {
      links.push_back(link->second);
    }
  };
  
  void getEntities(std::vector<boost::shared_ptr<BaseEntity> >& entities) const
  {
    for (std::map<std::string,boost::shared_ptr<BaseEntity> >::const_iterator entity = this->entities_.begin();entity != this->entities_.end(); entity++)
    {
      entities.push_back(entity->second);
    }
  };

  void clear()
  {
    name_.clear();
    this->links_.clear();
    this->bounding_volumes_.clear();
    this->entities_.clear();
    this->joints_.clear();
    this->joint_patterns_.clear();
    this->link_patterns_.clear();
    this->materials_.clear();
    this->root_link_.reset();
  };

  /// non-const getLink()
  void getLink(const std::string& name,boost::shared_ptr<Link> &link) const
  {
    boost::shared_ptr<Link> ptr;
    if (this->links_.find(name) == this->links_.end())
      ptr.reset();
    else
      ptr = this->links_.find(name)->second;
    link = ptr;
  };
  
  /// non-const getEntity()
 void getEntity(const std::string& name,boost::shared_ptr<BaseEntity> &entity) const
  {
    boost::shared_ptr<BaseEntity> ptr;
    if (this->entities_.find(name) == this->entities_.end())
      ptr.reset();
    else
      ptr = this->entities_.find(name)->second;
    entity = ptr;
  };


  /// non-const getMaterial()
  boost::shared_ptr<Material> getMaterial(const std::string& name) const
  {
    boost::shared_ptr<Material> ptr;
    if (this->materials_.find(name) == this->materials_.end())
      ptr.reset();
    else
      ptr = this->materials_.find(name)->second;
    return ptr;
  };

  /// in initXml(), onece all links are loaded,
  /// it's time to build a tree
  

  /// \brief complete list of Links
  std::map<std::string, boost::shared_ptr<Link> > links_;
  /// \brief complete list of Joints
  std::map<std::string, boost::shared_ptr<Joint> > joints_;
  /// \brief complete list of Materials
  std::map<std::string, boost::shared_ptr<Material> > materials_;
  
  /// \brief complete list of bounding volumes
  std::map<std::string, boost::shared_ptr<Bounding_volume> > bounding_volumes_;
  std::map<std::string, boost::shared_ptr<Link_pattern> > link_patterns_;
  std::map<std::string, boost::shared_ptr<Joint_pattern> > joint_patterns_;
  std::map<std::string, boost::shared_ptr<BaseEntity> > entities_; // both links and bounding_volumes
  std::string name_;

  /// ModelInterface is restricted to a tree for now, which means there exists one root link
  ///  typically, root link is the world(inertial).  Where world is a special link
  /// or is the root_link_ the link attached to the world by PLANAR/FLOATING joint?
  ///  hmm...
  boost::shared_ptr<BaseEntity> root_link_;
  
    
  ParamTable_t symbol_table;
  std::map<std::string, double> params_map_;
 
  
  double getParam(const std::string& name) const
  {
    double value;
    if (this->params_map_.find(name) == this->params_map_.end()){
      value = 0;
      //std::cerr << "ERROR: Could not find param "<< name <<" in object model "<< this->name_ << std::endl;
    }
    else
      value = this->params_map_.find(name)->second;
    return value;
  };
  
  void setParam(const std::string& name, double value) 
  {
    if (this->params_map_.find(name) == this->params_map_.end()){
      std::cerr << "ERROR: Could not find param "<< name <<" in object model "<< this->name_ << std::endl;
    }
    else
      this->params_map_[name]= value;
  };
  
  
  bool initTree(std::map<std::string, std::string> &parent_entity_tree)
  {
    // loop through all joints, for every link, assign children links and children joints
    for (std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = this->joints_.begin();joint != this->joints_.end(); joint++)
    {
      std::string parent_entity_name = joint->second->parent_link_name;
      std::string child_entity_name = joint->second->child_link_name;
      std::string parent_type = joint->second->parent_type;
      std::string child_type = joint->second->child_type;

      //ROS_DEBUG("build tree: joint: '%s' has parent link '%s' and child  link '%s'", joint->first.c_str(), parent_link_name.c_str(),child_link_name.c_str());
      if (parent_entity_name.empty() || child_entity_name.empty())
      {
        std::cerr<< "ERROR:  Joint " <<  (joint->second)->name << " is missing a parent and/or child link specification." << std::endl;
        return false;
      }
      else
      {
	//BaseEntity is a base class for bounding_volume's, link's, and link_pattern's
	boost::shared_ptr<BaseEntity>  parent_entity,child_entity;
	this->getEntity(parent_entity_name, parent_entity);
        if (!parent_entity)
        {
           std::cerr<< "ERROR:  parent entity "<< parent_entity_name <<"of joint " << joint->first <<" not found." << std::endl;
	    return false;
        }
	this->getEntity(child_entity_name, child_entity);
        if (!child_entity)
        {
           std::cerr<< "ERROR:  child entity "<< child_entity_name <<" of joint " << joint->first <<" not found." << std::endl;
	    return false;
        }

        //set parent link for child link
        child_entity->setParent(parent_entity);  

        //set parent joint for child link
        child_entity->setParentJoint(joint->second);

        //set child joint for parent link
        parent_entity->addChildJoint(joint->second);
      
        //set child link for parent link
        parent_entity->addChild(child_entity);
      //	std::cout << child_entity->name << " " << parent_entity_name << std::endl;
        // fill in child/parent string map
        parent_entity_tree[child_entity->name] = parent_entity_name;
      
        //ROS_DEBUG("    now Link '%s' has %i children ", parent_link->name.c_str(), (int)parent_link->child_links.size());
      }
    }
        
     //for link and joint patterns this has to be handled differently
     // loop through all joints in a joint pattern and link them to their associated parent entity and link_pattern child links.
    for (std::map<std::string,boost::shared_ptr<Joint_pattern> >::iterator joint_pattern = this->joint_patterns_.begin();joint_pattern != this->joint_patterns_.end(); joint_pattern++)
    {
       std::string parent_entity_name = joint_pattern->second->parent_link_name;
       std::string parent_type = joint_pattern->second->parent_type;
       std::string child_link_pattern_name = joint_pattern->second->child_link_pattern_name;
       std::map<std::string,boost::shared_ptr<Link_pattern> >::iterator it;
       it = this->link_patterns_.find(child_link_pattern_name);
       if (!it->second)
       {
           std::cerr<< "ERROR:  child link pattern  "<< child_link_pattern_name <<" of joint pattern " << joint_pattern->first <<" not found." << std::endl;
	    return false;
       }
       it->second->setParentJointPattern(joint_pattern->second);

      if (parent_entity_name.empty() || child_link_pattern_name.empty())
      {
        std::cerr<< "ERROR:  Joint pattern " <<  (joint_pattern->second)->name << " is missing a parent and/or child link pattern specification." << std::endl;
        return false;
      }
      else
      {
	//BaseEntity is a base class for bounding_volume's, link's
       boost::shared_ptr<BaseEntity>  parent_entity;
 
	this->getEntity(parent_entity_name, parent_entity);
        if (!parent_entity)
        {
           std::cerr<< "ERROR:  parent entity "<< parent_entity_name <<"of joint pattern" << joint_pattern->first <<" not found." << std::endl;
	    return false;
        }
     
        //set parent link for child link pattern
         it->second->setParent(parent_entity); 
       }
      // Now update all child links for all joints in joint pattern.
       for (unsigned int i=0; i < joint_pattern->second->joint_set.size(); i++)
       {
	 
 	  std::string parent_entity_name = joint_pattern->second->joint_set[i]->parent_link_name;
	  std::string child_entity_name = joint_pattern->second->joint_set[i]->child_link_name;
	  std::string parent_type = joint_pattern->second->joint_set[i]->parent_type;
 	  std::string child_type = joint_pattern->second->joint_set[i]->child_type;
 
 	  //ROS_DEBUG("build tree: joint: '%s' has parent link '%s' and child  link '%s'", joint->first.c_str(), parent_link_name.c_str(),child_link_name.c_str());
 	  if (parent_entity_name.empty() || child_entity_name.empty())
 	  {
 	    std::cerr<< "ERROR:  Joint " <<  joint_pattern->second->joint_set[i]->name << " is missing a parent and/or child link specification." << std::endl;
 	    return false;
 	  }
 	  else
 	  {
	    //BaseEntity is a base class for bounding_volume's, link's, and link_pattern's
	    boost::shared_ptr<BaseEntity>  parent_entity,child_entity;
	    this->getEntity(parent_entity_name, parent_entity);
	    if (!parent_entity)
	    {
	      std::cerr<< "ERROR:  parent entity "<< parent_entity_name <<"of joint " << joint_pattern->second->joint_set[i]->name <<" not found." << std::endl;
		return false;
	    }
	 
	   // this->getEntity(child_entity_name, child_entity);
	    child_entity = it->second->link_set[i];
	  
	    if (!child_entity)
	    {
	      std::cerr<< "ERROR:  child entity "<< child_entity_name <<" of joint " << joint_pattern->second->joint_set[i]->name <<" not found." << std::endl;
		return false;
	    }
	    //set parent link for child link
	    child_entity->setParent(parent_entity);  

	    //set parent joint for child link
	    child_entity->setParentJoint(joint_pattern->second->joint_set[i]);

	    //set child joint for parent link
	    parent_entity->addChildJoint(joint_pattern->second->joint_set[i]);
	  
	    //set child link for parent link
	    parent_entity->addChild(child_entity);
	   // std::cout << child_entity->name << " " << parent_entity_name << std::endl;
	    // fill in child/parent string map
	    parent_entity_tree[child_entity->name] = parent_entity_name;
	  }
       } // for joints in joint pattern
    }  // for all joints patterns
     
    return true;
  };

  /// in initXml(), onece tree is built,
  /// it's time to find the root Link
  bool initRoot(std::map<std::string, std::string> &parent_entity_tree)
  {
    this->root_link_.reset();

    // find the links that have no parent in the tree
    for (std::map<std::string, boost::shared_ptr<BaseEntity> >::iterator l=this->entities_.begin(); l!=this->entities_.end(); l++)  
    {
	if(l->second->getEntityType()!="Link_pattern"){
	    std::map<std::string, std::string >::iterator parent = parent_entity_tree.find(l->first);
	    if (parent == parent_entity_tree.end())
	    {
      // 	std::cout << "END" << l->first << " " << l->second->getEntityType() << std::endl;
	      // store root link
	      if (!this->root_link_)
	      {
		getEntity(l->first, this->root_link_);
	      }
	      // we already found a root link
	      else{
		//ROS_ERROR("Two root links found: '%s' and '%s'", this->root_link_->name.c_str(), l->first.c_str());
	      std::cerr << "ERROR: Two root links found, " << this->root_link_->name << " and " << l->first << std::endl;
	      return false;
	      }
	    }
	} // Cannot have a pattern as a root link
     
//      else if(l->second->getEntityType()=="Link_pattern"){
//        boost::shared_ptr<Link_pattern> downcasted_entity(boost::shared_dynamic_cast<Link_pattern>(l->second)); 
//    
//        for (unsigned int i=0; i < downcasted_entity->link_set.size(); i++)
//        {
// 	std::map<std::string, std::string >::iterator parent = parent_entity_tree.find(downcasted_entity->link_set[i]->name);
// 	if (parent == parent_entity_tree.end())
// 	{
// 	  // store root link
// 	  if (!this->root_link_)
// 	  {
// 	    getEntity(l->first, this->root_link_);
// 	  }
// 	  // we already found a root link
// 	  else{
// 	    //ROS_ERROR("Two root links found: '%s' and '%s'", this->root_link_->name.c_str(), l->first.c_str());
// 	  std::cerr << "ERROR: Two root links found, " << this->root_link_->name << " and " << l->first << std::endl;
// 	  return false;
// 	  }
// 	}
//        }// end link_pattern loop
//       } // end else if.

    }
    if (!this->root_link_)
    {
      std::cerr << "ERROR: No root link found. The robot xml is not a valid tree." << std::endl;
      return false;
    }
    return true;
  };
  
  void update()
  {
    // Update patterns (Can grow and shrink)
    // loop through all joints in joint patterns
    // update all in child link patterns
    for (std::map<std::string,boost::shared_ptr<Joint_pattern> >::iterator joint_pattern = this->joint_patterns_.begin();joint_pattern != this->joint_patterns_.end(); joint_pattern++)
    {

       joint_pattern->second->update();
	std::string child_link_pattern_name = joint_pattern->second->child_link_pattern_name;
       std::map<std::string,boost::shared_ptr<Link_pattern> >::iterator it;
       it = this->link_patterns_.find(child_link_pattern_name);
       it->second->update();

    }
      

    if(this->root_link_->getEntityType()=="Link"){
     boost::shared_ptr<Link> downcasted_entity(boost::shared_dynamic_cast<Link>(this->root_link_)); 
     downcasted_entity->update();
     }
     else if(this->root_link_->getEntityType()=="Bounding_volume"){
     boost::shared_ptr<Bounding_volume> downcasted_entity(boost::shared_dynamic_cast<Bounding_volume>(this->root_link_)); 
     downcasted_entity->update();
     }

    // loop through all joints, update joints and update all children links that are not included in a pattern.
    for (std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = this->joints_.begin();joint != this->joints_.end(); joint++)
    {
//       std::string parent_entity_name = joint->second->parent_link_name;
//       std::string parent_type = joint->second->parent_type;
      joint->second->update();
      std::string child_entity_name = joint->second->child_link_name;
      std::string child_type = joint->second->child_type;

	//BaseEntity is a base class for bounding_volume's, link's, and link_pattern's
// 	boost::shared_ptr<BaseEntity>  parent_entity,child_entity;
// 	this->getEntity(parent_entity_name, parent_entity);
// 	this->getEntity(child_entity_name, child_entity);
// 	parent_entity.update();
// 	child_entity.update();
      if(child_type!="link_pattern") {
      	boost::shared_ptr<BaseEntity>  child_entity;
	this->getEntity(child_entity_name, child_entity);
	child_entity->update();
      }
    }
    clearTreeStructure();
    std::map<std::string, std::string> parent_entity_tree;
    parent_entity_tree.clear();
   // DO YOU NEED TO INIT TREE AND INIT ROOT?
  // building tree: name mapping
    if (!initTree(parent_entity_tree))
   {
      std::cerr<< "ERROR: failed to build tree from otdf file."<< std::endl;
   }

  }; // end update

  bool clearTreeStructure()
  {
    // loop through all joints, for every link, assign children links and children joints
    for (std::map<std::string,boost::shared_ptr<Joint> >::iterator joint = this->joints_.begin();joint != this->joints_.end(); joint++)
    {
      std::string parent_entity_name = joint->second->parent_link_name;
      std::string child_entity_name = joint->second->child_link_name;
      std::string parent_type = joint->second->parent_type;
      std::string child_type = joint->second->child_type;

      //ROS_DEBUG("build tree: joint: '%s' has parent link '%s' and child  link '%s'", joint->first.c_str(), parent_link_name.c_str(),child_link_name.c_str());
      if (parent_entity_name.empty() || child_entity_name.empty())
      {
        std::cerr<< "ERROR:  Joint " <<  (joint->second)->name << " is missing a parent and/or child link specification." << std::endl;
        return false;
      }
      else
      {
	//BaseEntity is a base class for bounding_volume's, link's, and link_pattern's
	boost::shared_ptr<BaseEntity>  parent_entity,child_entity;
	this->getEntity(parent_entity_name, parent_entity);
        if (!parent_entity)
        {
           std::cerr<< "ERROR:  parent entity "<< parent_entity_name <<"of joint " << joint->first <<" not found." << std::endl;
	    return false;
        }
	this->getEntity(child_entity_name, child_entity);
        if (!child_entity)
        {
           std::cerr<< "ERROR:  child entity "<< child_entity_name <<" of joint " << joint->first <<" not found." << std::endl;
	    return false;
        }
        parent_entity->clearChildEntities();
	parent_entity->clearChildJoints();
        child_entity->clearChildEntities();	
	child_entity->clearChildJoints();
       }
    }
        
     //for link and joint patterns this has to be handled differently
     // loop through all joints in a joint pattern and link them to their associated parent entity and link_pattern child links.
    for (std::map<std::string,boost::shared_ptr<Joint_pattern> >::iterator joint_pattern = this->joint_patterns_.begin();joint_pattern != this->joint_patterns_.end(); joint_pattern++)
    {
       std::string parent_entity_name = joint_pattern->second->parent_link_name;
       std::string parent_type = joint_pattern->second->parent_type;
       std::string child_link_pattern_name = joint_pattern->second->child_link_pattern_name;
       std::map<std::string,boost::shared_ptr<Link_pattern> >::iterator it;
       it = this->link_patterns_.find(child_link_pattern_name);
       if (!it->second)
       {
           std::cerr<< "ERROR:  child link pattern  "<< child_link_pattern_name <<" of joint pattern " << joint_pattern->first <<" not found." << std::endl;
	    return false;
       }
       it->second->setParentJointPattern(joint_pattern->second);

      if (parent_entity_name.empty() || child_link_pattern_name.empty())
      {
        std::cerr<< "ERROR:  Joint pattern " <<  (joint_pattern->second)->name << " is missing a parent and/or child link pattern specification." << std::endl;
        return false;
      }
      else
      {
	//BaseEntity is a base class for bounding_volume's, link's
       boost::shared_ptr<BaseEntity>  parent_entity;
 
	this->getEntity(parent_entity_name, parent_entity);
        if (!parent_entity)
        {
           std::cerr<< "ERROR:  parent entity "<< parent_entity_name <<"of joint pattern" << joint_pattern->first <<" not found." << std::endl;
	    return false;
        }
     
        //set parent link for child link pattern
         it->second->setParent(parent_entity); 
       }
      // Now update all child links for all joints in joint pattern.
       for (unsigned int i=0; i < joint_pattern->second->joint_set.size(); i++)
       {
	 
 	  std::string parent_entity_name = joint_pattern->second->joint_set[i]->parent_link_name;
	  std::string child_entity_name = joint_pattern->second->joint_set[i]->child_link_name;
	  std::string parent_type = joint_pattern->second->joint_set[i]->parent_type;
 	  std::string child_type = joint_pattern->second->joint_set[i]->child_type;
 
 	  //ROS_DEBUG("build tree: joint: '%s' has parent link '%s' and child  link '%s'", joint->first.c_str(), parent_link_name.c_str(),child_link_name.c_str());
 	  if (parent_entity_name.empty() || child_entity_name.empty())
 	  {
 	    std::cerr<< "ERROR:  Joint " <<  joint_pattern->second->joint_set[i]->name << " is missing a parent and/or child link specification." << std::endl;
 	    return false;
 	  }
 	  else
 	  {
	    //BaseEntity is a base class for bounding_volume's, link's, and link_pattern's
	    boost::shared_ptr<BaseEntity>  parent_entity,child_entity;
	    this->getEntity(parent_entity_name, parent_entity);
	    if (!parent_entity)
	    {
	      std::cerr<< "ERROR:  parent entity "<< parent_entity_name <<"of joint " << joint_pattern->second->joint_set[i]->name <<" not found." << std::endl;
		return false;
	    }
	 
	   // this->getEntity(child_entity_name, child_entity);
	    child_entity = it->second->link_set[i];
	  
	    if (!child_entity)
	    {
	      std::cerr<< "ERROR:  child entity "<< child_entity_name <<" of joint " << joint_pattern->second->joint_set[i]->name <<" not found." << std::endl;
		return false;
	    }
	      parent_entity->clearChildEntities();
	      parent_entity->clearChildJoints();
	      child_entity->clearChildEntities();	
	      child_entity->clearChildJoints();
	  }
       } // for joints in joint pattern
    }  // for all joints patterns
     
    return true;
  };
  
  
  
}; // end class

}

#endif
	/*
        // find parent links
       if(parent_type == "link")
       {
	boost::shared_ptr<Link>  parent_link;
	this->getLink(parent_link_name, parent_link);
        if (!parent_link)
        {
           std::cerr<< "ERROR:  parent link "<< parent_link_name <<"of joint " << joint->first <<" not found." << std::endl;
	    return false;
        }
	 
       }
       else if (parent_type == "bounding_volume")
       {
	 boost::shared_ptr<Bounding_volume>  parent_link; 
	this->getBoundingVolume(parent_link_name, parent_link);
        if (!parent_link)
        {
           std::cerr<< "ERROR:  parent bounding_volume "<< parent_link_name <<"of joint " << joint->first <<" not found." << std::endl;
	    return false;
        } 
       }
       else{
	  std::cerr<< "ERROR:  unknown parent type "<< parent_type << "of joint " << joint->first << std::endl;
	  return false;
       }
       
       
         // find child links
       if(child_type == "link")
       {
	boost::shared_ptr<Link>  child_link;
	this->getLink(child_link_name, child_link);
        if (!child_link)
        {
           std::cerr<< "ERROR:  child link "<< child_link_name <<"of joint " << joint->first <<" not found." << std::endl;
	    return false;
        }
	 
       }
       else if (child_type == "bounding_volume")
       {
	 boost::shared_ptr<Bounding_volume>  child_link; 
	this->getBoundingVolume(child_link_name, child_link);
        if (!child_link)
        {
           std::cerr<< "ERROR:  child bounding_volume "<< child_link_name <<"of joint " << joint->first <<" not found." << std::endl;
	    return false;
        } 
       }
       else{
	  std::cerr<< "ERROR:  unknown child type "<< child_type << "of joint " << joint->first << std::endl;
	  return false;
       }*/