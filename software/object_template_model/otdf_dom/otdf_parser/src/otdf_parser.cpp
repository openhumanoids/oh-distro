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
#include <boost/algorithm/string.hpp>
#include <vector>
#include "otdf_parser/otdf_parser.h"
#include "otdf_interface/exceptions.h"



namespace otdf{

boost::shared_ptr<ModelInterface>  parseOTDF(const std::string &xml_string)
{
  boost::shared_ptr<ModelInterface> model(new ModelInterface);
  model->clear();

  TiXmlDocument xml_doc;
  xml_doc.Parse(xml_string.c_str());

  TiXmlElement *object_xml = xml_doc.FirstChildElement("object");

  if (!object_xml)
  {
    std::cerr<< "ERROR: Could not find the 'object' element in the xml file"<< std::endl;
    model.reset();
    return model;
  }

  // Get object name
  const char *name = object_xml->Attribute("name");
  if (!name)
  {
    std::cerr<< "ERROR: No name given for the object template."<< std::endl;
    model.reset();
    return model;
  }
  model->name_ = std::string(name);

  // Get all params elements and add to param_map_
  for (TiXmlElement* param_xml = object_xml->FirstChildElement("param"); param_xml; param_xml = param_xml->NextSiblingElement("param"))
  {
   boost::shared_ptr<Param> param;
	 param.reset(new Param);  
   try {
      param->initXml(param_xml);
      // Add param to a parammap_
      if (model->getParam(param->name))
      {
        //ROS_ERROR("material '%s' is not unique.", material->name.c_str());
        std::cerr<< "ERROR: param" << param->name <<"is not unique."<< std::endl;   
	param.reset();
        model.reset();
        return model;
      }
      else
      {
        model->params_map_.insert(make_pair(param->name,param->value));
	
        // Add to symbol table
        model->symbol_table.add_variable(param->name,model->params_map_[param->name]);
        //ROS_DEBUG("successfully added a new material '%s'", material->name.c_str());
      }
    }
    catch (ParseError &e) {
      //ROS_ERROR("material xml is not initialized correctly");
       std::cerr<< "ERROR: param xml is not initialized correctly"<< std::endl;   
	
      param.reset();
      model.reset();
      return model;
    }	  
  }
  model->symbol_table.add_constants();


  // Get all Material elements
  for (TiXmlElement* material_xml = object_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
  {
    boost::shared_ptr<Material> material;
    material.reset(new Material);

    try {
      material->initXml(material_xml,model->symbol_table);
      if (model->getMaterial(material->name))
      {
        //ROS_ERROR("material '%s' is not unique.", material->name.c_str());
        std::cerr<< "ERROR: material" << material->name <<"is not unique."<< std::endl;   
	material.reset();
        model.reset();
        return model;
      }
      else
      {
        model->materials_.insert(make_pair(material->name,material));
        //ROS_DEBUG("successfully added a new material '%s'", material->name.c_str());
      }
    }
    catch (ParseError &e) {
      //ROS_ERROR("material xml is not initialized correctly");
       std::cerr<< "ERROR: material xml is not initialized correctly"<< std::endl;   
	
      material.reset();
      model.reset();
      return model;
    }
  }


  // Get all Link elements
  for (TiXmlElement* link_xml = object_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
//     std::cout<< link_xml->Attribute("name") << std::endl;
    boost::shared_ptr<Link> link;
    link.reset(new Link);

  
    try {
      link->initXml(link_xml,model->symbol_table);

      if (model->getLink(link->name))
      {
        //ROS_ERROR("link '%s' is not unique.", link->name.c_str());
        std::cerr<< "ERROR: link" << link->name <<"is not unique."<< std::endl;   
        model.reset();
        return model;
      }
      else
      {
        // set link visual material
        //ROS_DEBUG("setting link '%s' material", link->name.c_str());
        if (link->visual)
        {
          if (!link->visual->material_name.empty())
          {
            if (model->getMaterial(link->visual->material_name))
            {
              //ROS_DEBUG("setting link '%s' material to '%s'", link->name.c_str(),link->visual->material_name.c_str());
              link->visual->material = model->getMaterial( link->visual->material_name.c_str() );
            }
            else
            {
              if (link->visual->material)
              {
                //ROS_DEBUG("link '%s' material '%s' defined in Visual.", link->name.c_str(),link->visual->material_name.c_str());
                model->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
              }
              else
              {
                //ROS_ERROR("link '%s' material '%s' undefined.", link->name.c_str(),link->visual->material_name.c_str());
                std::cerr<< "ERROR: link" << link->name <<" material " << link->visual->material_name << " is not unique."<< std::endl;   
                model.reset();
                return model;
              }
            }
          }
        }

        model->links_.insert(make_pair(link->name,link));
	model->entities_.insert(make_pair(link->name,link));
        //ROS_DEBUG("successfully added a new link '%s'", link->name.c_str());
      }
    }
    catch (ParseError &e) {
      //ROS_ERROR("link xml is not initialized correctly");
      std::cerr<< "ERROR: link xml is not initialized correctly"<< std::endl;  
      model.reset();
      return model;
    }
  }
 
  // Get all Link elements in a link_pattern
  for (TiXmlElement* link_pattern_xml = object_xml->FirstChildElement("link_pattern"); link_pattern_xml; link_pattern_xml = link_pattern_xml->NextSiblingElement("link_pattern"))
  {
//     std::cout<< link_pattern_xml->Attribute("name") << std::endl;
    boost::shared_ptr<Link_pattern> link_pattern;
    link_pattern.reset(new Link_pattern);

    try {
            
          link_pattern->initXml(link_pattern_xml,model->symbol_table);

//          //Loop through all links in the link pattern acc to noofrepetitions
//          for  (unsigned int i=0; i < link_pattern->noofrepetitions; i++)
//          {
//             boost::shared_ptr<Link> link;
//             link.reset(new Link(*link_pattern->link_set[i]));
//         
//             if (model->getLink(link->name))
//             {
//               //ROS_ERROR("link '%s' is not unique.", link->name.c_str());
//               std::cerr<< "ERROR: link in link_pattern" << link->name <<"is not unique."<< std::endl;   
//               model.reset();
//               return model;
//             }
//             else
//             {
//               // set link visual material
//               //ROS_DEBUG("setting link '%s' material", link->name.c_str());
//               if (link->visual)
//               {
//                 if (!link->visual->material_name.empty())
//                 {
//                   if (model->getMaterial(link->visual->material_name))
//                   {
//                     //ROS_DEBUG("setting link '%s' material to '%s'", link->name.c_str(),link->visual->material_name.c_str());
//                     link->visual->material = model->getMaterial( link->visual->material_name.c_str() );
//                   }
//                   else
//                   {
//                     if (link->visual->material)
//                     {
//                       //ROS_DEBUG("link '%s' material '%s' defined in Visual.", link->name.c_str(),link->visual->material_name.c_str());
//                       model->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
//                     }
//                     else
//                     {
//                       //ROS_ERROR("link '%s' material '%s' undefined.", link->name.c_str(),link->visual->material_name.c_str());
//                       std::cerr<< "ERROR: link in link pattern" << link->name <<" material " << link->visual->material_name << " is not unique."<< std::endl;   
//                       model.reset();
//                       return model;
//                     }
//                   }//end if (model->getMaterial(link->visual->material_name))
//                 }//end if (!link->visual->material_name.empty())
//               }//end if (link->visual)
//               
//               model->links_.insert(make_pair(link->name,link));
// 	      model->entities_.insert(make_pair(link->name,link));
// 
//             }  //end if (model->getLink( ))
//             }// end for  (unsigned int i=0; i < noofrepetitions; i++)
         model->entities_.insert(make_pair(link_pattern->name,link_pattern));   
         model->link_patterns_.insert(make_pair(link_pattern->name,link_pattern));  
            
    }
    catch (ParseError &e) {
      std::cerr<< "ERROR: link pattern xml is not initialized correctly"<< std::endl;  
      model.reset();
      return model;
    }
  }// for (TiXmlElement* link_pattern_xml 
  
  if (model->links_.empty()){
    std::cerr<< "ERROR: No link elements found in otdf file."<< std::endl;
    model.reset();
    return model;
  }

   
 // Get all Bounding_volume elements
  for (TiXmlElement* bvolume_xml = object_xml->FirstChildElement("bounding_volume"); bvolume_xml; bvolume_xml = bvolume_xml->NextSiblingElement("bounding_volume"))
  {
    boost::shared_ptr<Bounding_volume> bounding_volume;
    bounding_volume.reset(new Bounding_volume);
    
    try {
      bounding_volume->initXml(bvolume_xml,model->symbol_table);
      if (model->getBoundingVolume(bounding_volume->name))
      {
        //ROS_ERROR("link '%s' is not unique.", link->name.c_str());
        std::cerr<< "ERROR: bounding_volume" << bounding_volume->name <<" is not unique."<< std::endl;   
        model.reset();
        return model;
      }
      else
      {
        model->bounding_volumes_.insert(make_pair(bounding_volume->name,bounding_volume));
	model->entities_.insert(make_pair(bounding_volume->name,bounding_volume));
        //ROS_DEBUG("successfully added a new link '%s'", link->name.c_str());
      }
    }
    catch (ParseError &e) {
      //ROS_ERROR("link xml is not initialized correctly");
      std::cerr<< "ERROR: bounding_volume xml is not initialized correctly"<< std::endl;  
      model.reset();
      return model;
    }
  }

  // Get all Joint elements
  for (TiXmlElement* joint_xml = object_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (joint->initXml(joint_xml,model->symbol_table))
    {
      if (model->getJoint(joint->name))
      {
        //ROS_ERROR("joint '%s' is not unique.", joint->name.c_str());
         std::cerr<< "ERROR: material" << joint->name <<"is not unique."<< std::endl;   
        model.reset();
        return model;
      }
      else
      {
        model->joints_.insert(make_pair(joint->name,joint));
        //ROS_DEBUG("successfully added a new joint '%s'", joint->name.c_str());
      }
    }
    else
    {
      //ROS_ERROR("joint xml is not initialized correctly");
      std::cerr<< "ERROR: joint xml is not initialized correctly"<< std::endl;  
      model.reset();
      return model;
    }
  }

  // Get all Joint elements in a joint_pattern and add them to model
  for (TiXmlElement* joint_pattern_xml = object_xml->FirstChildElement("joint_pattern"); joint_pattern_xml; joint_pattern_xml = joint_pattern_xml->NextSiblingElement("joint_pattern"))
  {
  
    boost::shared_ptr<Joint_pattern> joint_pattern;
    joint_pattern.reset(new Joint_pattern);

    if (joint_pattern->initXml(joint_pattern_xml,model->symbol_table))
    {
        //Loop through all joints in the joint pattern acc to noofrepetitions
//         for(unsigned int i=0; i < joint_pattern->noofrepetitions; i++)
//         {
//           boost::shared_ptr<Joint> joint;
//           joint.reset(new Joint(*joint_pattern->joint_set[i]));
//         
//           if (model->getJoint(joint->name))
//           {
//             //ROS_ERROR("joint '%s' is not unique.", joint->name.c_str());
//              std::cerr<< "ERROR: material" << joint->name <<"is not unique."<< std::endl;   
//             model.reset();
//             return model;
//           }
//           else
//           {
//             model->joints_.insert(make_pair(joint->name,joint)); 
//             //ROS_DEBUG("successfully added a new joint '%s'", joint->name.c_str());
//           }
//         } // end for joint_set
        model->joint_patterns_.insert(make_pair(joint_pattern->name,joint_pattern));  
    }
    else
    {
      //ROS_ERROR("joint xml is not initialized correctly");
      std::cerr<< "ERROR: joint pattern xml is not initialized correctly"<< std::endl;  
      model.reset();
      return model;
    }
  }
 

  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::map<std::string, std::string> parent_entity_tree;
  parent_entity_tree.clear();

  // building tree: name mapping
  if (!model->initTree(parent_entity_tree))
  {
    std::cerr<< "ERROR: failed to build tree from otdf file."<< std::endl;
    model.reset();
    return model;
  }
//   std::cout << "OK! initTree" << std::endl;

  // find the root link
  if (!model->initRoot(parent_entity_tree))
  {
    //ROS_ERROR("failed to find root link");
     std::cerr<< "ERROR: failed to find root link in otdf file."<< std::endl;
    model.reset();
    return model;
  }
//    std::cout << "OK! initRoot" << std::endl;
  return model;
}

}

