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
// Modified from ROS's URDF_DOM library Written by John Hsu, Wim Meeussen and  Josh Faust
#include "otdf_parser/otdf_parser.h"
#include <iostream>
#include <fstream>

using namespace otdf;

void printTree(boost::shared_ptr<const BaseEntity> entity,int level = 0)
{
  level+=2;
  int count = 0;
  std::string type = entity->getEntityType();
  if (type == "Link"){
    boost::shared_ptr<const Link> downcasted_entity(boost::shared_dynamic_cast<const Link>(entity)); 
      
  for (std::vector<boost::shared_ptr<BaseEntity> >::const_iterator child = downcasted_entity->child_links.begin(); child != downcasted_entity->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << downcasted_entity->name << " has a null child!" << *child << std::endl;
    }
  }
    
  } 
  else if (type == "Bounding_volume"){
    boost::shared_ptr<const Bounding_volume> downcasted_entity(boost::shared_dynamic_cast<const Bounding_volume>(entity));  
    
      
  for (std::vector<boost::shared_ptr<BaseEntity> >::const_iterator child = downcasted_entity->child_links.begin(); child != downcasted_entity->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << downcasted_entity->name << " has a null child!" << *child << std::endl;
    }
  }
  }


}


int main(int argc, char** argv)
{
  if (argc < 2){
    std::cerr << "Expect OTDF xml file to parse" << std::endl;
    return -1;
  }

  std::string xml_string;
  std::fstream xml_file(argv[1], std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  boost::shared_ptr<ModelInterface> object = parseOTDF(xml_string);
  if (!object){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }
  std::cout << "object name is: " << object->getName() << std::endl;

  // get info from parser
  std::cout << "---------- Successfully Parsed XML ---------------" << std::endl;
  // get root link
  boost::shared_ptr<const BaseEntity> root_entity=object->getRoot();
  if (!root_entity) return -1;
  
 std::string type = root_entity->getEntityType();
  if (type == "Link"){
    boost::shared_ptr<const Link> downcasted_root_entity(boost::shared_dynamic_cast<const Link>(root_entity)); 
      
  std::cout << "root_entity: " << downcasted_root_entity->name << " has " << downcasted_root_entity->child_links.size() << " child(ren)" << std::endl;

  }
  else if(type == "Bounding_volume"){
    boost::shared_ptr<const Bounding_volume> downcasted_root_entity(boost::shared_dynamic_cast<const Bounding_volume>(root_entity)); 
      
  std::cout << "root_entity: " << downcasted_root_entity->name << " has " << downcasted_root_entity->child_links.size() << " child(ren)" << std::endl;

  }

  // print entire tree
  printTree(root_entity);
  
  if(object->getName()=="Ladder1"){ // for testing.
    std::cout << "Changing Param NO_OF_STEPS to : " << 5 << std::endl;
    object->setParam("NO_OF_STEPS", 5);
    object->update();

  std::cout << "---------- Successfully updated object ---------------" << std::endl;
    // get root link
    root_entity=object->getRoot();
    if (!root_entity) return -1;
    
    type = root_entity->getEntityType();
    if (type == "Link"){
      boost::shared_ptr<const Link> downcasted_root_entity(boost::shared_dynamic_cast<const Link>(root_entity)); 
	
    std::cout << "root_entity: " << downcasted_root_entity->name << " has " << downcasted_root_entity->child_links.size() << " child(ren)" << std::endl;

    }
    else if(type == "Bounding_volume"){
      boost::shared_ptr<const Bounding_volume> downcasted_root_entity(boost::shared_dynamic_cast<const Bounding_volume>(root_entity)); 
	
    std::cout << "root_entity: " << downcasted_root_entity->name << " has " << downcasted_root_entity->child_links.size() << " child(ren)" << std::endl;
    }
      // print entire tree after update
    printTree(root_entity);
  }
  
  return 0;
}

// #include <otdf_parser/otdf_parser.h>
//  boost::shared_ptr<ModelInterface> object = parseOTDF(xml_string);
//  std::map<std::string, double> params = object->params_map_; // all parameters define in the object template
//  // Updating.
//  object->setParam("X", 10);
//  object->update();
 
