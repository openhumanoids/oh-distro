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
*   * Redstributions of source code must retain the above copyright
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

#include "otdf_parser/otdf_parser.h"
#include "otdf_parser/otdf_urdf_converter.h"

#include <iostream>
#include <fstream>

using namespace otdf;
using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2){
    std::cerr << "Usage: otdf_2_xacro_urdf input.otdf" << std::endl;
    return -1;
  }

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(argv[1], std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
 // cout << xml_string << endl;
  
  
  boost::shared_ptr<ModelInterface> object = parseOTDF(xml_string);
  if (!object){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }
  
  //testing object update for ladder
  //object->setParam("NO_OF_STEPS", 5);
  //object->update();

  convertObjectInstanceToURDFfile(object);
  
 string output = object->getName();
 cout << "Created file " << output << ".urdf" << endl;

//   string urdf_xml_string=convertObjectInstanceToURDFstring(object);
//   cout<< urdf_xml_string << endl;
  
  return 0;
}

