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
#ifndef OTDF_INTERFACE_POSE_H
#define OTDF_INTERFACE_POSE_H

#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <otdf_interface/exceptions.h>
#include "expression_parsing.h"
#include <tinyxml.h> // FIXME: remove parser from here

namespace otdf{

class Vector3
{
public:
  Vector3(double _x,double _y, double _z) {
    this->x=_x;    this->y=_y;    this->z=_z;
    local_expressions.clear();
    expression_flags.clear(); 
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression);// initialise local_expressions
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false); 
  };
  Vector3() {this->clear();};
  double x;
  double y;
  double z;
  
  std::vector<bool> expression_flags;
  std::vector<exprtk::expression<double> >  local_expressions;

  void clear() {
    local_expressions.clear();
    expression_flags.clear(); 
    exprtk::expression<double> expression; 
    exprtk::parser<double> parser;
    parser.compile("1",expression);
    local_expressions.push_back(expression);// initialise local_expressions
    local_expressions.push_back(expression);
    local_expressions.push_back(expression);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    this->x=this->y=this->z=0.0;
  };
  void init(const std::string &vector_str ,ParamTable_t &symbol_table)
  {

    this->clear();
    std::vector<std::string> pieces;
    std::vector<double> xyz;
    //TODO: equations can have white spaces
    
    std::string vector_of_expressions;
    vector_of_expressions=removeWhiteSpaceInExpressions(vector_str);
    boost::split( pieces, vector_of_expressions, boost::is_any_of(" "));

//     std::cout  << "pieces: " << vector_str << std::endl;

     
    for (unsigned int i = 0; i < pieces.size(); ++i){
    exprtk::expression<double> expression;  
      if (pieces[i] != "")
      {
        if(!isExpression(pieces[i]))
	{	  
	  try
	  {
	    xyz.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
	  }
	  catch (boost::bad_lexical_cast &e)
	  {
	     throw ParseError("Vector3 xyz element ("+ pieces[i] +") is not a valid float");
	  }
	  
	}
	else
	{
// 	  std::cout << "expression "<< i << " : " << pieces[i] << std::endl;
	  expression_flags[i] = true;
	 
	  if(compileExpression(pieces[i], symbol_table, expression))
	  {
	    this->local_expressions[i] = expression;
	    double value = local_expressions[i].value();
	    //double value = expression.value();
	    xyz.push_back(value);
	  }
	}
      }
    }

    if (xyz.size() != 3) {
      std::stringstream stm;
      stm << "Vector contains " << xyz.size()  << "elements instead of 3 elements";
      throw ParseError(stm.str());
    }

    this->x = xyz[0];
    this->y = xyz[1];
    this->z = xyz[2];

  };
  Vector3 operator+(Vector3 vec)
  {
    return Vector3(this->x+vec.x,this->y+vec.y,this->z+vec.z);
  };
  void update()
  {
      std::vector<float> xyz;
     xyz[0] =   this->x;
     xyz[1] =   this->y;
     xyz[2] =   this->z;
      for (unsigned int i = 0; i < xyz.size(); ++i)
      {
	if(expression_flags[i])
	{
	  xyz[i] = local_expressions[i].value();
	}
      }      
    this->x = xyz[0];
    this->y = xyz[1];
    this->z = xyz[2];	
  };
};

class Rotation
{
public:
  Rotation(double _x,double _y, double _z, double _w) {this->x=_x;this->y=_y;this->z=_z;this->w=_w;};
  Rotation() {this->clear();};
  void getQuaternion(double &quat_x,double &quat_y,double &quat_z, double &quat_w) const
  {
    quat_x = this->x;
    quat_y = this->y;
    quat_z = this->z;
    quat_w = this->w;
  };
  void getRPY(double &roll,double &pitch,double &yaw) const
  {
    double sqw;
    double sqx;
    double sqy;
    double sqz;

    sqx = this->x * this->x;
    sqy = this->y * this->y;
    sqz = this->z * this->z;
    sqw = this->w * this->w;

    roll  = atan2(2 * (this->y*this->z + this->w*this->x), sqw - sqx - sqy + sqz);
    double sarg = -2 * (this->x*this->z - this->w*this->y);
    pitch = sarg <= -1.0 ? -0.5*M_PI : (sarg >= 1.0 ? 0.5*M_PI : asin(sarg));
    yaw   = atan2(2 * (this->x*this->y + this->w*this->z), sqw + sqx - sqy - sqz);

  };
  void setFromQuaternion(double quat_x,double quat_y,double quat_z,double quat_w)
  {
    this->x = quat_x;
    this->y = quat_y;
    this->z = quat_z;
    this->w = quat_w;
    this->normalize();
  };
  void setFromRPY(double roll, double pitch, double yaw)
  {
    double phi, the, psi;

    phi = roll / 2.0;
    the = pitch / 2.0;
    psi = yaw / 2.0;

    this->x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
    this->y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
    this->z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
    this->w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);

    this->normalize();
  };

  double x,y,z,w;
  Vector3 rpy; // Needs to be persistent for runtime expression parsing
  
  void init(const std::string &rotation_str, ParamTable_t &symbol_table)
  {
    this->clear();

    try {
      this->rpy.init(rotation_str,symbol_table);
    }
    catch (ParseError &e) {
      throw e.addMessage("malfomed rpy string ["+rotation_str+"]");
    }

    this->setFromRPY(this->rpy.x,this->rpy.y,this->rpy.z);  

  };
  
  void update(){
    this->rpy.update();
    this->setFromRPY(rpy.x,rpy.y,rpy.z);  
  };

  void clear() { this->x=this->y=this->z=0.0;this->w=1.0; }

  void normalize()
  {
    double s = sqrt(this->x * this->x +
                    this->y * this->y +
                    this->z * this->z +
                    this->w * this->w);
    if (s == 0.0)
    {
      this->x = 0.0;
      this->y = 0.0;
      this->z = 0.0;
      this->w = 1.0;
    }
    else
    {
      this->x /= s;
      this->y /= s;
      this->z /= s;
      this->w /= s;
    }
  };

  // Multiplication operator (copied from gazebo)
  Rotation operator*( const Rotation &qt ) const
  {
    Rotation c;

    c.x = this->w * qt.x + this->x * qt.w + this->y * qt.z - this->z * qt.y;
    c.y = this->w * qt.y - this->x * qt.z + this->y * qt.w + this->z * qt.x;
    c.z = this->w * qt.z + this->x * qt.y - this->y * qt.x + this->z * qt.w;
    c.w = this->w * qt.w - this->x * qt.x - this->y * qt.y - this->z * qt.z;

    return c;
  };
  /// Rotate a vector using the quaternion
  Vector3 operator*(Vector3 vec) const
  {
    Rotation tmp;
    Vector3 result;

    tmp.w = 0.0;
    tmp.x = vec.x;
    tmp.y = vec.y;
    tmp.z = vec.z;

    tmp = (*this) * (tmp * this->GetInverse());

    result.x = tmp.x;
    result.y = tmp.y;
    result.z = tmp.z;

    return result;
  };
  // Get the inverse of this quaternion
  Rotation GetInverse() const 
  {
    Rotation q;

    double norm = this->w*this->w+this->x*this->x+this->y*this->y+this->z*this->z;

    if (norm > 0.0)
    {
      q.w = this->w / norm;
      q.x = -this->x / norm;
      q.y = -this->y / norm;
      q.z = -this->z / norm;
    }

    return q;
  };


};

class Pose
{
public:
  Pose() { this->clear(); };

  Vector3  position;
  Rotation rotation;

  void clear()
  {
    this->position.clear();
    this->rotation.clear();
  };
  
  void update(){
    this->position.update();
    this->rotation.update();
  };
  
  void initXml(TiXmlElement* xml,ParamTable_t &symbol_table)
  {
    this->clear();
    if (xml)
    {
      const char* xyz_str = xml->Attribute("xyz");
      if (xyz_str != NULL)
      {
        try {
          this->position.init(xyz_str, symbol_table);
        }
        catch (ParseError &e) {
          throw e.addMessage("malformed xyz string ["+std::string(xyz_str)+"]");
        }
      }

      const char* rpy_str = xml->Attribute("rpy");
      if (rpy_str != NULL)
      {
        try {
          this->rotation.init(rpy_str, symbol_table);
        }
        catch (ParseError &e) {
          this->rotation.clear();
          throw e.addMessage("malformed rpy ["+std::string(rpy_str)+"]");
        }
      }

    }
  };
};

}

#endif
