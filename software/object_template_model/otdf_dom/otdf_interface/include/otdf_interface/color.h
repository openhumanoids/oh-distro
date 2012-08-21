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

/* Author: Josh Faust */

// Modified from ROS's URDF_DOM library Written by Josh Faust and Wim Meeussen

#ifndef OTDF_INTERFACE_COLOR_H
#define OTDF_INTERFACE_COLOR_H

#include <string>
#include <vector>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "expression_parsing.h"

namespace otdf
{

class Color
{
public:
  Color(){this->clear();};
  float r;
  float g;
  float b;
  float a;
  
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
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    expression_flags.push_back(false);
    r = g = b = 0.0f;
    a = 1.0f;
  }
  bool init(const std::string &vector_str, ParamTable_t &symbol_table)
  {
    this->clear();
    std::vector<std::string> pieces;
    std::vector<float> rgba;
    std::string vector_of_expressions;
    vector_of_expressions=removeWhiteSpaceInExpressions(vector_str);
    boost::split( pieces, vector_of_expressions, boost::is_any_of(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (!pieces[i].empty())
      {
	if(!isExpression(pieces[i]))
	{
	  try
	  {
	    rgba.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
	  }
	  catch (boost::bad_lexical_cast &e)
	  {
	    throw("color rgba element ("+pieces[i]+") is not a valid float");
	  }
	}
	else
	{
	  expression_flags[i] = true;
	    exprtk::expression<double> expression;  
	  if(!compileExpression(pieces[i], symbol_table, expression)){
	    return false;
	  }
	  else {
	    this->local_expressions[i] = expression;
	    double value = local_expressions[i].value();
	    rgba.push_back(value);
	  }
// 	  double value = parseExpression(pieces[i], symbol_table);
// 	  rgba.push_back(value);

	}
      }
    }

    if (rgba.size() != 4)
    {
      //ROS_ERROR("Color contains %i elements instead of 4 elements", (int)rgba.size());
      return false;
    }

    this->r = rgba[0];
    this->g = rgba[1];
    this->b = rgba[2];
    this->a = rgba[3];

    return true;
  };

  void update()
  {
      std::vector<float> rgba;
     rgba[0] =   this->r;
     rgba[1] =   this->g;
     rgba[2] =   this->b;
     rgba[3] =   this->a;
      for (unsigned int i = 0; i < rgba.size(); ++i)
      {
	if(expression_flags[i])
	{
	  rgba[i] = local_expressions[i].value();
	}
      }
      
    this->r = rgba[0];
    this->g = rgba[1];
    this->b = rgba[2];
    this->a = rgba[3];
    
//     std::cout << "r" << this->r 
// 	      << "g" << this->g 	    
// 	      << "b" << this->b 
// 	      << "a"  << this->a << std::endl;
  };
  
  
};

}

#endif

