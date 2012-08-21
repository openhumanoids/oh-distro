#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "otdf_interface/exceptions.h"
#include "otdf_interface/expression_parsing.h"

namespace otdf
{
//typedef exprtk::symbol_table<double> ParamTable_t;

// removes ${ and } in string.
std::string unXacroString (std::string expression_string)
{
  size_t found;

  std::string key = "${";
  found=expression_string.find("${");
  while (found!=std::string::npos)
  {
    expression_string.erase(found,key.length());
    found=expression_string.find("${",found+1);
  }

  found=expression_string.find_first_of("}");
  while (found!=std::string::npos)
  {
    expression_string.erase(found,1);
    found=expression_string.find_first_of("}",found+1);
  }

  return expression_string;
}

std::string removeWhiteSpaceInExpressions (const std::string &string)
{
  size_t found,found2,found3;
  std::string start_key = "${"; 
  std::string end_key = "}";
  std::string expression_string = string;
  found=expression_string.find(start_key);
  found2=expression_string.find_first_of("}",found);
 
  while ((found!=found2)&&(found2!=std::string::npos))
  {
    std::string substr = expression_string.substr(found,found2-found+1);

     // substr.erase (std::remove (substr.begin(), substr.end(), ' '), substr.end());
      //for some reason gcc does not recognise std::isspace, ::isspace works. wierd!      
    substr.erase (std::remove_if(substr.begin(), substr.end(), ::isspace), substr.end());
    expression_string.erase(found,found2-found+1);
    expression_string.insert(found,substr);

    int offset = found2+1-found - substr.length();    
    found=expression_string.find(start_key,found+1);
    found2=expression_string.find_first_of("}",found2-offset+1);

  }

  return expression_string;
}


bool isExpression(std::string str)
{
    size_t found = str.find("${");
    return (found != std::string::npos);
}


bool compileExpression(std::string expression_string, ParamTable_t &symbol_table, exprtk::expression<double> &expression)
{
  // removes ${ and } in string. 
  expression_string=unXacroString(expression_string);
  expression.register_symbol_table(symbol_table); 
   exprtk::parser<double> parser;
   //parser.compile(expression_string,expression); // expression string only needs to be compiled at init.
  if (!parser.compile(expression_string,expression))
   {
      std::cout << "Error: " << parser.error() << "\tExpression: " << expression_string << std::endl;
      return false;
   }
   return true;
}

double lexicalCastOrExpressionParsing(std::string attribute_str, TiXmlElement *_xml, ParamTable_t &symbol_table, exprtk::expression<double> &expression, std::vector<bool>::reference flag)
{
  double value;
     flag = false;
   std::stringstream stm;
   stm << _xml->Attribute(attribute_str.c_str());
  std::string str = stm.str();

  if(!isExpression(str))
  {

    flag = false;
    try
    {
      value  = boost::lexical_cast<double>(_xml->Attribute(attribute_str.c_str()));
    }
    catch (boost::bad_lexical_cast &e)
    {
      std::stringstream stm;
      stm << attribute_str <<"[" << str << "]" << " is not a valid double ";
      throw ParseError(stm.str());
    }
  }
  else {
    flag = true;
    //value = parseExpression(str, symbol_table);
    if(!compileExpression(str, symbol_table, expression)){
	value = 0; //compilation failed. 
    }
    else {
	value = expression.value();
    }
  }
  return value;
}

}