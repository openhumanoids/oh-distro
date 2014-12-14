#include <stdio.h>
#include <iostream>
#include "math_parser/exprtk/exprtk.hpp"
#include <string>
//#include <cctype>
#include <algorithm>
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
};

std::string removeWhiteSpaceInExpressions (std::string expression_string)
{
  size_t found,found2,found3;
  std::string start_key = "${"; 
  std::string end_key = "}";
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
};

int main(int argc, char** argv)
{

   std::string expression_string = "clamp(-1.0,sin(2 * pi * x) + cos(x / 2 * pi),+1.0)";
   
//    expression_string ="${X + Y} 0  ${ X + Y }";
//        std::cout <<  expression_string  <<std::endl;
//    expression_string=removeWhiteSpaceInExpressions(expression_string);
//       std::cout <<  expression_string  << std::endl << std::endl;
   expression_string = "2*(${x}*${x})";
   size_t found;
   
   expression_string=unXacroString(expression_string);
   std::cout <<  expression_string  <<std::endl;
   
//    std::map <std::string, double> params_map_;

   double x;
   exprtk::symbol_table<double> symbol_table;
//    symbol_table.add_variable("x",params_map_["x"]);
   symbol_table.add_variable("x",x);
   symbol_table.add_constants();

   exprtk::expression<double> expression;
   expression.register_symbol_table(symbol_table);

   exprtk::parser<double> parser;
   parser.compile(expression_string,expression);
  if (!parser.compile(expression_string,expression))
   {
      std::cout << "Error: " << parser.error() << "\tExpression: " << expression_string << std::endl;
      return -1;
   }
   
//    params_map_["x"] = 3.0;
   
   x = 3.0;	

   double y = 1.0,z;
   y=expression.value();
   std::cout << expression_string << ": " << x << " " << y <<std::endl;
  
//    params_map_["x"] = 5.0;
   x = 5.0;
   std::cout << expression_string << ": " << x << " " << expression.value() <<std::endl;
  
  
   expression_string = "12-(x)";
  if (!parser.compile(expression_string,expression))
   {
      std::cout << "Error: " << parser.error() << "\tExpression: " << expression_string << std::endl;
      return -1;
   }
   std::cout << expression_string << ": " << x << " " << expression.value() <<std::endl;
   return 0;
}
