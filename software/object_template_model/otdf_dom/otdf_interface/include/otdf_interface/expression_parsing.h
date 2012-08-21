#ifndef OTDF_EXPRESSION_PARSING_H
#define OTDF_EXPRESSION_PARSING_H

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <tinyxml.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "otdf_interface/exceptions.h"
#include "math_parser/exprtk/exprtk.hpp"

namespace otdf
{
typedef exprtk::symbol_table<double> ParamTable_t;

std::string unXacroString (std::string expression_string);
std::string removeWhiteSpaceInExpressions (const std::string &string);
bool isExpression(std::string str);

//Checks if an xml tag is an expression. If it is not, it performs a lexical cast and returns the value. 
// Or else it compiles the expression and returns the value. 
// The third argument is a reference to a local expression variable which is made persistent in memory so that any expressed variables 
// can be updated without having to recompile the expression when params are changed.
// The fourth argument indicates if a expression was compiled or not.
double lexicalCastOrExpressionParsing(std::string attribute_str, TiXmlElement *_xml, ParamTable_t &symbol_table, exprtk::expression<double> &expression, std::vector<bool>::reference flag);

//Associates a symbol table with the expression, compiles and return the expression. expression.value() returns it value.
bool compileExpression(std::string expression_string, ParamTable_t &symbol_table, exprtk::expression<double> &expression);
}

#endif

