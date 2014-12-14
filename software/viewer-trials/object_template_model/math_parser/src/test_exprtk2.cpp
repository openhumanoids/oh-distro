

#include <stdio.h>
#include <iostream>
#include "math_parser/exprtk/exprtk.hpp"

#include <string>
#include <vector>

typedef exprtk::symbol_table<double> ParamTable_t;


class B
{
public:
   B() {
     exprtk::expression<double> expression; 
     exprtk::parser<double> parser;
     parser.compile("1",expression);
     this->local_expressions.push_back(expression);
  };
  ~B() {};
   
  bool init (std::string expression_string, ParamTable_t &symbol_table) {
    this->local_expressions[0].register_symbol_table(symbol_table);
    exprtk::parser<double> parser;
    if (!parser.compile(expression_string,this->local_expressions[0]))
   {
      std::cout << "Error: " << parser.error() << "\tExpression: " << expression_string << std::endl;
      return false;
   }
   //this->local_expressions[0] =this->local_expression;
   return true;
  };
  double getValue (){
   // return local_expression.value();   
    return this->local_expressions[0].value();
  };
   std::vector<exprtk::expression<double> >  local_expressions;
   
  private:
   exprtk::expression<double> local_expression;

   double value;
};


class A
{
 
public:
   A() {};
  ~A() {};
  void init(){
  std::string expression_string = "2*(x*y)";
    symbol_table.add_variable("x",x);
    symbol_table.add_variable("y",y);
//     symbol_table.add_variable("x",this->params_map_["x"]);
//     symbol_table.add_variable("y",this->params_map_["y"]);
    symbol_table.add_constants();
    if(!this->b.init(expression_string,symbol_table)){
	    std::cout << "Error: expression compilation failed" <<std::endl; 
    } 
  };
  
  void print(){
     x = 1;
     y = 1;
//      this->params_map_["x"] = 1;
//      this->params_map_["y"] = 1;
     std::cout << b.getValue() <<std::endl;
  };
  
  void print2(){
     x = 2;
     y = 3;
//      this->params_map_["x"] = 2;
//      this->params_map_["y"] = 3;
     std::cout << b.getValue() <<std::endl;
  };
private:
  B b;
  ParamTable_t symbol_table;
  double x;
  double y;
//   std::map <std::string, double> params_map_;
  
};

int main(int argc, char** argv)
{
   A a;
   a.init();
   a.print();
   a.print2();
   return 0;
}
