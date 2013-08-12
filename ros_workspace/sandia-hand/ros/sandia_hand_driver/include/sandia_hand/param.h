#ifndef SANDIA_HAND_PARAM_H
#define SANDIA_HAND_PARAM_H

#include <stdint.h>
#include <string>

namespace sandia_hand
{

class Param
{
public:
  Param(const char *name, int   val);
  Param(const char *name, float val);
  enum Type { PARAM_INT, PARAM_FLOAT };
  inline Type getType() const { return type_; } 
  int   getIntVal() const;
  float getFloatVal() const;
  inline const std::string &getName() const { return name_; }
private:
  std::string name_;
  Type type_;
  int val_int_;
  float val_float_;
};

}

#endif

