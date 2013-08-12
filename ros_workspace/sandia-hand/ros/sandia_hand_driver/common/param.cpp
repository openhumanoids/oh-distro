#include "sandia_hand/param.h"
using namespace sandia_hand;

Param::Param(const char *name, int val)
: name_(name), type_(PARAM_INT), val_int_(val), val_float_(0)
{
}

Param::Param(const char *name, float val)
: name_(name), type_(PARAM_FLOAT), val_int_(0), val_float_(val)
{
}

int Param::getIntVal() const
{
  if (type_ == PARAM_INT)
    return val_int_;
  else
    return (int)val_float_;
}

float Param::getFloatVal() const
{
  if (type_ == PARAM_FLOAT)
    return val_float_;
  else
    return (float)val_int_;
}

