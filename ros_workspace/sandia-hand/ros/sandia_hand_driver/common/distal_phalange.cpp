#include <cstdio>
#include "sandia_hand/distal_phalange.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace sandia_hand;

DistalPhalange::DistalPhalange()
: SerialMessageProcessor(2)
{
  //print_parser_debris_ = true;
}

DistalPhalange::~DistalPhalange()
{
}

