#include <cstdio>
#include "sandia_hand/proximal_phalange.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace sandia_hand;

ProximalPhalange::ProximalPhalange()
: SerialMessageProcessor(1)
{
  //print_parser_debris_ = true;
}

ProximalPhalange::~ProximalPhalange()
{
}

