#include <cstdio>
#include "sandia_hand/finger.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace sandia_hand;

Finger::Finger()
: mm(10) // default finger address in firmware (historical...)
{
  mm.addPhalangeRxFunctor(boost::bind(&ProximalPhalange::rx, &pp, _1, _2));
  mm.addPhalangeRxFunctor(boost::bind(&DistalPhalange::rx, &dp, _1, _2));
  pp.setRawTx(boost::bind(&MotorModule::phalangeTxRx, &mm, _1, _2));
  dp.setRawTx(boost::bind(&MotorModule::phalangeTxRx, &mm, _1, _2));
  mm.registerListenHandler(boost::bind(&Finger::listen, this, _1));
  pp.registerListenHandler(boost::bind(&Finger::listen, this, _1));
  dp.registerListenHandler(boost::bind(&Finger::listen, this, _1));
}

Finger::~Finger()
{
}

bool Finger::programDistalPhalangeAppFile(FILE *bin_file)
{
  return dp.programAppFile(bin_file,
              boost::bind(&MotorModule::setPhalangeBusPower, &mm, false),
              boost::bind(&MotorModule::setPhalangeBusPower, &mm, true));
}

bool Finger::programProximalPhalangeAppFile(FILE *bin_file)
{
  return pp.programAppFile(bin_file,
              boost::bind(&MotorModule::setPhalangeBusPower, &mm, false),
              boost::bind(&MotorModule::setPhalangeBusPower, &mm, true));
}

void Finger::listen(const float max_seconds)
{
  if (!listen_functor_)
  {
    printf("WOAH THERE. called Finger::listen with no listen_functor_");
    return; // really should assert...
  }
  listen_functor_(max_seconds);
}

void Finger::registerListenHandler(ListenFunctor f)
{
  listen_functor_ = f;
}

