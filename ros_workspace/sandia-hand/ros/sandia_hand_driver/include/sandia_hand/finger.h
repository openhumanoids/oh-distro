#ifndef SANDIA_HAND_FINGER_H
#define SANDIA_HAND_FINGER_H

#include "sandia_hand/motor_module.h"
#include "sandia_hand/proximal_phalange.h"
#include "sandia_hand/distal_phalange.h"

namespace sandia_hand
{

class Finger
{
public:
  Finger();
  virtual ~Finger();

  MotorModule mm;
  ProximalPhalange pp;
  DistalPhalange dp;

  bool programDistalPhalangeAppFile(FILE *bin_file);
  bool programProximalPhalangeAppFile(FILE *bin_file);
  typedef boost::function<void(const float)> ListenFunctor;
  void registerListenHandler(ListenFunctor functor);
protected:
  void listen(const float max_seconds);
  ListenFunctor listen_functor_;
};

}

#endif
