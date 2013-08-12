#ifndef SANDIA_HAND_DISTAL_PHALANGE_H
#define SANDIA_HAND_DISTAL_PHALANGE_H

#include "sandia_hand/serial_message_processor.h"

namespace sandia_hand
{

class DistalPhalange : public SerialMessageProcessor
{
public:
  DistalPhalange();
  virtual ~DistalPhalange();
private:
};

}

#endif

