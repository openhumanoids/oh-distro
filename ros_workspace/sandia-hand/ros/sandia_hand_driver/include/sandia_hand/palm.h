#ifndef SANDIA_HAND_PALM_H
#define SANDIA_HAND_PALM_H

#include "sandia_hand/serial_message_processor.h"

namespace sandia_hand
{

class Palm : public SerialMessageProcessor
{
public:
  Palm(const uint8_t addr = 10);
  virtual ~Palm();
  static const uint8_t PKT_PALM_STATE = 0x21;
  bool pollState();
};

}

#endif
