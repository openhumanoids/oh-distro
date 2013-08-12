#include "sandia_hand/palm.h"
using namespace sandia_hand;

Palm::Palm(const uint8_t addr)
: SerialMessageProcessor(addr)
{
}

Palm::~Palm()
{
}

bool Palm::pollState()
{
  if (!sendTxBuffer(PKT_PALM_STATE, 0))
    return false;
  return listenFor(PKT_PALM_STATE, 0.5);
}


