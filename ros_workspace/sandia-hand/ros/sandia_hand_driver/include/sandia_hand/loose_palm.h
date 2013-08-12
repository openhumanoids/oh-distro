#ifndef SANDIA_HAND_LOOSE_PALM_H
#define SANDIA_HAND_LOOSE_PALM_H

#include <sandia_hand/palm.h>
class LightweightSerial;

namespace sandia_hand
{

class LoosePalm : public Palm
{
public:
  LoosePalm();
  virtual ~LoosePalm();
  bool init(const char *serial_device);
  bool listen(const float max_seconds);
  bool tx(const uint8_t *pkt, const uint16_t pkt_len);
private:
  LightweightSerial *serial_; // I know, this should be boost::asio... someday
                              // I will switch it.
};

}

#endif
