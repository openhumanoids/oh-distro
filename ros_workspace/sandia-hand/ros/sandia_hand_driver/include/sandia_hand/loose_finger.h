#ifndef SANDIA_HAND_LOOSE_FINGER_H
#define SANDIA_HAND_LOOSE_FINGER_H

#include <sandia_hand/finger.h>
class LightweightSerial;

namespace sandia_hand
{

class LooseFinger : public Finger
{
public:
  LooseFinger();
  virtual ~LooseFinger();
  bool init(const char *serial_device);
  bool listen(const float max_seconds);
  bool tx(const uint8_t *pkt, const uint16_t pkt_len);
private:
  LightweightSerial *serial_; // I know, this should be boost::asio... someday
                             // I will switch it.
};

}

#endif
