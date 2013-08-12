#include <cstdio>
#include "sandia_hand/loose_finger.h"
#include "sandia_hand/lightweightserial.h"
#include <ros/time.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace sandia_hand;

LooseFinger::LooseFinger() :
  serial_(NULL)
{
  mm.setRawTx(boost::bind(&LooseFinger::tx, this, _1, _2));
  registerListenHandler(boost::bind(&LooseFinger::listen, this, _1));
}

LooseFinger::~LooseFinger()
{
  if (serial_)
  {
    delete serial_;
    serial_ = NULL;
  }
}

bool LooseFinger::init(const char *serial_device)
{
  if (serial_)
  {
    printf("WOAH THERE PARTNER. you already initialized this loose finger.");
    return false;
  }
  serial_ = new LightweightSerial(serial_device, 2000000);
  if (!serial_->is_ok())
  {
    printf("couldn't open serial port\n");
    delete serial_;
    serial_ = NULL;
    return false;
  }
  return true;
}

bool LooseFinger::tx(const uint8_t *pkt, const uint16_t pkt_len)
{
  /*
  printf("tx %d bytes:\n  ", pkt_len);
  for (int i = 0; i < pkt_len; i++)
    printf("0x%02x ", pkt[i]);
  printf("\n");
  */
  if (!serial_)
  {
    printf("WOAH THERE PARTNER. serial device not initialized\n");
    return false;
  }
  return serial_->write_block(pkt, pkt_len);
}

bool LooseFinger::listen(const float max_seconds)
{
  if (!serial_)
  {
    printf("WOAH THERE PARTNER. serial device not initialized\n");
    return false;
  }
  for(ros::Time t_start(ros::Time::now());
      (ros::Time::now() - t_start).toSec() < max_seconds;)
  {
    uint8_t buf[1024];
    int nread = serial_->read_block(buf, sizeof(buf)-1);
    if (nread < 0)
    {
      printf("error reading serial device\n");
      return false;
    }
    else if (nread == 0)
      ros::Duration(0.0001).sleep();
    else
    {
      /*
      printf("rx %d bytes:\n  ", nread);
      for (int i = 0; i < nread; i++)
        printf("0x%02x ", buf[i]);
      printf("\n");
      */
      mm.rx(buf, nread);
    }
  }
  return true; // no i/o error
}

