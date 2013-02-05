#include <boost/bind.hpp>
#include <multisense_driver/multisense_driver.h>
#include <LibSensorPodCommunications/CamImageDataMessage.h>
#include <LibSensorPodCommunications/CamGetConfigMessage.h>
#include <LibSensorPodCommunications/CamStopImageStreamMessage.h>
#include <cstdio>

void imageCB(const boost::shared_ptr<const CamImageDataMessage>& msg)
{
  printf("Image callback called\n");
  return;
}

int main(int argc, char* argv[])
{
  printf("%u\n", CamStopImageStreamMessage::MSG_ID);
  multisense_driver::MultisenseDriver driver("10.10.72.52");
  multisense_driver::MultisenseSubscriber<CamImageDataMessage> sub;
  boost::function<void (const boost::shared_ptr<const CamImageDataMessage>&) > cb = boost::bind(imageCB, _1);
  sub = driver.subscribe<CamImageDataMessage>(cb);
  CamGetConfigMessage cam_get_config_msg;
  driver.publish(cam_get_config_msg);

  while(true)
  {
    sleep(1.0);
    driver.publish(cam_get_config_msg);
    printf("Sending CamGetConfigMsg to device\n");
  }
}

