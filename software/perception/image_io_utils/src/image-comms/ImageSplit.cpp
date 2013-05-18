#include <iostream>
#include <memory>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/LcmWrapper.hpp>

#include <lcmtypes/bot_core/image_t.hpp>

struct ImageSplit {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;

  ImageSplit() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
  }

  void addChannel(const std::string& iChannel) {
    mLcm->subscribe(iChannel, &ImageSplit::onImage, this);
  }

  void onImage(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel,
               const bot_core::image_t* iMessage) {
    bot_core::image_t msg = *iMessage;
    msg.height /= 2;
    std::vector<uint8_t> buf(&msg.data[0],
                             &msg.data[0] + msg.height*msg.row_stride);
    msg.size = buf.size();
    msg.data = buf;
    msg.nmetadata = 0;
    mLcm->publish(iChannel + "LEFT", &msg);
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }
};

int main() {
  ImageSplit obj;
  obj.addChannel("CAMERA_LHAND");
  obj.addChannel("CAMERA_RHAND");
  obj.start();
}
