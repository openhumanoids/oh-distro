#include <memory>
#include <string>

#include "pfgrasp.hpp"
#include <opencv2/opencv.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>
#include <bot_lcmgl_client/lcmgl.h>
#include <lcmtypes/drc/pfgrasp_command_t.hpp>
#include <drc_utils/LcmWrapper.hpp>

int main(){
  lcm::LCM lcm_publish_ ;

  drc::pfgrasp_command_t msg_out;
  cv::namedWindow("Command");
  while(1) {
      std::cout << "s: START  q: STOP  g: RUN_ONE_ITER  r: RESTART" << std::endl;
      int k = cv::waitKey(0) & 0xff;
      msg_out.utime = bot_timestamp_now();
      switch (k){
      case 's':
        msg_out.command = drc::pfgrasp_command_t::START;
        std::cout << "pushed s: START" << std::endl;
        break;
      case 'q':
        msg_out.command = drc::pfgrasp_command_t::STOP;
        std::cout << "pushed q: STOP" << std::endl;
        break;
      case 'g':
        msg_out.command = drc::pfgrasp_command_t::RUN_ONE_ITER;
        std::cout << "pushed g: RUN_ONE_ITER" << std::endl;
        break;
      case 'r':
        msg_out.command = drc::pfgrasp_command_t::RESTART;
        std::cout << "pushed r: RESTART" << std::endl;
        break;
      }
      lcm_publish_.publish("PFGRASP_CMD", &msg_out);
  }
}
