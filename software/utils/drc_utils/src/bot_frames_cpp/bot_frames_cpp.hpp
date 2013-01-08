#ifndef _bot_frames_hpp_
#define _bot_frames_hpp_

#include <inttypes.h>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <boost/shared_ptr.hpp>
#include <bot_frames/bot_frames.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

namespace bot {

class frames {
public:
  frames();
  //static frames* instance();

  int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d&mat);

/*  virtual void setLcm(const boost::shared_ptr<lcm::LCM>& iLcm);

  virtual void setChannel(const std::string& iChannelName);
  std::string getChannel() const;

  void setTimeoutInterval(const int iMilliseconds);

  virtual int64_t getCurrentTime() const = 0;
*/

protected:
};
 
}
#endif
