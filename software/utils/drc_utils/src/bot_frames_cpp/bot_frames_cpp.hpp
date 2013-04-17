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
  int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d&mat);

  // Varient of above directly returnig the frame
  Eigen::Isometry3d get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime);
  

protected:
};
 
}
#endif
