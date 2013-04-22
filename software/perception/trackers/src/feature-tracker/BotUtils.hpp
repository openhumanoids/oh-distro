#ifndef _tracking_BotUtils_hpp_
#define _tracking_BotUtils_hpp_

typedef struct _BotParam BotParam;

namespace tracking {

class StereoCamera;

class BotUtils {
public:
  static bool configure(const BotParam* iParam, StereoCamera& oCamera);
};

}

#endif
