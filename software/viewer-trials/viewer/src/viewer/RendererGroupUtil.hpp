#ifndef _RendererGroupUtil_hpp_
#define _RendererGroupUtil_hpp_

#include <boost/shared_ptr.hpp>

typedef struct _BotViewer BotViewer;
typedef struct _BotParam BotParam;

class RendererGroupUtil {
public:
  RendererGroupUtil(BotViewer* iViewer, BotParam* iParam);
  void setup();

private:
  struct ViewerBundle;
  boost::shared_ptr<ViewerBundle> mBundle;
};

#endif
