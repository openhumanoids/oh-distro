#ifndef _mexmaps_MapLib_hpp_
#define _mexmaps_MapLib_hpp_

#include <memory>
#include <string>

namespace maps {
  class ViewBase;
}

namespace mexmaps {

struct ViewClientWrapper;

class MapHandle {

public:
  static const std::string kHeightMapChannel;
  static const int kHeightMapViewId;

public:
  MapHandle(ViewClientWrapper* iWrapper);
  std::shared_ptr<maps::ViewBase> getView() const;

protected:
  ViewClientWrapper* mWrapper;
};

}

#endif
