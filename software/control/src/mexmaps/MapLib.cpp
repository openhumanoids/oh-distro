#include "MapLib.hpp"
#include "ViewClientWrapper.hpp"

using namespace mexmaps;

const std::string kHeightMapChannel = "MAP_CONTROL_HEIGHT";
const int kHeightMapViewId = 1000;

MapHandle::
MapHandle(ViewClientWrapper* iWrapper) : mWrapper(iWrapper) {}

std::shared_ptr<maps::ViewBase> MapHandle::
getView() const {
  return mWrapper->getView();
}
