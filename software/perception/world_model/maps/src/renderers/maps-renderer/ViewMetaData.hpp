#ifndef _maps_ViewMetaData_hpp_
#define _maps_ViewMetaData_hpp_

#include <memory>
#include <inttypes.h>
#include <Eigen/Geometry>

namespace Gtk {
  class Box;
}

namespace maps {

struct MapsRenderer;
class ViewBase;
class MeshRenderer;

class ViewMetaData {
public:
  typedef std::shared_ptr<ViewMetaData> Ptr;

public:
  ViewMetaData(const MapsRenderer* iRenderer, const int64_t iViewId);
  bool addWidgets(Gtk::Box* iBox);
  void draw(const std::shared_ptr<maps::ViewBase>& iView,
            const std::shared_ptr<MeshRenderer>& iMeshRenderer);

  void setLatestTransform(const Eigen::Isometry3f& iTransform);

private:
  struct Helper;
  std::shared_ptr<Helper> mHelper;
};

}

#endif
