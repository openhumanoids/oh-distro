#ifndef _maps_LcmTranslator_hpp_
#define _maps_LcmTranslator_hpp_

namespace drc {
  class map_request_t;
  class map_params_t;
  class map_cloud_t;
  class map_octree_t;
  class map_image_t;
}

#include "LocalMap.hpp"
#include "MapView.hpp"

namespace maps {

class LcmTranslator {
public:

  // for map specification
  static drc::map_params_t toLcm(const LocalMap::Spec& iSpec);
  static LocalMap::Spec fromLcm(const drc::map_params_t& iMessage);

  // for view request
  static drc::map_request_t toLcm(const MapView::Spec& iSpec);
  static MapView::Spec fromLcm(const drc::map_request_t& iMessage);

  // for point cloud
  static drc::map_cloud_t toLcm(const maps::PointCloud& iCloud);
  static maps::PointCloud::Ptr fromLcm(const drc::map_cloud_t& iMessage);

  // for octree
  static drc::map_octree_t toLcm(const maps::Octree& iTree);
  static maps::Octree fromLcm(const drc::map_octree_t& iMessage);

  // for range image
  static drc::map_image_t toLcm(const maps::RangeImage& iImage);
  static maps::RangeImage fromLcm(const drc::map_image_t& iMessage);
};

}

#endif
