#ifndef _maps_LcmTranslator_hpp_
#define _maps_LcmTranslator_hpp_

namespace drc {
  class map_request_t;
  class map_params_t;
  class map_blob_t;
  class map_cloud_t;
  class map_octree_t;
  class map_image_t;
  class map_scans_t;
  class map_scan_t;
}

#include "LocalMap.hpp"
#include "ViewBase.hpp"

namespace maps {

class DataBlob;
class PointCloudView;
class OctreeView;
class DepthImageView;
class ScanBundleView;
class LidarScan;

class LcmTranslator {
public:

  // for map specification
  static bool toLcm(const LocalMap::Spec& iSpec, drc::map_params_t& oMessage);
  static bool fromLcm(const drc::map_params_t& iMessage, LocalMap::Spec& oSpec);

  // for view request
  static bool toLcm(const ViewBase::Spec& iSpec, drc::map_request_t& oMessage);
  static bool fromLcm(const drc::map_request_t& iMessage,
                      ViewBase::Spec& oSpec);

  // for data blobs
  static bool toLcm(const maps::DataBlob& iBlob, drc::map_blob_t& oMessage);
  static bool fromLcm(const drc::map_blob_t& iMessage, maps::DataBlob& oBlob);

  // for point cloud
  static bool toLcm(const PointCloudView& iView, drc::map_cloud_t& oMessage,
                    const float iQuantMax=-1, const bool iCompress=true);
  static bool fromLcm(const drc::map_cloud_t& iMessage, PointCloudView& oView);

  // for octree
  static bool toLcm(const OctreeView& iView, drc::map_octree_t& oMessage);
  static bool fromLcm(const drc::map_octree_t& iMessage, OctreeView& oView);

  // for depth image
  static bool toLcm(const DepthImageView& iView, drc::map_image_t& oMessage,
                    const float iQuantMax=-1, const bool iCompress=true);
  static bool fromLcm(const drc::map_image_t& iMessage, DepthImageView& oView);

  // for scan
  static bool toLcm(const LidarScan& iScan, drc::map_scan_t& oMessage,
                    const float iQuantMax=-1, const bool iCompress=true,
                    const bool iIncludeIntensities=false);
  static bool fromLcm(const drc::map_scan_t& iMessage, LidarScan& oScan);

  // for scan bundle
  static bool toLcm(const ScanBundleView& iView, drc::map_scans_t& oMessage,
                    const float iQuantMax=-1, const bool iCompress=true,
                    const bool iIncludeIntensities=false);
  static bool fromLcm(const drc::map_scans_t& iMessage, ScanBundleView& oView);
};

}

#endif
