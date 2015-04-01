#ifndef _maps_LcmTranslator_hpp_
#define _maps_LcmTranslator_hpp_

namespace drc {
  class map_request_t;
  class map_blob_t;
  class map_image_t;
}

#include "ViewBase.hpp"

namespace maps {

class DataBlob;
class DepthImageView;

class LcmTranslator {
public:

  // for view request
  static bool toLcm(const ViewBase::Spec& iSpec, drc::map_request_t& oMessage);
  static bool fromLcm(const drc::map_request_t& iMessage,
                      ViewBase::Spec& oSpec);

  // for data blobs
  static bool toLcm(const maps::DataBlob& iBlob, drc::map_blob_t& oMessage);
  static bool fromLcm(const drc::map_blob_t& iMessage, maps::DataBlob& oBlob);

  // for depth image
  static bool toLcm(const DepthImageView& iView, drc::map_image_t& oMessage,
                    const float iQuantMax=-1, const bool iCompress=true);
  static bool fromLcm(const drc::map_image_t& iMessage, DepthImageView& oView);
};

}

#endif
