#include <iostream>
#include <unordered_map>

#include <drc_utils/LcmWrapper.hpp>
#include <ConciseArgs>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/drc/map_snapshot_request_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>

#include <maps/BotWrapper.hpp>
#include <maps/ViewBase.hpp>
#include <maps/ViewClient.hpp>
#include <maps/LcmTranslator.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/OctreeView.hpp>
#include <maps/DepthImageView.hpp>

struct ViewData {
  int64_t mViewId;
  int64_t mViewIdOrig;
  std::string mPublishChannel;
  std::shared_ptr<maps::ViewBase> mView;
};

struct State {
  std::shared_ptr<drc::LcmWrapper> mLcmWrapper;
  std::shared_ptr<lcm::LCM> mLcm;
  std::shared_ptr<maps::BotWrapper> mBotWrapper;
  std::shared_ptr<maps::ViewClient> mViewClient;
  std::unordered_map<int64_t,ViewData> mViewDataSet;

  State() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcm = mLcmWrapper->get();
    mBotWrapper.reset(new maps::BotWrapper(mLcm));
    mViewClient.reset(new maps::ViewClient());
    mViewClient->setBotWrapper(mBotWrapper);
    // mViewClient.addListener(this);

    mLcm->subscribe("MAP_SNAPSHOT_REQUEST", &State::onSnapshotRequest, this);
  }

  void start() {
    mViewClient->start();
    mLcmWrapper->startHandleThread(true);
  }

  void storeView(const drc::map_snapshot_request_t* iMessage) {
    auto view = mViewClient->getView(iMessage->view_id);
    if (view != NULL) {
      ViewData viewData;
      viewData.mViewIdOrig = iMessage->view_id;
      viewData.mViewId = iMessage->new_view_id;
      switch(view->getType()) {
      case maps::ViewBase::TypePointCloud:
        viewData.mPublishChannel = "MAP_CLOUD"; break;
      case maps::ViewBase::TypeOctree:
        viewData.mPublishChannel = "MAP_OCTREE"; break;
      case maps::ViewBase::TypeDepthImage:
        viewData.mPublishChannel = "MAP_DEPTH"; break;
      }
      viewData.mView = view->clone();
      viewData.mView->setId(viewData.mViewId);
      mViewDataSet[viewData.mViewId] = viewData;
      std::cout << "stored new view " << viewData.mViewIdOrig << " -> " <<
        viewData.mViewId << std::endl;
    }
    else {
      std::cout << "error: view " << iMessage->view_id <<
        " does not exist to store" << std::endl;
    }
  }

  void deleteView(const drc::map_snapshot_request_t* iMessage) {
    auto item = mViewDataSet.find(iMessage->view_id);
    if (item != mViewDataSet.end()) {
      mViewDataSet.erase(iMessage->view_id);
      std::cout << "deleted snapshot " << iMessage->view_id << std::endl;
    }
    else {
      std::cout << "error: view " << iMessage->view_id <<
        " does not exist to delete" << std::endl;
    }
  }

  void publishView(const drc::map_snapshot_request_t* iMessage) {
    auto item = mViewDataSet.find(iMessage->view_id);
    if (item != mViewDataSet.end()) {
      auto view = item->second.mView;
      maps::ViewBase::Type type = view->getType();
      if (type == maps::ViewBase::TypePointCloud) {
        drc::map_cloud_t msg;
        maps::LcmTranslator::toLcm
          (*std::dynamic_pointer_cast<maps::PointCloudView>(view), msg);
        mLcm->publish(item->second.mPublishChannel, &msg);
      }
      else if (type == maps::ViewBase::TypeOctree) {
        drc::map_octree_t msg;
        maps::LcmTranslator::toLcm
          (*std::dynamic_pointer_cast<maps::OctreeView>(view), msg);
        mLcm->publish(item->second.mPublishChannel, &msg);
      }
      else if (type == maps::ViewBase::TypeDepthImage) { 
        drc::map_image_t msg;
        maps::LcmTranslator::toLcm
          (*std::dynamic_pointer_cast<maps::DepthImageView>(view), msg);
        mLcm->publish(item->second.mPublishChannel, &msg);
      }
      else {
        std::cout << "error: invalid view type" << std::endl;
      }
      std::cout << "sent snapshot " << iMessage->view_id << std::endl;
    }
    else {
      std::cout << "error: view " << iMessage->view_id <<
        " does not exist to publish" << std::endl;
    }      
  }

  void updateTransform(const drc::map_snapshot_request_t* iMessage) {
    auto item = mViewDataSet.find(iMessage->view_id);
    if (item != mViewDataSet.end()) {
      Eigen::Projective3f xform;
      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          xform(i,j) = iMessage->transform[i][j];
        }
      }
      item->second.mView->setTransform(xform);
    }
    else {
      std::cout << "error: view " << iMessage->view_id <<
        " does not exist to update" << std::endl;
    }      
  }

  void onSnapshotRequest(const lcm::ReceiveBuffer* iBuffer,
                         const std::string& iChannel,
                         const drc::map_snapshot_request_t* iMessage) {
    switch (iMessage->command) {
    case drc::map_snapshot_request_t::STORE: storeView(iMessage); break;
    case drc::map_snapshot_request_t::DELETE: deleteView(iMessage); break;
    case drc::map_snapshot_request_t::PUBLISH: publishView(iMessage); break;
    case drc::map_snapshot_request_t::UPDATE_TRANSFORM: updateTransform(iMessage); break;
    default: std::cout << "error: unknown snapshot command" << std::endl; break;
    }
  }
};

int main(const int iArgc, const char** iArgv) {

  // instantiate state object
  State state;

  // parse arguments
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.parse();

  state.start();

  return 0;
}
