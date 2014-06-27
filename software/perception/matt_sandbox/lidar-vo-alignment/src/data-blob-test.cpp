#include <iostream>
#include <thread>

#include <maps/DataBlob.hpp>
#include <maps/LcmTranslator.hpp>
#include <maps/PointCloudView.hpp>
#include <maps/ViewClient.hpp>

#include <drc_utils/LcmWrapper.hpp>
#include <maps/BotWrapper.hpp>
#include <lcmtypes/drc/map_blob_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcm/lcm-cpp.hpp>

struct State {
  drc::LcmWrapper::Ptr mLcmWrapper;
  maps::BotWrapper::Ptr mBotWrapper;
  std::thread mPublishThread;
  std::shared_ptr<maps::ViewClient> mViewClient;

  void onBlob(const lcm::ReceiveBuffer* iBuf,
              const std::string& iChannel,
              const drc::map_blob_t* iMessage) {
    maps::DataBlob blob;
    maps::LcmTranslator::fromLcm(*iMessage, blob);
    std::cout << "BEFORE " << blob.getBytes().size() << std::endl;
    blob.convertTo(maps::DataBlob::CompressionTypeNone,
                   maps::DataBlob::DataTypeFloat32);
    std::cout << "AFTER " << blob.getBytes().size() << std::endl;

    std::cout << "RECEIVED" << std::endl;
  }

  void onCloud(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const drc::map_cloud_t* iMessage) {
    maps::PointCloudView view;
    maps::LcmTranslator::fromLcm(*iMessage, view);
    std::cout << "GOT CLOUD" << std::endl;
  }

  void operator()() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    /*
    maps::DataBlob blob;
    maps::DataBlob::Spec spec;
    const int size = 1000000;
    spec.mDimensions = { size };
    spec.mStrideBytes = { 4 };
    spec.mCompressionType = maps::DataBlob::CompressionTypeNone;
    spec.mDataType = maps::DataBlob::DataTypeFloat32;
    std::vector<float> data(size);
    for (int i = 0; i < size; ++i) {
      data[i] = i;
    }
    blob.setData((uint8_t*)data.data(), data.size()*sizeof(float), spec);
    std::cout << "BEFORE " << blob.getBytes().size() << std::endl;
    blob.convertTo(maps::DataBlob::CompressionTypeZlib,
                   maps::DataBlob::DataTypeFloat32);
    std::cout << "AFTER " << blob.getBytes().size() << std::endl;

    drc::map_blob_t msg;
    maps::LcmTranslator::toLcm(blob, msg);
    mLcmWrapper->get()->publish("TEST_BLOB", &msg);
    std::cout << "SENT" << std::endl;
    */

    maps::PointCloudView view;
    maps::PointCloud::Ptr cloud(new maps::PointCloud());
    const int size = 1000000;
    cloud->reserve(size);
    for (int i = 0; i < size; ++i) {
      maps::PointType pt;
      pt.x = 3*i; pt.y = 3*i+1; pt.z = 3*i+2;
      cloud->push_back(pt);
    }
    view.setResolution(0);
    view.set(cloud);
    drc::map_cloud_t msg;
    maps::LcmTranslator::toLcm(view, msg, 0);
    mLcmWrapper->get()->publish("TEST_CLOUD", &msg);
    mLcmWrapper->get()->publish("MAP_CLOUD", &msg);
    std::cout << "SENT " << cloud->size() << " POINTS" << std::endl;
  }

  void go() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mLcmWrapper->get()->subscribe("TEST_BLOB",&State::onBlob,this);
    mLcmWrapper->get()->subscribe("TEST_CLOUD",&State::onCloud,this);
    mBotWrapper.reset(new maps::BotWrapper(mLcmWrapper->get()));
    mPublishThread = std::thread(std::ref(*this));
    mViewClient.reset(new maps::ViewClient());
    mViewClient->setBotWrapper(mBotWrapper);
    mViewClient->start();
    mLcmWrapper->startHandleThread(true);
  }
};

int main() {
  State state;
  state.go();
}
