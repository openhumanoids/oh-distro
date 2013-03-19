#include "RendererBase.hpp"

#include <iostream>
#include <sstream>
#include <thread>
#include <unordered_map>

#include <gtkmm.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>

#include <lcmtypes/drc/data_request_list_t.hpp>
#include <lcmtypes/drc/twist_timed_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>

#include <bot_vis/viewer.h>

// TODO: slow sweep commands

using namespace maps;

class DataControlRenderer : public RendererBase {
protected:
  enum ChannelType {
    ChannelTypeAnonymous,
    ChannelTypeDepthImage,
  };

  struct RequestControl {
    const int mId;
    bool mEnabled;
    double mPeriod;
    typedef std::shared_ptr<RequestControl> Ptr;
  };

  struct TimeKeeper {
    int64_t mLastUpdateTime;
    Gtk::Label* mLabel;
    TimeKeeper() : mLastUpdateTime(-1), mLabel(NULL) {}
  };

  struct LastUpdateChecker {
    DataControlRenderer* mRenderer;
    std::thread mThread;
    bool mIsRunning;

    LastUpdateChecker(DataControlRenderer* iRenderer) : mRenderer(iRenderer) {
      mThread = std::thread(std::ref(*this));
    }

    ~LastUpdateChecker() {
      mIsRunning = false;
      mThread.join();
    }

    void operator()() {
      mIsRunning = true;
      while (mIsRunning) {
        int64_t currentTime = drc::Clock::instance()->getCurrentWallTime();
        std::unordered_map<std::string, TimeKeeper>::const_iterator iter;
        for (iter = mRenderer->mTimeKeepers.begin();
             iter != mRenderer->mTimeKeepers.end(); ++iter) {
          int64_t lastUpdateTime = iter->second.mLastUpdateTime;
          if (lastUpdateTime < 0) continue;
          int dtSec = (currentTime - lastUpdateTime)/1000000;
          if (dtSec > 0) {
            std::string text = static_cast<std::ostringstream*>
              (&(std::ostringstream() << dtSec) )->str();
            iter->second.mLabel->set_text("(" + text + "s)");
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }
    }
  };

protected:
  Gtk::VBox* mRequestControlBox;
  std::unordered_map<int, RequestControl::Ptr> mRequestControls;
  int mSpinRate;

  std::shared_ptr<LastUpdateChecker> mUpdateChecker;
  std::unordered_map<std::string, ChannelType> mChannels;
  std::unordered_map<std::string, TimeKeeper> mTimeKeepers;

public:

  DataControlRenderer(BotViewer* iViewer, const int iPriority,
                      const lcm_t* iLcm,
                      const BotParam* iParam, const BotFrames* iFrames)
    : RendererBase("Data Control", iViewer, iPriority, iLcm,
                   iParam, iFrames, 0) {

    // set up robot time clock
    drc::Clock::instance()->setLcm(getLcm());
    drc::Clock::instance()->setVerbose(false);

    // create and show ui widgets
    setupWidgets();

    // create subscriptions and handlers
    mUpdateChecker.reset(new LastUpdateChecker(this));
  }

  ~DataControlRenderer() {
  }

  void onMessage(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel) {
    std::unordered_map<std::string, ChannelType>::const_iterator channelItem =
      mChannels.find(iChannel);
    if (channelItem == mChannels.end()) return;
    std::string key = iChannel;
    if (channelItem->second == ChannelTypeDepthImage) {
      drc::map_image_t msg;
      if (msg.decode(iBuf->data, 0, iBuf->data_size) < 0) {
        std::cout << "Error decoding " << iChannel << std::endl;
        return;
      }
      key += static_cast<std::ostringstream*>
        (&(std::ostringstream() << msg.view_id) )->str();
    }
    else {
    }
    mTimeKeepers[key].mLastUpdateTime =
      drc::Clock::instance()->getCurrentWallTime();
    mTimeKeepers[key].mLabel->set_text("");
  }

  void setupWidgets() {
    Gtk::Container* container = getGtkContainer();

    Gtk::Notebook* notebook = Gtk::manage(new Gtk::Notebook());

    // for data requests
    mRequestControlBox = Gtk::manage(new Gtk::VBox());
    typedef drc::data_request_t dr;
    addControl(drc::data_request_t::CAMERA_IMAGE, "Camera Image",
               "CAMERALEFT_COMPRESSED", ChannelTypeAnonymous);
    addControl(drc::data_request_t::MINIMAL_ROBOT_STATE, "Robot State",
               "EST_ROBOT_STATE", ChannelTypeAnonymous);
    addControl(drc::data_request_t::AFFORDANCE_LIST, "Affordance List",
               "AFFORDANCE_LIST", ChannelTypeAnonymous);
    addControl(drc::data_request_t::MAP_CATALOG, "Map Catalog",
               "MAP_CATALOG", ChannelTypeDepthImage);
    addControl(drc::data_request_t::OCTREE_SCENE, "Scene Octree",
               "MAP_OCTREE", ChannelTypeAnonymous);
    addControl(drc::data_request_t::HEIGHT_MAP_SCENE, "Scene Height",
               "MAP_DEPTH", ChannelTypeDepthImage);
    addControl(drc::data_request_t::HEIGHT_MAP_CORRIDOR, "Corridor Height",
               "MAP_DEPTH", ChannelTypeDepthImage);
    addControl(drc::data_request_t::DEPTH_MAP_SCENE, "Scene Depth",
               "MAP_DEPTH", ChannelTypeDepthImage);
    addControl(drc::data_request_t::DEPTH_MAP_WORKSPACE, "Workspace Depth",
               "MAP_DEPTH", ChannelTypeDepthImage);
    Gtk::Button* button = Gtk::manage(new Gtk::Button("Submit Request"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onDataRequestButton));
    mRequestControlBox->add(*button);
    notebook->append_page(*mRequestControlBox, "Data");

    // for sensor control
    Gtk::VBox* sensorControlBox = Gtk::manage(new Gtk::VBox());
    mSpinRate = 15;
    addSpin("Spin Rate (rpm)", mSpinRate, 0, 60, 1, sensorControlBox);
    button = Gtk::manage(new Gtk::Button("Submit Rate"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onRateControlButton));
    sensorControlBox->pack_start(*button, false, false);
    notebook->append_page(*sensorControlBox, "Sensor");
    
    container->add(*notebook);
    container->show_all();
  }

  void addControl(const int iId, const std::string& iLabel,
                  const std::string& iChannel, const ChannelType iChannelType) {
    Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
    Gtk::CheckButton* check = Gtk::manage(new Gtk::CheckButton());
    Gtk::Label* label = Gtk::manage(new Gtk::Label(iLabel));
    Gtk::SpinButton* spin = Gtk::manage(new Gtk::SpinButton());
    Gtk::Label* ageLabel = Gtk::manage(new Gtk::Label(""));
    spin->set_range(0, 10);
    spin->set_increments(1, 2);
    spin->set_digits(0);
    box->add(*check);
    box->add(*label);
    box->add(*ageLabel);
    box->add(*spin);
    RequestControl::Ptr group(new RequestControl());
    group->mEnabled = false;
    group->mPeriod = 0;
    bind(check, iLabel + " enable", group->mEnabled);
    bind(spin, iLabel + " period", group->mPeriod);
    mRequestControlBox->add(*box);
    mRequestControls[iId] = group;

    std::string key = iChannel;
    if (iChannelType == ChannelTypeDepthImage) {
      key += static_cast<std::ostringstream*>
        (&(std::ostringstream() << iId) )->str();
    }
    TimeKeeper timeKeeper;
    timeKeeper.mLabel = ageLabel;
    timeKeeper.mLastUpdateTime = -1;
    mTimeKeepers[key] = timeKeeper;

    if (mChannels.find(iChannel) == mChannels.end()) {
      add(getLcm()->subscribe(iChannel, &DataControlRenderer::onMessage, this));
      mChannels[iChannel] = iChannelType;
    }
  }

  void onDataRequestButton() {
    drc::data_request_list_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    std::unordered_map<int,RequestControl::Ptr>::const_iterator iter;
    for (iter = mRequestControls.begin();
         iter != mRequestControls.end(); ++iter) {
      if (!iter->second->mEnabled) continue;
      drc::data_request_t req;
      req.type = iter->first;
      req.period = (int)(iter->second->mPeriod*10);
      msg.requests.push_back(req);
    }
    msg.num_requests = msg.requests.size();
    if (msg.num_requests > 0) {
      getLcm()->publish("DATA_REQUEST", &msg);
    }
  }

  void onRateControlButton() {
    const double kPi = 4*atan(1);
    double radiansPerSecond = (double)mSpinRate/60*2*kPi;
    drc::twist_timed_t rate;
    rate.utime = drc::Clock::instance()->getCurrentTime();
    rate.angular_velocity.x = radiansPerSecond;
    rate.angular_velocity.y = 0.0;
    rate.angular_velocity.z = 0.0;
    rate.linear_velocity.x = 0.0;
    rate.linear_velocity.y = 0.0;
    rate.linear_velocity.z = 0.0;
    getLcm()->publish("SCAN_RATE_CMD", &rate);
  }

  void draw() {
    // intentionally left blank
  }
};


// this is the single setup method exposed for integration with the viewer
void data_control_renderer_setup(BotViewer* iViewer,
                                 const int iPriority,
                                 const lcm_t* iLcm,
                                 const BotParam* iParam,
                                 const BotFrames* iFrames) {
  new DataControlRenderer(iViewer, iPriority, iLcm, iParam, iFrames);
}
