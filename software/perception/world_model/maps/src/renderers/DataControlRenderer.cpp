#include "RendererBase.hpp"

#include <iostream>
#include <unordered_map>

#include <gtkmm.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>

#include <lcmtypes/drc/data_request_list_t.hpp>
#include <lcmtypes/drc/twist_timed_t.hpp>

#include <bot_vis/viewer.h>

// TODO: slow sweep commands

using namespace maps;

class DataControlRenderer : public RendererBase {
protected:
  struct RequestControl {
    const int mId;
    bool mEnabled;
    double mPeriod;
    typedef boost::shared_ptr<RequestControl> Ptr;
  };

protected:
  Gtk::VBox* mRequestControlBox;
  std::unordered_map<int, RequestControl::Ptr> mRequestControls;
  int mSpinRate;

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
  }

  void setupWidgets() {
    Gtk::Container* container = getGtkContainer();

    Gtk::Notebook* notebook = Gtk::manage(new Gtk::Notebook());

    // for data requests
    mRequestControlBox = Gtk::manage(new Gtk::VBox());
    addControl(drc::data_request_t::CAMERA_IMAGE, "Camera Image");
    addControl(drc::data_request_t::MINIMAL_ROBOT_STATE, "Robot State");
    addControl(drc::data_request_t::MAP_CATALOG, "Map Catalog");
    addControl(drc::data_request_t::OCTREE_SCENE, "Scene Octree");
    addControl(drc::data_request_t::HEIGHT_MAP_SCENE, "Scene Height");
    addControl(drc::data_request_t::HEIGHT_MAP_CORRIDOR, "Corridor Height");
    addControl(drc::data_request_t::DEPTH_MAP_SCENE, "Scene Depth");
    addControl(drc::data_request_t::DEPTH_MAP_WORKSPACE, "Workspace Depth");
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

  void addControl(const int iId, const std::string& iLabel) {
    Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
    Gtk::CheckButton* check = Gtk::manage(new Gtk::CheckButton());
    Gtk::Label* label = Gtk::manage(new Gtk::Label(iLabel));
    Gtk::SpinButton* spin = Gtk::manage(new Gtk::SpinButton());
    spin->set_range(0, 10);
    spin->set_increments(1, 2);
    spin->set_digits(0);
    box->add(*check);
    box->add(*label);
    box->add(*spin);
    RequestControl::Ptr group(new RequestControl());
    group->mEnabled = false;
    group->mPeriod = 0;
    bind(check, iLabel + " enable", group->mEnabled);
    bind(spin, iLabel + " period", group->mPeriod);
    mRequestControlBox->add(*box);
    mRequestControls[iId] = group;
  }

  ~DataControlRenderer() {
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
