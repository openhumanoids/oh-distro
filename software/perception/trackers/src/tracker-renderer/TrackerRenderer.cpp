#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <unordered_map>

#include <gtkmm.h>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>
#include <drc_utils/PointerUtils.hpp>
#include <gtkmm-renderer/RendererBase.hpp>
#include <affordance/AffordanceUpWrapper.h>

// convenience class for combo boxes
struct ComboColumns : public Gtk::TreeModel::ColumnRecord {
  Gtk::TreeModelColumn<int> mId;
  Gtk::TreeModelColumn<Glib::ustring> mLabel;
  ComboColumns() { add(mId); add(mLabel); }
};


class TrackerRenderer : public gtkmm::RendererBase {
protected:
  std::shared_ptr<affordance::AffordanceUpWrapper> mAffordanceWrapper;

  int mTrackerId;
  int mTrackerType;

  Gtk::ComboBox* mAffordanceCombo;
  Glib::RefPtr<Gtk::ListStore> mAffordanceTreeModel;

public:

  TrackerRenderer(BotViewer* iViewer, const int iPriority,
                  const lcm_t* iLcm,
                  const BotParam* iParam, const BotFrames* iFrames)
    : gtkmm::RendererBase("Tracker", iViewer, iPriority, iLcm,
                          iParam, iFrames, 0) {

    // set up robot time clock
    drc::Clock::instance()->setLcm(getLcm());
    drc::Clock::instance()->setVerbose(false);

    // set up affordance wrapper
    mAffordanceWrapper.reset(new affordance::AffordanceUpWrapper
                             (drc::PointerUtils::boostPtr(getLcm())));

    // create and show ui widgets
    setupWidgets();

    // create update timer
    Glib::signal_timeout().connect
      (sigc::mem_fun(*this, &TrackerRenderer::checkAffordances), 1000);
  }

  ~TrackerRenderer() {
  }

  bool checkAffordances() {
    // save existing selection
    ComboColumns columns;
    Gtk::TreeModel::iterator iter = mAffordanceCombo->get_active();
    int id = -1;
    if(iter) {
      Gtk::TreeModel::Row row = *iter;
      id = row[columns.mId];
    }

    // populate new list
    std::vector<affordance::AffConstPtr> affordances;
    mAffordanceWrapper->getAllAffordances(affordances);
    mAffordanceTreeModel->clear();
    mAffordanceCombo->clear();
    for (size_t i = 0; i < affordances.size(); ++i) {
      Gtk::TreeModel::iterator localIter = mAffordanceTreeModel->append();
      const Gtk::TreeModel::Row& row = *localIter;
      row[columns.mId] = affordances[i]->_uid;
      char name[256];
      sprintf(name, "%d - %s", affordances[i]->_uid,
              affordances[i]->getName().c_str());
      row[columns.mLabel] = name;
      if (affordances[i]->_uid == id) iter = localIter;
    }
    mAffordanceCombo->pack_start(columns.mLabel);

    // set to previously active
    if (iter) mAffordanceCombo->set_active(iter);

    return true;
  }

  void setupWidgets() {
    Gtk::Container* container = getGtkContainer();

    // combo box for affordances
    Gtk::VBox* vbox = Gtk::manage(new Gtk::VBox());
    Gtk::Label* label =
      Gtk::manage(new Gtk::Label("Affordance", Gtk::ALIGN_LEFT));
    vbox->add(*label);
    ComboColumns columns;
    mAffordanceTreeModel = Gtk::ListStore::create(ComboColumns());
    mAffordanceCombo = Gtk::manage(new Gtk::ComboBox());
    mAffordanceCombo->set_model(mAffordanceTreeModel);
    vbox->add(*mAffordanceCombo);
    container->add(*vbox);

    // combo box for tracker id
    std::vector<int> ids;
    std::vector<std::string> labels;
    ids = { 0, 1, 2, 3, 4 };
    labels = { "0", "1", "2", "3", "4" };
    mTrackerId = 0;
    addCombo("Tracker ID", mTrackerId, labels, ids);

    // combo box for tracker types
    ids = { drc::tracker_command_t::STOP, drc::tracker_command_t::PLANE,
            drc::tracker_command_t::COLOR, drc::tracker_command_t::HISTOGRAM,
            drc::tracker_command_t::FEATURE, drc::tracker_command_t::ICP,
            drc::tracker_command_t::STEREOICP };
    labels = { "Stop", "Plane", "Color", "Histogram", "Feature", "ICP",
               "Stereo ICP" };
    mTrackerType = drc::tracker_command_t::STEREOICP;
    addCombo("Tracker Type", mTrackerType, labels, ids);

    // buttons for sending commands
    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());
    Gtk::Button* button = Gtk::manage(new Gtk::Button("Track"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &TrackerRenderer::onTrackButton));
    hbox->add(*button);
    button = Gtk::manage(new Gtk::Button("Stare"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &TrackerRenderer::onStareButton));
    hbox->add(*button);
    container->add(*hbox);

    // button for requesting affordance states
    Gtk::HSeparator* sep = Gtk::manage(new Gtk::HSeparator());
    container->add(*sep);
    button = Gtk::manage(new Gtk::Button("Update Affordances"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &TrackerRenderer::onUpdateAffordancesButton));
    container->add(*button);
    
    container->show_all();
  }

  void onTrackButton() {
    Gtk::TreeModel::iterator iter = mAffordanceCombo->get_active();
    if(!iter) return;
    Gtk::TreeModel::Row row = *iter;
    ComboColumns columns;
    int id = row[columns.mId];
    if (id < 0) return;

    drc::tracker_command_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.uid = id;
    msg.tracker_id = mTrackerId;
    msg.tracker_type = mTrackerType;
    msg.plane_uid = -1;
    getLcm()->publish("TRACKER_COMMAND", &msg);
  }

  void onStareButton() {
    Gtk::TreeModel::iterator iter = mAffordanceCombo->get_active();
    if(!iter) return;
    Gtk::TreeModel::Row row = *iter;
    ComboColumns columns;
    int id = row[columns.mId];
    if (id < 0) return;

    drc::gaze_command_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.affordance_id = id;
    getLcm()->publish("GAZE_COMMAND", &msg);
  }

  void onUpdateAffordancesButton() {
    drc::data_request_list_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    drc::data_request_t req;
    req.type = drc::data_request_t::AFFORDANCE_LIST;
    req.period = 0;
    msg.requests.push_back(req);
    msg.num_requests = msg.requests.size();
    getLcm()->publish("DATA_REQUEST", &msg);
  }

  void draw() {
    // intentionally left blank
  }
};


// this is the single setup method exposed for integration with the viewer
void tracker_renderer_setup(BotViewer* iViewer,
                            const int iPriority,
                            const lcm_t* iLcm,
                            const BotParam* iParam,
                            const BotFrames* iFrames) {
  new TrackerRenderer(iViewer, iPriority, iLcm, iParam, iFrames);
}
