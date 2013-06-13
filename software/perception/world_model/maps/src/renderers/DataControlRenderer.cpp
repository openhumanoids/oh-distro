#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <unordered_map>

#include <gtkmm.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>
#include <drc_utils/PointerUtils.hpp>
#include <gtkmm-renderer/RendererBase.hpp>

#include <lcmtypes/drc/data_request_list_t.hpp>
#include <lcmtypes/drc/sensor_request_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/neck_pitch_t.hpp>
#include <lcmtypes/drc/affordance_mini_t.hpp>
#include <lcmtypes/drc/affordance_mini_collection_t.hpp>
#include <lcmtypes/drc/simple_grasp_t.hpp>
#include <lcmtypes/drc/motionest_request_t.hpp>


#include <bot_vis/viewer.h>
#include <affordance/AffordanceUpWrapper.h>

// convenience class for list boxes
struct ComboColumns : public Gtk::TreeModel::ColumnRecord {
  Gtk::TreeModelColumn<int> mId;
  Gtk::TreeModelColumn<Glib::ustring> mLabel;
  ComboColumns() { add(mId); add(mLabel); }
};



class DataControlRenderer : public gtkmm::RendererBase {
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

protected:
  Gtk::VBox* mRequestControlBox;
  Gtk::VBox* mPushControlBox;
  std::unordered_map<int, RequestControl::Ptr> mRequestControls;
  int mSpinRate;
  int mHeadCameraFrameRate;
  int mHandCameraFrameRate;
  int mCameraCompression;
  bool mVOIMU; 
  int mHeadPitchAngle;
  int mLeftGraspState;
  int mRightGraspState;
  bool mMinimalAffordances;

  Glib::RefPtr<Gtk::ListStore> mAffordanceTreeModel;
  Gtk::TreeView* mAffordanceListBox;
  std::shared_ptr<affordance::AffordanceUpWrapper> mAffordanceWrapper;
  std::unordered_map<int,std::string> mAffordanceNames;

  std::unordered_map<std::string, ChannelType> mChannels;
  std::unordered_map<std::string, TimeKeeper> mTimeKeepers;
  std::mutex mTimeKeepersMutex;

public:

  DataControlRenderer(BotViewer* iViewer, const int iPriority,
                      const lcm_t* iLcm,
                      const BotParam* iParam, const BotFrames* iFrames)
    : gtkmm::RendererBase("Data Control", iViewer, iPriority, iLcm,
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
      (sigc::mem_fun(*this, &DataControlRenderer::checkTimers), 500);

    // create affordance update timer
    Glib::signal_timeout().connect
      (sigc::mem_fun(*this, &DataControlRenderer::checkAffordances), 500);
  }

  ~DataControlRenderer() {
  }

  bool checkTimers() {
    int64_t currentTime = drc::Clock::instance()->getCurrentWallTime();
    std::unordered_map<std::string, TimeKeeper>::const_iterator iter;
    {
      std::lock_guard<std::mutex> lock(mTimeKeepersMutex);
      for (iter = mTimeKeepers.begin(); iter != mTimeKeepers.end(); ++iter) {
	int64_t lastUpdateTime = iter->second.mLastUpdateTime;
	if (lastUpdateTime < 0) continue;
	int dtSec = (currentTime - lastUpdateTime)/1000000;
	if (dtSec > 0) {
	  std::string text;
          Gdk::Color color;
          if (dtSec <= 60) {
            text = static_cast<std::ostringstream*>
              (&(std::ostringstream() << dtSec) )->str();
            text = "(" + text + "s)";
            color.set_rgb_p(0, 0, 0);
          }
          else {
            text = "(>60s)";
            color.set_rgb_p(0.7, 0, 0);
          }
          iter->second.mLabel->modify_fg(Gtk::STATE_NORMAL, color);
	  iter->second.mLabel->set_text(text);
	}
      }
    }
    return true;
  }

  std::vector<int> getSelectedAffordanceIds() {
    struct Functor {
      std::vector<int> mIds;
      void callback(const Gtk::TreeModel::iterator& iIter) {
        ComboColumns columns;
        Gtk::TreeModel::Row row = *iIter;
        mIds.push_back(row[columns.mId]);
      }
    };

    ComboColumns columns;
    Functor functor;
    auto activeRows = mAffordanceListBox->get_selection();
    activeRows->selected_foreach_iter
      (sigc::mem_fun(functor, &Functor::callback) );
    return functor.mIds;
  }

  bool checkAffordances() {
    std::vector<int> selectedIds = getSelectedAffordanceIds();

    // check whether affordances have changed
    std::vector<affordance::AffConstPtr> affordances;
    mAffordanceWrapper->getAllAffordances(affordances);
    bool affordancesChanged = false;
    std::unordered_map<int,std::string> newNames;
    for (size_t i = 0; i < affordances.size(); ++i) {
      char name[256];
      sprintf(name, "%d - %s", affordances[i]->_uid,
              affordances[i]->getName().c_str());
      newNames[affordances[i]->_uid] = name;
      if ((mAffordanceNames.find(affordances[i]->_uid) ==
           mAffordanceNames.end()) ||
          (mAffordanceNames[affordances[i]->_uid] != name)) {
        affordancesChanged = true;
      }
    }
    if (!affordancesChanged && (mAffordanceNames.size() != newNames.size())) {
      affordancesChanged = true;
    }

    // populate new list
    ComboColumns columns;
    auto activeRows = mAffordanceListBox->get_selection();
    if (affordancesChanged) {
      mAffordanceTreeModel->clear();
      for (size_t i = 0; i < affordances.size(); ++i) {
        Gtk::TreeModel::iterator localIter = mAffordanceTreeModel->append();
        const Gtk::TreeModel::Row& row = *localIter;
        row[columns.mId] = affordances[i]->_uid;
        row[columns.mLabel] = newNames[affordances[i]->_uid];
        if (std::find(selectedIds.begin(), selectedIds.end(),
                      affordances[i]->_uid) != selectedIds.end()) {
          activeRows->select(localIter);
        }
      }
      mAffordanceNames = newNames;
    }

    return true;
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
    {
      std::lock_guard<std::mutex> lock(mTimeKeepersMutex);
      auto item = mTimeKeepers.find(key);
      if (item != mTimeKeepers.end()) {
        item->second.mLastUpdateTime =
          drc::Clock::instance()->getCurrentWallTime();
        item->second.mLabel->set_text("");
      }
    }
  }

  void setupWidgets() {
    Gtk::Container* container = getGtkContainer();

    Gtk::Notebook* notebook = Gtk::manage(new Gtk::Notebook());

    // for data requests
    mRequestControlBox = Gtk::manage(new Gtk::VBox());
    typedef drc::data_request_t dr;
    addControl(drc::data_request_t::CAMERA_IMAGE_HEAD, "Camera Head",
               "CAMERALEFT", ChannelTypeAnonymous);
    addControl(drc::data_request_t::CAMERA_IMAGE_LHAND, "Camera L.Hand",
               "CAMERA_LHANDLEFT", ChannelTypeAnonymous);
    addControl(drc::data_request_t::CAMERA_IMAGE_RHAND, "Camera R.Hand",
               "CAMERA_RHANDLEFT", ChannelTypeAnonymous);
    mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    
    /*
    addControl(drc::data_request_t::MINIMAL_ROBOT_STATE, "Robot State",
               "EST_ROBOT_STATE", ChannelTypeAnonymous);
    */
    addControl(drc::data_request_t::AFFORDANCE_LIST, "Affordance List",
               "AFFORDANCE_LIST", ChannelTypeAnonymous);
    mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    /*
    addControl(drc::data_request_t::MAP_CATALOG, "Map Catalog",
               "MAP_CATALOG", ChannelTypeAnonymous);
    */
    /*
    addControl(drc::data_request_t::OCTREE_SCENE, "Scene Octree",
               "MAP_OCTREE", ChannelTypeAnonymous);
    */
    addControl(drc::data_request_t::HEIGHT_MAP_SCENE, "Scene Height",
               "MAP_DEPTH", ChannelTypeDepthImage);
    /*
    addControl(drc::data_request_t::HEIGHT_MAP_CORRIDOR, "Corridor Height",
               "MAP_DEPTH", ChannelTypeDepthImage);
    */
    addControl(drc::data_request_t::HEIGHT_MAP_COARSE, "Coarse Height",
               "MAP_DEPTH", ChannelTypeDepthImage);
    addControl(drc::data_request_t::DEPTH_MAP_SCENE, "Scene Depth",
               "MAP_DEPTH", ChannelTypeDepthImage);
    addControl(drc::data_request_t::DEPTH_MAP_WORKSPACE, "Workspace Depth",
               "MAP_DEPTH", ChannelTypeDepthImage);
    mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    addControl(drc::data_request_t::TERRAIN_COST, "Terrain Cost",
               "TERRAIN_DIST_MAP", ChannelTypeAnonymous);
    addControl(drc::data_request_t::HEIGHT_MAP_DENSE, "*DENSE HEIGHT!!*",
               "MAP_DEPTH", ChannelTypeDepthImage);
    Gtk::Button* button = Gtk::manage(new Gtk::Button("Submit Request"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onDataRequestButton));
    mRequestControlBox->add(*button);
    notebook->append_page(*mRequestControlBox, "Pull");

    // for pushing data (e.g., affordances)
    mPushControlBox = Gtk::manage(new Gtk::VBox());
    Gtk::VBox* vbox = Gtk::manage(new Gtk::VBox());
    Gtk::Label* label =
      Gtk::manage(new Gtk::Label("Affordances", Gtk::ALIGN_LEFT));
    vbox->pack_start(*label, false, false);
    mMinimalAffordances = false;
    addCheck("Minimal", mMinimalAffordances, vbox);
    Gtk::ScrolledWindow* scroll = Gtk::manage(new Gtk::ScrolledWindow());
    scroll->set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
    scroll->set_size_request(-1, 100);
    ComboColumns columns;
    mAffordanceTreeModel = Gtk::ListStore::create(ComboColumns());
    mAffordanceListBox = Gtk::manage(new Gtk::TreeView());
    mAffordanceListBox->set_model(mAffordanceTreeModel);
    mAffordanceListBox->append_column("name",columns.mLabel);
    mAffordanceListBox->get_selection()->set_mode(Gtk::SELECTION_MULTIPLE);
    scroll->add(*mAffordanceListBox);
    vbox->pack_start(*scroll, false, false);
    mPushControlBox->pack_start(*vbox, false, false);
    button = Gtk::manage(new Gtk::Button("Push"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onDataPushButton));
    mPushControlBox->pack_start(*button, false, false);
    notebook->append_page(*mPushControlBox, "Push");

    // for sensor control
    Gtk::VBox* sensorControlBox = Gtk::manage(new Gtk::VBox());
    mSpinRate = 7;
    addSpin("Spin Rate (rpm)", mSpinRate, -1, 60, 1, sensorControlBox);
    mHeadCameraFrameRate = 5;
    addSpin("Head Cam fps", mHeadCameraFrameRate, -1, 10, 1, sensorControlBox); // maxing out at 10hz for safety
    mHandCameraFrameRate = 5;
    addSpin("Hands Cam fps", mHandCameraFrameRate, -1, 10, 1, sensorControlBox); // maxing out at 10hz for safety
    mCameraCompression = 0;
    std::vector<std::string> labels = { "-", "Low", "Med", "High" };
    std::vector<int> ids =
      { -1, drc::sensor_request_t::QUALITY_LOW,
        drc::sensor_request_t::QUALITY_MED,
        drc::sensor_request_t::QUALITY_HIGH};
    addCombo("Camera Quality", mCameraCompression, labels,
             ids, sensorControlBox);

    button = Gtk::manage(new Gtk::Button("Submit Sensor Config"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onSendRatesControlButton));
    sensorControlBox->pack_start(*button, false, false);

    mHeadPitchAngle = 45;
    addSpin("Pitch (deg)", mHeadPitchAngle, -90, 90, 5, sensorControlBox);
    button = Gtk::manage(new Gtk::Button("Submit Head Pitch"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onHeadPitchControlButton));
    sensorControlBox->pack_start(*button, false, false);
    
    //
    // grasp
    //
    labels = { "Leave", "Closed", "Open" };
    ids = { drc::simple_grasp_t::UNCHANGED, drc::simple_grasp_t::CLOSED,
            drc::simple_grasp_t::OPEN };
    Glib::RefPtr<Gtk::ListStore> treeModel = Gtk::ListStore::create(columns);
    for (size_t i = 0; i < ids.size(); ++i) {
      const Gtk::TreeModel::Row& row = *(treeModel->append());
      row[columns.mId] = ids[i];
      row[columns.mLabel] = labels[i];
    }

    // left
    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());
    Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
    Gtk::ComboBox* combo = Gtk::manage(new Gtk::ComboBox());
    combo->set_model(treeModel);
    combo->pack_start(columns.mLabel);
    mLeftGraspState = drc::simple_grasp_t::UNCHANGED;
    bind(combo, "LeftGrasp", mLeftGraspState);
    combo->set_active(mLeftGraspState);
    label = Gtk::manage(new Gtk::Label("left"));
    box->pack_start(*label,false,false);
    box->pack_start(*combo,false,false);
    hbox->add(*box);

    // right
    box = Gtk::manage(new Gtk::HBox());
    combo = Gtk::manage(new Gtk::ComboBox());
    combo->set_model(treeModel);
    combo->pack_start(columns.mLabel);
    mRightGraspState = drc::simple_grasp_t::UNCHANGED;
    bind(combo, "RightGrasp", mRightGraspState);
    combo->set_active(mRightGraspState);
    label = Gtk::manage(new Gtk::Label("right"));
    box->pack_start(*label,false,false);
    box->pack_start(*combo,false,false);
    hbox->add(*box);

    // send button
    button = Gtk::manage(new Gtk::Button("Grasp"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onGraspButton));
    hbox->add(*button);
    sensorControlBox->pack_start(*hbox, false, false);
    
    notebook->append_page(*sensorControlBox, "Sensor");
    
    Gtk::VBox* fusionControlBox = Gtk::manage(new Gtk::VBox());
    addCheck("Enable VO-IMU", mVOIMU, fusionControlBox);   
    button = Gtk::manage(new Gtk::Button("Submit Fusion Config"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onSendFusionButton));
    fusionControlBox->pack_start(*button, false, false);
    
    notebook->append_page(*fusionControlBox, "Fusion");

    container->add(*notebook);
    container->show_all();
  }

  void addControl(const int iId, const std::string& iLabel,
                  const std::string& iChannel, const ChannelType iChannelType) {
    Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
    Gtk::CheckButton* check = Gtk::manage(new Gtk::CheckButton());
    Gtk::Label* label = Gtk::manage(new Gtk::Label(iLabel));
    Gtk::SpinButton* spin = Gtk::manage(new Gtk::SpinButton());
    Gtk::Label* ageLabel = Gtk::manage(new Gtk::Label(" "));
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
    {
      std::lock_guard<std::mutex> lock(mTimeKeepersMutex);
      mTimeKeepers[key] = timeKeeper;
    }

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

  void onDataPushButton() {
    std::vector<int> selectedIds = getSelectedAffordanceIds();
    std::vector<affordance::AffPlusPtr> affordances;
    mAffordanceWrapper->getAllAffordancesPlus(affordances);

    if (!mMinimalAffordances) {
      drc::affordance_plus_collection_t msg;
      msg.name = "updateFromDataControlRenderer";
      msg.utime = drc::Clock::instance()->getCurrentTime();
      msg.map_id = -1;
      for (size_t i = 0; i < affordances.size(); ++i) {
        if (std::find(selectedIds.begin(), selectedIds.end(),
                      affordances[i]->aff->_uid) == selectedIds.end()) continue;
        drc::affordance_plus_t aff;
        affordances[i]->toMsg(&aff);
        aff.aff.aff_store_control = drc::affordance_t::UPDATE;
        msg.affs_plus.push_back(aff);
      }
      msg.naffs = msg.affs_plus.size();
      if (msg.naffs > 0) {
        getLcm()->publish(affordance::AffordanceServer::
                          AFFORDANCE_PLUS_BOT_OVERWRITE_CHANNEL, &msg);
        std::cout << "Sent " << msg.naffs << " affordances" << std::endl;
      }
      else std::cout << "Warning: did not send any affordances" << std::endl;
    }

    else {
      drc::affordance_mini_collection_t msg;
      for (size_t i = 0; i < affordances.size(); ++i) {
        if (std::find(selectedIds.begin(), selectedIds.end(),
                      affordances[i]->aff->_uid) == selectedIds.end()) continue;
        affordance::AffPtr aff = affordances[i]->aff;
        drc::affordance_mini_t msgAff;
        msgAff.uid = aff->_uid;
        for (int k = 0; k < 3; ++k) {
          msgAff.origin_xyz[k] = aff->_origin_xyz[k];
          msgAff.origin_rpy[k] = aff->_origin_rpy[k];
        }

        if (aff->getType() == affordance::AffordanceState::CAR) {
          msgAff.type = drc::affordance_mini_t::CAR;
          msgAff.nparams = 0;
          if (aff->_modelfile == "car.pcd") {
            msgAff.modelfile = drc::affordance_mini_t::CAR_PCD;
          }
          else if (aff->_modelfile == "car_cabin_2cm.pcd") {
            msgAff.modelfile = drc::affordance_mini_t::CABIN_2CM_PCD;
          }
          else {
            msgAff.modelfile = drc::affordance_mini_t::UNKNOWN_FILE;
          }
        }

        else if (aff->getType() == affordance::AffordanceState::CYLINDER) {
          msgAff.type = drc::affordance_mini_t::CYLINDER;
          msgAff.nparams = 2;
          msgAff.param_names.resize(2);
          msgAff.param_names[0] = drc::affordance_mini_t::LENGTH;
          msgAff.param_names[1] = drc::affordance_mini_t::RADIUS;
          msgAff.params.resize(2);
          msgAff.params[0] =
            aff->_params[affordance::AffordanceState::LENGTH_NAME];
          msgAff.params[1] =
            aff->_params[affordance::AffordanceState::RADIUS_NAME];
          msgAff.modelfile = drc::affordance_mini_t::UNKNOWN_FILE;
        }

        else continue;

        msg.affs.push_back(msgAff);
      }
      msg.naffs = msg.affs.size();
      if (msg.naffs > 0) {
        getLcm()->publish("AFFORDANCE_MINI_BOT_OVERWRITE", &msg);
        std::cout << "Sent " << msg.naffs << " affordances" << std::endl;
      }
      else std::cout << "Warning: did not send any affordances" << std::endl;
    }
  }

  void onSendRatesControlButton() {
    drc::sensor_request_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.spindle_rpm = mSpinRate;
    msg.head_fps = mHeadCameraFrameRate;
    msg.hand_fps = mHandCameraFrameRate;
    msg.camera_compression = mCameraCompression;
    getLcm()->publish("SENSOR_REQUEST", &msg);
    // TODO: set all to -1
  }
    
  void onHeadPitchControlButton() {
    const double kPi = 4*atan(1);
    double degreesToRadians = kPi/180;
    drc::neck_pitch_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.pitch = mHeadPitchAngle*degreesToRadians;
    getLcm()->publish("DESIRED_NECK_PITCH", &msg);
  }

  void onGraspButton() {
    drc::simple_grasp_t msg;
    msg.left_state = mLeftGraspState;
    msg.right_state = mRightGraspState;
    if ((mLeftGraspState != drc::simple_grasp_t::UNCHANGED) ||
        (mRightGraspState != drc::simple_grasp_t::UNCHANGED)) {
      getLcm()->publish("SIMPLE_GRASP_COMMAND", &msg);
    }
  }

  void onSendFusionButton() {
    drc::motionest_request_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    if (mVOIMU){
      msg.fusion_mode = (int8_t) drc::motionest_request_t::VO;
    }else{
      msg.fusion_mode = (int8_t) drc::motionest_request_t::DEFAULT;
    }
    getLcm()->publish("MOTIONEST_REQUEST", &msg);
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
