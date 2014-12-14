#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <string>
#include <algorithm>

#include <gtkmm.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm-cpp.hpp>
#include <drc_utils/Clock.hpp>
#include <drc_utils/PointerUtils.hpp>
#include <gtkmm-renderer/RendererBase.hpp>

#include <lcmtypes/drc/data_request_list_t.hpp>
#include <lcmtypes/drc/camera_settings_t.hpp>
#include <lcmtypes/drc/map_depth_settings_t.hpp>
#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/drc/neck_pitch_t.hpp>
#include <lcmtypes/drc/affordance_mini_t.hpp>
#include <lcmtypes/drc/affordance_mini_collection_t.hpp>
#include <lcmtypes/drc/simple_grasp_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>
#include <lcmtypes/drc/map_registration_command_t.hpp>
#include <lcmtypes/drc/motionest_request_t.hpp>
#include <lcmtypes/drc/robot_state_t.hpp>
#include <lcmtypes/drc/sandia_simple_grasp_t.hpp>

#include <lcmtypes/multisense.hpp>
#include <lcmtypes/irobothand.hpp>
#include <lcmtypes/robotiqhand.hpp>

#include <bot_vis/viewer.h>
#include <affordance/AffordanceUpWrapper.h>

#include <bot_param/param_client.h>
#include <drc_utils/BotWrapper.hpp>

#include "ParamManager.hpp"

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
    int mId;
    double mPeriod;
    int mQuality;
    bool mEnabled;
    typedef std::shared_ptr<RequestControl> Ptr;
  };

  struct TimeKeeper {
    int64_t mLastUpdateTime;
    Gtk::Label* mLabel;
    TimeKeeper() : mLastUpdateTime(-1), mLabel(NULL) {}
  };

protected:
  int mDummyIntValue;
  double mDummyDoubleValue;

  Gtk::VBox* mRequestControlBox;
  Gtk::Table* mRequestControlTable;
  Gtk::VBox* mAffControlBox;
  std::unordered_map<int, RequestControl::Ptr> mRequestControls;
  int mHandCameraFrameRate;
  bool mCameraAutoGain;

  int mLeftGraspState;
  int mLeftGraspNameEnum;
  int mRightGraspState;
  int mRightGraspNameEnum;
  std::vector <std::string>  mGraspNames;

  int mControlIrobotRightHand; // 0 left/ 1 right
  int mIrobotCalibrate;
  int mIrobotClosePercent;
  int mIrobotSpreadDegree;
  bool mIrobotFingerEnabled[3];

  int mControlRobotiqRightHand; // 0 left/ 1 right
  int mRobotiqMode;
  int mRobotiqClosePercent;
  int mRobotiqForce;
  int mRobotiqVelocity;
  bool mRoboticFingerEnabled[3];

  bool mMinimalAffordances;
  int mControllerHeightMapMode;

  int mWorkspaceDepthFov;
  int mWorkspaceDepthYaw;

  int mPresetId;
  std::vector<std::string> mPresetNames;
  
  Gtk::Label* mNeckPitchLabel;
  int64_t mLastNeckPitchLabelUpdateTime;

  Glib::RefPtr<Gtk::ListStore> mAffordanceTreeModel;
  Gtk::TreeView* mAffordanceListBox;
  std::shared_ptr<affordance::AffordanceUpWrapper> mAffordanceWrapper;
  std::unordered_map<int,std::string> mAffordanceNames;

  std::unordered_map<std::string, ChannelType> mChannels;
  std::unordered_map<std::string, TimeKeeper> mTimeKeepers;
  std::mutex mTimeKeepersMutex;

  std::shared_ptr<ParamManager> mParamManager;

public:

  DataControlRenderer(BotViewer* iViewer, const int iPriority,
                      const lcm_t* iLcm,
                      const BotParam* iParam, const BotFrames* iFrames)
    : gtkmm::RendererBase("Data Control", iViewer, iPriority, iLcm,
                          iParam, iFrames, 0) {

    mGraspNames = { "cylindrical", "spherical", "prismatic" };
    mLastNeckPitchLabelUpdateTime = 0;
    // set up robot time clock
    drc::Clock::instance()->setLcm(getLcm());
    drc::Clock::instance()->setVerbose(false);

    std::shared_ptr<drc::BotWrapper> updatingBotWrapper;
    lcm_t* lcm = const_cast<lcm_t*>(iLcm);
    BotParam* updatingBotParam = bot_param_new_from_server(lcm, 1);
    if (updatingBotParam == NULL) {
      updatingBotParam = getBotParam();
      std::cout << "Warning: DataControlRenderer: " <<
        "param server not found; params will not auto-update" << std::endl;
    }
    updatingBotWrapper.reset(new drc::BotWrapper(lcm, updatingBotParam));
    mParamManager.reset(new ParamManager(updatingBotWrapper));
    mParamManager->setKeyBase("datacontrol.pull");

    // set up affordance wrapper
    mAffordanceWrapper.reset(new affordance::AffordanceUpWrapper
                             (drc::PointerUtils::boostPtr(getLcm())));

    // create and show ui widgets
    setupWidgets();

    // create update timer
    Glib::signal_timeout().connect
      (sigc::mem_fun(*this, &DataControlRenderer::checkTimers), 500);
      
    // create a subscription to EST_ROBOT_STATE and update some widget labels
    getLcm()->subscribe("EST_ROBOT_STATE", &DataControlRenderer::onRobotState, this);
       
    // create affordance update timer
    Glib::signal_timeout().connect
      (sigc::mem_fun(*this, &DataControlRenderer::checkAffordances), 500);
  }

  ~DataControlRenderer() {
  }

  void onRobotState(const lcm::ReceiveBuffer* iBuf,
                    const std::string& iChannel,
                    const drc::robot_state_t* iMessage) { 
    auto found = std::find(iMessage->joint_name.begin(),
                           iMessage->joint_name.end(), "neck_ay"); 
    int64_t currentTime = iMessage->utime;
    int dtSec = (currentTime - mLastNeckPitchLabelUpdateTime)/1000000;
    if ((found != iMessage->joint_name.end()) && (dtSec>1)) {
      unsigned int index = found - iMessage->joint_name.begin();
      char valStr[64];
      sprintf(valStr,"%.1f", iMessage->joint_position[index]*(180/M_PI));
      std::string text = "Pitch (" + std::string(valStr) + " deg)";
      mNeckPitchLabel->set_text(text);
      mLastNeckPitchLabelUpdateTime = currentTime;
   }
  }	
  
  bool checkTimers() {
    int64_t currentTime = drc::Clock::instance()->getCurrentWallTime();
    std::unordered_map<std::string, TimeKeeper>::const_iterator iter;
    {
      std::lock_guard<std::mutex> lock(mTimeKeepersMutex);
      for (iter = mTimeKeepers.begin(); iter != mTimeKeepers.end(); ++iter) {
        int64_t lastUpdateTime = iter->second.mLastUpdateTime;
        int dtSec = (currentTime - lastUpdateTime)/1000000;
        //if (lastUpdateTime < 0) continue;
        if (lastUpdateTime < 0) dtSec = 100;
        if (dtSec == 0) continue;
        if (dtSec >= 0) {
          std::string text;
          Gdk::Color color;
          color.set_rgb_p(0, 0, 0);
          if (dtSec > 60) {
            color.set_rgb_p(0.7, 0, 0);
            text = ">60s";
          }
          else if (dtSec > 0) {
            text = static_cast<std::ostringstream*>
              (&(std::ostringstream() << dtSec) )->str();
            text = text + "s";
          }
          iter->second.mLabel->modify_fg(Gtk::STATE_NORMAL, color);
          iter->second.mLabel->set_text(" " + text + " ");
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
        item->second.mLabel->set_text(" 0s ");
        Gdk::Color color;
        color.set_rgb_p(0, 0.7, 0);
        item->second.mLabel->modify_fg(Gtk::STATE_NORMAL, color);
      }
    }
  }

  void loadPreset(const int iNum) {
    if (iNum < 0) return;
    drc::BotWrapper botWrapper(getLcm(), getBotParam(), getBotFrames());
    const std::string keyBase = "viewer.datacontrol_presets";
    ParamManager manager = *mParamManager;
    manager.setKeyBase(keyBase + "." + mPresetNames[iNum]);
    manager.onParamChange();
    std::cout << "Loaded preset " << mPresetNames[iNum] << std::endl;
  }

  void setupWidgets() {
    Gtk::Container* container = getGtkContainer();

    Gtk::Notebook* notebook = Gtk::manage(new Gtk::Notebook());

    // for data requests
    mRequestControlBox = Gtk::manage(new Gtk::VBox());
    mRequestControlTable = Gtk::manage(new Gtk::Table());
    mRequestControlBox->pack_start(*mRequestControlTable,false,false);
    typedef drc::data_request_t dr;
    addControl(drc::data_request_t::CAMERA_IMAGE_HEAD_LEFT, "Camera Head",
               "CAMERA_LEFT", ChannelTypeAnonymous,true);
    /*
    addControl(drc::data_request_t::CAMERA_IMAGE_HEAD_RIGHT, "Camera Head R.",
               "CAMERA_RIGHT", ChannelTypeAnonymous,true);
    */
    /*
    addControl(drc::data_request_t::CAMERA_IMAGE_LHAND, "Camera L.Hand",
               "CAMERA_LHANDLEFT", ChannelTypeAnonymous);
    addControl(drc::data_request_t::CAMERA_IMAGE_RHAND, "Camera R.Hand",
               "CAMERA_RHANDLEFT", ChannelTypeAnonymous);
    */
    addControl(drc::data_request_t::CAMERA_IMAGE_LCHEST, "Camera L.Chest",
               "CAMERACHEST_LEFT", ChannelTypeAnonymous,true);
    addControl(drc::data_request_t::CAMERA_IMAGE_RCHEST, "Camera R.Chest",
               "CAMERACHEST_RIGHT", ChannelTypeAnonymous,true);
    //mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));

    /*
    addControl(drc::data_request_t::MINIMAL_ROBOT_STATE, "Robot State",
               "EST_ROBOT_STATE", ChannelTypeAnonymous);
    mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    */
    /*
    addControl(drc::data_request_t::MAP_CATALOG, "Map Catalog",
               "MAP_CATALOG", ChannelTypeAnonymous);
    */
    /*
    addControl(drc::data_request_t::CLOUD_WORKSPACE, "Workspace Points",
               "MAP_CLOUD", ChannelTypeAnonymous);
    addControl(drc::data_request_t::OCTREE_WORKSPACE, "Workspace Octree",
               "MAP_OCTREE", ChannelTypeAnonymous);
    */


    addControl(drc::data_request_t::HEIGHT_MAP_SCENE, "Scene Height",
               "MAP_DEPTH", ChannelTypeDepthImage);


    /*
    addControl(drc::data_request_t::HEIGHT_MAP_CORRIDOR, "Corridor Height",
               "MAP_DEPTH", ChannelTypeDepthImage);
    */
    /*
    addControl(drc::data_request_t::HEIGHT_MAP_COARSE, "Coarse Height",
               "MAP_DEPTH", ChannelTypeDepthImage);
    */
    addControl(drc::data_request_t::DEPTH_MAP_SCENE, "Scene Depth",
               "MAP_DEPTH", ChannelTypeDepthImage);
    /*
    addControl(drc::data_request_t::OCTREE_SCENE, "Scene Points",
               "MAP_OCTREE", ChannelTypeAnonymous);
    */

    /*
    addControl(drc::data_request_t::DEPTH_MAP_WORKSPACE_C, "Workspace Depth C",
               "MAP_DEPTH", ChannelTypeDepthImage);
    addControl(drc::data_request_t::DEPTH_MAP_WORKSPACE_L, "Workspace Depth L",
               "MAP_DEPTH", ChannelTypeDepthImage);
    addControl(drc::data_request_t::DEPTH_MAP_WORKSPACE_R, "Workspace Depth R",
               "MAP_DEPTH", ChannelTypeDepthImage);
    */

    //mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    //addControl(drc::data_request_t::TERRAIN_COST, "Terrain Cost",
    //           "TERRAIN_DIST_MAP", ChannelTypeAnonymous);
    //addControl(drc::data_request_t::HEIGHT_MAP_DENSE, "*DENSE HEIGHT!!*",
    //           "MAP_DEPTH", ChannelTypeDepthImage);

    /*
    addControl(drc::data_request_t::DENSE_CLOUD_LHAND, "Dense Box L.Hand",
               "MAP_CLOUD", ChannelTypeDepthImage);
    addControl(drc::data_request_t::DENSE_CLOUD_RHAND, "Dense Box R.Hand",
               "MAP_CLOUD", ChannelTypeDepthImage);
    */

    addWorkspaceViewWidgets();
    addControl(drc::data_request_t::DEPTH_MAP_WORKSPACE, "Workspace Depth",
               "MAP_DEPTH", ChannelTypeDepthImage);

    // some widgets and variables
    Gtk::Button* button;
    Gtk::SpinButton* spin;
    Gtk::ComboBox* combo;
    Gtk::Label* label;
    Gtk::VBox* vbox;
    Gtk::HBox* hbox;
    Gtk::Box* box;
    Gtk::Table* table;
    Gtk::CheckButton* check;
    Gtk::AttachOptions xOpts = Gtk::FILL | Gtk::EXPAND;
    Gtk::AttachOptions yOpts = Gtk::SHRINK;
    int xCur(0), yCur(0);

    // request button
    /* TODO: disabled for now
    button = Gtk::manage(new Gtk::Button("Submit Request"));
    button->signal_clicked().connect
      ([this]{this->sendDataRequest(this->mRequestControls);});
    mRequestControlBox->add(*button);
    */

    // set up preset controls for pull
    {
      hbox = Gtk::manage(new Gtk::HBox());
      label = Gtk::manage(new Gtk::Label("preset"));
      drc::BotWrapper botWrapper(getLcm(), getBotParam(), getBotFrames());
      const std::string keyBase = "viewer.datacontrol_presets";
      mPresetNames = botWrapper.getKeys(keyBase);
      std::vector<std::string> presetLabels(mPresetNames.size());
      for (size_t i = 0; i < presetLabels.size(); ++i) {
        presetLabels[i] =
          botWrapper.get(keyBase + "." + mPresetNames[i] + ".label");
      }
      std::vector<int> ids(presetLabels.size());
      for (size_t i = 0; i < ids.size(); ++i) ids[i] = i;
      mPresetId = -1;
      Gtk::ComboBox* combo = createCombo(mPresetId,presetLabels,ids);
      combo->signal_changed().connect([this]{loadPreset(mPresetId);});
      button = Gtk::manage(new Gtk::Button("apply all"));
      button->signal_clicked().connect([this]{sendAllDataRequests();});
      hbox->pack_start(*label,false,false);
      hbox->pack_start(*combo,false,false);
      hbox->pack_start(*button,false,false);
      mRequestControlBox->add(*hbox);
    }

    notebook->append_page(*mRequestControlBox, "Pull");

    // ground filter
    hbox = Gtk::manage(new Gtk::HBox());
    check = Gtk::manage(new Gtk::CheckButton("filter ground"));
    button = Gtk::manage(new Gtk::Button("send"));
    hbox->pack_start(*check,false,false);
    hbox->pack_start(*button,false,false);
    button->signal_clicked().connect
      ([this,check]{
        drc::map_depth_settings_t msg;
        msg.data_request.type = drc::data_request_t::DEPTH_MAP_WORKSPACE;
        msg.data_request.period = -1;
        msg.remove_ground = check->get_active();
        this->getLcm()->publish("MAP_DEPTH_SETTINGS",&msg);
      });
    mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    mRequestControlBox->add(*hbox);

    // neck pitch
    mNeckPitchLabel= Gtk::manage(new Gtk::Label("Pitch (deg)"));
    mDummyIntValue = 45;
    spin = gtkmm::RendererBase::createSpin(mDummyIntValue, -90, 90, 5);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,spin]{this->onHeadPitchControlButton(spin->get_value());});
    hbox = Gtk::manage(new Gtk::HBox());
    hbox->pack_start(*mNeckPitchLabel, false, false);
    hbox->pack_start(*spin, false, false);
    hbox->pack_start(*button, false, false);
    mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    mRequestControlBox->pack_start(*hbox, false, false);

    // for pushing data (e.g., affordances)
    mAffControlBox = Gtk::manage(new Gtk::VBox());
    vbox = Gtk::manage(new Gtk::VBox());
    label = Gtk::manage(new Gtk::Label("Aff", Gtk::ALIGN_LEFT));
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
    mAffControlBox->pack_start(*vbox, false, false);
    button = Gtk::manage(new Gtk::Button("Push"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onDataPushButton));
    mAffControlBox->pack_start(*button, false, false);
    mAffControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    /*
    addControl(drc::data_request_t::AFFORDANCE_LIST, "Affordance List",
               "AFFORDANCE_LIST", ChannelTypeAnonymous, false, mAffControlBox);
    */
    button = Gtk::manage(new Gtk::Button("Pull"));
    /* TODO: disabled for now
    button->signal_clicked().connect
      ([this]{this->sendDataRequest(this->mRequestControls);});
    */
    mAffControlBox->pack_start(*button,false,false);
    {
      box = Gtk::manage(new Gtk::HBox());
      label = Gtk::manage(new Gtk::Label("AutoCorrect",Gtk::ALIGN_RIGHT));
      check = Gtk::manage(new Gtk::CheckButton());
      check->set_active(false);
      button = Gtk::manage(new Gtk::Button("send"));
      button->signal_clicked().connect([this,check]{
          drc::map_registration_command_t msg;
          msg.utime = now();
          msg.command = check->get_active() ?
            (int)drc::map_registration_command_t::AFF_UPDATE_START :
            (int)drc::map_registration_command_t::AFF_UPDATE_PAUSE;
          getLcm()->publish("MAP_REGISTRATION_COMMAND", &msg);
        });
      box->pack_start(*label,false,false);
      box->pack_start(*check,false,false);
      box->pack_start(*button,false,false);
      mAffControlBox->pack_start(*box,false,false);
    }
    notebook->append_page(*mAffControlBox, "Affs");


    // Hands
    Gtk::VBox* handControlBox = Gtk::manage(new Gtk::VBox());

    /////////////////////////////////
    // sandia hands
    //

    label = Gtk::manage(new Gtk::Label("Sandia Hand", Gtk::ALIGN_CENTER));
    handControlBox->pack_start(*label,false,false);
    table = Gtk::manage(new Gtk::Table());
    handControlBox->pack_start(*table,false,false);

    // left grasp
    xCur = yCur = 0;
    std::vector<int> ids = { 0, 1, 2 };
    mLeftGraspNameEnum = 0;
    mLeftGraspState = 0;
    Gtk::Label* label1 = Gtk::manage(new Gtk::Label("Sandia Left"));
    Gtk::Label* label2 = Gtk::manage(new Gtk::Label("Closed % L"));
    spin = createSpin(mLeftGraspState, 0, 100, 10);
    combo = createCombo(mLeftGraspNameEnum, mGraspNames, ids);
    button = Gtk::manage(new Gtk::Button("Grasp"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onSandiaLeftGraspButton));
    table->attach(*label1, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    table->attach(*combo, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    table->attach(*label2, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    table->attach(*spin, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    table->attach(*button, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    ++yCur;

    // right grasp
    xCur = 0;
    mRightGraspNameEnum = 0;
    mRightGraspState = 0;
    label1 = Gtk::manage(new Gtk::Label("Sandia Right"));
    label2 = Gtk::manage(new Gtk::Label("Closed % R"));
    spin = createSpin(mRightGraspState, 0, 100, 10);
    combo = createCombo(mRightGraspNameEnum, mGraspNames, ids);
    button = Gtk::manage(new Gtk::Button("Grasp"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onSandiaRightGraspButton));
    table->attach(*label1, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    table->attach(*combo, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    table->attach(*label2, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    table->attach(*spin, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    table->attach(*button, xCur, xCur+1, yCur, yCur+1, xOpts, yOpts); ++xCur;
    ++yCur;

    /////////////////////////////////
    // irobot hands
    //

    handControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    label = Gtk::manage(new Gtk::Label("iRobot Hand", Gtk::ALIGN_CENTER));
    handControlBox->pack_start(*label, false, false);

    ids = { 0, 1 };
    std::vector<std::string> labels = { "iRobot Left", "iRobot Right" };
    hbox = Gtk::manage(new Gtk::HBox());
    label = Gtk::manage(new Gtk::Label("Controlling:"));
    mControlIrobotRightHand = 0;
    combo = createCombo(mControlIrobotRightHand, labels, ids);
    mIrobotFingerEnabled[0] = true;
    mIrobotFingerEnabled[1] = true;
    mIrobotFingerEnabled[2] = true;
    hbox->pack_start(*label,false,false);
    hbox->pack_start(*combo,false,false);
    box = Gtk::manage(new Gtk::VBox());
    addCheck("0 Finger", mIrobotFingerEnabled[0], box);
    addCheck("1 Finger", mIrobotFingerEnabled[1], box);
    hbox->pack_start(*box,false,false);
    box = Gtk::manage(new Gtk::VBox());
    addCheck("2 Thumb", mIrobotFingerEnabled[2], box);
    hbox->pack_start(*box,false,false);
    handControlBox->pack_start(*hbox, false, false);


    hbox = Gtk::manage(new Gtk::HBox());
    button = Gtk::manage(new Gtk::Button("Open"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onIrobotOpenButton));
    hbox->add(*button);
    button = Gtk::manage(new Gtk::Button("Close"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onIrobotCloseButton));
    hbox->add(*button);
    handControlBox->pack_start(*hbox, false, false);

    table = Gtk::manage(new Gtk::Table());
    handControlBox->pack_start(*table, false, false);
    xCur = yCur = 0;
    label = Gtk::manage(new Gtk::Label("Partial Closed %"));
    mIrobotClosePercent = 0;
    spin = createSpin(mIrobotClosePercent, 0, 100, 10);
    button = Gtk::manage(new Gtk::Button("Grasp"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onIrobotPartialGraspButton));
    table->attach(*label,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*spin,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*button,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    ++yCur;

    xCur = 0;
    label = Gtk::manage(new Gtk::Label("Spread [deg]"));
    mIrobotSpreadDegree = 0;
    spin = createSpin(mIrobotSpreadDegree, 0, 90, 10);
    button = Gtk::manage(new Gtk::Button("Spread"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onIrobotSpreadDegreeButton));
    table->attach(*label,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*spin,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*button,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    ++yCur;

    xCur = 0;
    ids = { 0, 1 };
    labels = { "No Jig", "Jig" };
    label = Gtk::manage(new Gtk::Label("Calibrate"));
    mIrobotCalibrate = 0;
    combo = createCombo(mIrobotCalibrate, labels, ids);
    button = Gtk::manage(new Gtk::Button("Send"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onIrobotCalibrateButton));
    table->attach(*label,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*combo,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*button,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    notebook->append_page(*handControlBox, "Hands");



    // Robotiq
    Gtk::VBox* robotiqControlBox = Gtk::manage(new Gtk::VBox());

    ids = { 0, 1 };
    labels = { "Robotiq Left", "Robotiq Right" };
    hbox = Gtk::manage(new Gtk::HBox());
    label = Gtk::manage(new Gtk::Label("Controlling:"));
    mControlRobotiqRightHand = 0;
    combo = createCombo(mControlRobotiqRightHand, labels, ids);
    mRoboticFingerEnabled[0] = true;
    mRoboticFingerEnabled[1] = true;
    mRoboticFingerEnabled[2] = true;
    hbox->pack_start(*label,false,false);
    hbox->pack_start(*combo,false,false);
    box = Gtk::manage(new Gtk::VBox());
    addCheck("0 Finger ", mRoboticFingerEnabled[0], box);
    addCheck("1 Finger ", mRoboticFingerEnabled[1], box);
    hbox->pack_start(*box,false,false);
    box = Gtk::manage(new Gtk::VBox());
    addCheck("2 Thumb ", mRoboticFingerEnabled[2], box);
    hbox->pack_start(*box,false,false);
    robotiqControlBox->pack_start(*hbox, false, false);


    hbox = Gtk::manage(new Gtk::HBox());
    button = Gtk::manage(new Gtk::Button("Open"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onRobotiqOpenButton));
    hbox->add(*button);
    button = Gtk::manage(new Gtk::Button("Close"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onRobotiqCloseButton));
    hbox->add(*button);
    robotiqControlBox->pack_start(*hbox, false, false);

    table = Gtk::manage(new Gtk::Table());
    robotiqControlBox->pack_start(*table, false, false);
    xCur = yCur = 0;
    label = Gtk::manage(new Gtk::Label("Partial Closed %"));
    mRobotiqClosePercent = 0;
    spin = createSpin(mRobotiqClosePercent, 0, 100, 10);
    table->attach(*label,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*spin,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    ++yCur;

    xCur = 0;
    label = Gtk::manage(new Gtk::Label("Force %"));
    mRobotiqForce = 100;
    spin = createSpin(mRobotiqForce, 0, 100, 10);
    table->attach(*label,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*spin,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    ++yCur;

    xCur = 0;
    label = Gtk::manage(new Gtk::Label("Velocity %"));
    mRobotiqVelocity = 100;
    spin = createSpin(mRobotiqVelocity, 0, 100, 10);
    table->attach(*label,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*spin,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    ++yCur;

    xCur = 0;
    ids = { 0, 1 , 2 };
    labels = { "Basic", "Pinch", "Wide" };
    label = Gtk::manage(new Gtk::Label("Mode"));
    mRobotiqMode = 0;
    combo = createCombo(mRobotiqMode, labels, ids);
    table->attach(*label,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;
    table->attach(*combo,xCur,xCur+1,yCur,yCur+1,xOpts,yOpts); ++xCur;




    hbox = Gtk::manage(new Gtk::HBox());
    button = Gtk::manage(new Gtk::Button("Grasp"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onRobotiqPartialGraspButton));
    hbox->add(*button);
    robotiqControlBox->pack_start(*hbox, false, false);


    hbox = Gtk::manage(new Gtk::HBox());
    button = Gtk::manage(new Gtk::Button("Reset/Calibrate"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onRobotiqResetCalibrateButton));
    hbox->add(*button);
    robotiqControlBox->pack_start(*hbox, false, false);

    notebook->append_page(*robotiqControlBox, "Robotiq");



    ///////////////////////////////////////////////////////////
    Gtk::Box* sensorControlBox = Gtk::manage(new Gtk::VBox());
    Gtk::Table* sensorControlTable = Gtk::manage(new Gtk::Table(5,3,false));

    // maxing out at 5hz for safety
    mHandCameraFrameRate = -1;
    //addSpin("Hands Cam fps", mHandCameraFrameRate, -1, 10, 1, handControlBox);
    yCur = 0;
    mLeftGraspNameEnum = 0;

    label = Gtk::manage(new Gtk::Label("Head Cam fps", Gtk::ALIGN_RIGHT));
    mDummyIntValue = 5;
    spin = gtkmm::RendererBase::createSpin(mDummyIntValue, 0, 30, 1);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,spin]{this->publishMultisense(-1000,spin->get_value(),-1);});
    sensorControlTable->attach(*label, 0, 1, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*spin, 1, 2, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*button, 2, 3, yCur, yCur+1, xOpts, yOpts);
    // Artificial limit added here - to limit LCM traffic
    ++yCur;

    label = Gtk::manage(new Gtk::Label("Head Cam Gain", Gtk::ALIGN_RIGHT));
    mDummyDoubleValue = 1.0;
    check = Gtk::manage(new Gtk::CheckButton());
    check->set_active(true);
    spin = gtkmm::RendererBase::createSpin(mDummyDoubleValue, 1.0, 8.0, 0.1);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,spin,check]{
        this->publishMultisense(-1000,-1,spin->get_value(),false);
        check->set_active(false);
      });
    sensorControlTable->attach(*label, 0, 1, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*spin, 1, 2, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*button, 2, 3, yCur, yCur+1, xOpts, yOpts);
    ++yCur;

    label = Gtk::manage(new Gtk::Label("Head Auto Gain", Gtk::ALIGN_RIGHT));
    mDummyIntValue = 45;
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,check]{this->publishMultisense(-1000,-1,-1,check->get_active());});
    sensorControlTable->attach(*label, 0, 1, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*check, 1, 2, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*button, 2, 3, yCur, yCur+1, xOpts, yOpts);
    ++yCur;

    // DRCSIM max: 60rpm | Real Sensor: 49rpm | Temporary Safety: 25
    label = Gtk::manage(new Gtk::Label("Spin Rate (rpm)", Gtk::ALIGN_RIGHT));
    mDummyIntValue = 5;
    spin = gtkmm::RendererBase::createSpin(mDummyIntValue, -25, 25, 1);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,spin]{this->publishMultisense(spin->get_value(),-1,-1);});
    sensorControlTable->attach(*label, 0, 1, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*spin, 1, 2, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*button, 2, 3, yCur, yCur+1, xOpts, yOpts);
    ++yCur;

    label = Gtk::manage(new Gtk::Label("LED on", Gtk::ALIGN_RIGHT));
    check = Gtk::manage(new Gtk::CheckButton());
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,check]{this->publishMultisense(-1000,-1,-1,-1,check->get_active());});
    sensorControlTable->attach(*label, 0, 1, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*check, 1, 2, yCur, yCur+1, xOpts, yOpts);
    sensorControlTable->attach(*button, 2, 3, yCur, yCur+1, xOpts, yOpts);
    ++yCur;

    
    sensorControlBox->pack_start(*sensorControlTable,false,false);

    /*
    addSpin("Pitch (deg)", mHeadPitchAngle, -90, 90, 5, sensorControlBox);
    button = Gtk::manage(new Gtk::Button("Submit Head Pitch"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onHeadPitchControlButton));
    sensorControlBox->pack_start(*button, false, false);
    */
    notebook->append_page(*sensorControlBox, "Sensors");

    container->add(*notebook);
    container->show_all();

    mParamManager->onParamChange();
  }

  void addWorkspaceViewWidgets() {
    Gtk::Box* mainBox = Gtk::manage(new Gtk::VBox());
    Gtk::Box* vbox = Gtk::manage(new Gtk::VBox());
    Gtk::Box* hbox = Gtk::manage(new Gtk::HBox());
    mWorkspaceDepthFov = 90;
    Gtk::HScale* fovSlider = createSlider(mWorkspaceDepthFov,60,130,10);
    mWorkspaceDepthYaw = 0;
    Gtk::HScale* yawSlider = createSlider(mWorkspaceDepthYaw,-90,100,10);
    Gtk::Label* label = Gtk::manage(new Gtk::Label("yaw"));
    vbox->pack_start(*label,false,false);
    vbox->pack_start(*yawSlider,false,false);
    hbox->pack_start(*vbox,true,true);
    vbox = Gtk::manage(new Gtk::VBox());
    label = Gtk::manage(new Gtk::Label("fov"));
    vbox->pack_start(*label,false,false);
    vbox->pack_start(*fovSlider,false,false);
    hbox->pack_start(*vbox,true,true);
    mainBox->pack_start(*hbox,false,false);
    mRequestControlBox->add(*mainBox);
    mParamManager->bind("Workspace_Depth.fov", *fovSlider);
    mParamManager->bind("Workspace_Depth.yaw", *yawSlider);
  }

  void addControl(const int iId, const std::string& iLabel,
                  const std::string& iChannel, const ChannelType iChannelType,
                  const bool iQuality=false, Gtk::Box* iContainer=NULL) {
    RequestControl::Ptr group(new RequestControl());
    group->mId = iId;
    group->mPeriod = 0;
    group->mQuality = 0;
    group->mEnabled = false;

    Gtk::CheckButton* check = Gtk::manage(new Gtk::CheckButton());
    Gtk::Label* label = Gtk::manage(new Gtk::Label(iLabel,Gtk::ALIGN_LEFT));
    Gtk::SpinButton* spin = createSpin(group->mPeriod,1,20,1);
    spin->set_digits(0);
    Gtk::Label* ageLabel = Gtk::manage(new Gtk::Label(" "));
    Gtk::ComboBox* combo = NULL;
    Gtk::Button* button = Gtk::manage(new Gtk::Button("now"));
    button->signal_clicked().connect([this,group]{sendOneShotRequest(group);});

    // callbacks
    check->signal_toggled().connect
      ([check,spin,group]{
        group->mEnabled = check->get_active();
        group->mPeriod = group->mEnabled ? spin->get_value() : -1;
      });
    spin->signal_value_changed().connect
      ([check,spin,group]{
        group->mPeriod = check->get_active() ? spin->get_value() : -1;
      });
    group->mEnabled = false;
    group->mPeriod = -1;
    
    std::string safeLabel = iLabel;
    std::replace(safeLabel.begin(), safeLabel.end(), ' ', '_');
    std::replace(safeLabel.begin(), safeLabel.end(), '.', '_');
    mParamManager->bind(safeLabel+".enable",*check);
    mParamManager->bind(safeLabel+".period",*spin);
    if (iQuality) {
      std::vector<std::string> labels = { "-", "Low", "Med", "High" };
      std::vector<int> ids = { -1, drc::camera_settings_t::QUALITY_LOW,
                               drc::camera_settings_t::QUALITY_MED,
                               drc::camera_settings_t::QUALITY_HIGH };
      combo = createCombo(group->mQuality,labels,ids);
      mParamManager->bind(safeLabel+".quality",*combo);
    }
    if (iContainer == NULL) {
      Gtk::AttachOptions xOpts = Gtk::FILL;
      Gtk::AttachOptions yOpts = Gtk::SHRINK;
      unsigned int yCur, cols;
      mRequestControlTable->get_size(yCur,cols);
      mRequestControlTable->attach(*button,0,1,yCur,yCur+1,xOpts,yOpts);
      mRequestControlTable->attach(*check,1,2,yCur,yCur+1,xOpts,yOpts);
      mRequestControlTable->attach(*label,2,3,yCur,yCur+1,xOpts,yOpts);
      mRequestControlTable->attach(*ageLabel,3,4,yCur,yCur+1,xOpts,yOpts);
      mRequestControlTable->attach(*spin,4,5,yCur,yCur+1,xOpts,yOpts);
      if (combo != NULL) {
        mRequestControlTable->attach(*combo,5,6,yCur,yCur+1,xOpts,yOpts);
      }
    }
    else {
      Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
      box->add(*check);
      box->add(*label);
      box->add(*ageLabel);
      box->add(*spin);
      if (combo != NULL) box->add(*combo);
      iContainer->pack_start(*box,false,false);
    }
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

  drc::data_request_t getDataRequest(const RequestControl::Ptr& iGroup) {
    drc::data_request_t req;
    req.type = iGroup->mId;
    req.period = iGroup->mPeriod*10;
    if (req.type == drc::data_request_t::DEPTH_MAP_WORKSPACE) {
      sendWorkspaceDepthRequest(iGroup->mPeriod);
    }
    if (iGroup->mQuality > 0) {
      drc::camera_settings_t settingsMessage;
      settingsMessage.data_request.type = req.type;
      settingsMessage.data_request.period = 0;
      settingsMessage.quality = iGroup->mQuality;
      getLcm()->publish("CAMERA_SETTINGS", &settingsMessage);
    }
    return req;
  }

  void sendAllDataRequests() {
    drc::data_request_list_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    for (auto iter : mRequestControls) {
      auto req = getDataRequest(iter.second);
      msg.requests.push_back(req);
    }
    msg.num_requests = msg.requests.size();
    if (msg.num_requests > 0) {
      getLcm()->publish("DATA_REQUEST", &msg);
    }
    mParamManager->pushValues();
  }

  void sendOneShotRequest(const RequestControl::Ptr& iGroup) {
    drc::data_request_list_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    RequestControl::Ptr tempGroup(new RequestControl(*iGroup));
    tempGroup->mPeriod = 0;
    auto req = getDataRequest(tempGroup);
    msg.requests.push_back(req);
    msg.num_requests = msg.requests.size();
    if (msg.num_requests > 0) {
      getLcm()->publish("DATA_REQUEST", &msg);
    }
    mParamManager->pushValues();
  }

  void sendWorkspaceDepthRequest(const int iPeriod) {
    drc::map_request_t msg;

    // basic message field setup
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.map_id = 3;
    msg.view_id = drc::data_request_t::DEPTH_MAP_WORKSPACE;
    msg.type = drc::map_request_t::DEPTH_IMAGE;
    msg.resolution = 0.01;
    msg.frequency = (iPeriod>0) ? 1.0f/iPeriod : (iPeriod==0 ? 0 : -1);
    msg.quantization_max = 0.02;
    msg.width = 200;
    msg.height = 200;
    msg.time_min = -190;
    msg.time_max = 0;
    msg.time_mode = drc::map_request_t::ROLL_ANGLE_RELATIVE;
    msg.relative_location = true;
    msg.active = true;

    // bounding planes
    msg.clip_planes.push_back(std::vector<float>({ 1, 0, 0, 1}));
    msg.clip_planes.push_back(std::vector<float>({-1, 0, 0, 2}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 1, 0, 2}));
    msg.clip_planes.push_back(std::vector<float>({ 0,-1, 0, 2}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 0, 1, 2}));
    msg.clip_planes.push_back(std::vector<float>({ 0, 0,-1, 1}));
    msg.num_clip_planes = msg.clip_planes.size();

    // projector
    const float kPi = acos(-1);
    const float kDegToRad = kPi/180;
    const float hFov = mWorkspaceDepthFov*kDegToRad;
    const float vFovDeg = 110;
    const float vFov = vFovDeg*kDegToRad;
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() << 0,0,1, -1,0,0, 0,-1,0;
    Eigen::Projective3f calib = Eigen::Projective3f::Identity();
    calib.matrix().row(2).swap(calib.matrix().row(3));
    calib(0,0) = msg.width/2/tan(hFov/2);
    calib(1,1) = msg.height/2/tan(vFov/2);
    calib(0,2) = msg.width/2;
    calib(1,2) = msg.height/2;

    // adjustment for yaw and pitch
    Eigen::AngleAxisf angleAxis;
    angleAxis = Eigen::AngleAxisf((10 + (90-vFovDeg/2))*kDegToRad,
                                  Eigen::Vector3f(0,1,0));
    pose = angleAxis*pose;
    angleAxis = Eigen::AngleAxisf(-mWorkspaceDepthYaw*kDegToRad,
                                  Eigen::Vector3f(0,0,1));
    pose = angleAxis*pose;
    Eigen::Projective3f projector = calib*pose.inverse();
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) msg.transform[i][j] = projector(i,j);
    }
    getLcm()->publish("MAP_REQUEST",&msg);
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

  void onHeadPitchControlButton(const double iHeadPitchAngle ) {
    const double kPi = 4*atan(1);
    double degreesToRadians = kPi/180;
    drc::neck_pitch_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.pitch = iHeadPitchAngle*degreesToRadians;
    getLcm()->publish("DESIRED_NECK_PITCH", &msg);
  }

  /* TODO: no longer needed
  void onControllerHeightMapMode() {
    sendHeightMode();
  }
  */

  void onSandiaLeftGraspButton() {
    drc::sandia_simple_grasp_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.name = mGraspNames[ mLeftGraspNameEnum ];
    msg.closed_amount = (float) mLeftGraspState/100;
    getLcm()->publish("SANDIA_LEFT_SIMPLE_GRASP", &msg);
  }

  void onSandiaRightGraspButton() {
    drc::sandia_simple_grasp_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.name = mGraspNames[ mRightGraspNameEnum ];
    msg.closed_amount = (float) mRightGraspState/100;
    getLcm()->publish("SANDIA_RIGHT_SIMPLE_GRASP", &msg);
  }

  void onIrobotPartialGraspButton() {
    irobothand::position_control_close_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.close_fraction = (float) mIrobotClosePercent/100;
    int8_t valid[] = {mIrobotFingerEnabled[0], mIrobotFingerEnabled[1], mIrobotFingerEnabled[2], false};
    memcpy(msg.valid,valid, 4*sizeof(int8_t));
    if (mControlIrobotRightHand)
      getLcm()->publish("IROBOT_RIGHT_POSITION_CONTROL_CLOSE", &msg);
    else
      getLcm()->publish("IROBOT_LEFT_POSITION_CONTROL_CLOSE", &msg);
  }

  void onIrobotSpreadDegreeButton() {
    irobothand::spread_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.angle_radians = (float) mIrobotSpreadDegree*M_PI/180 ;
    if (mControlIrobotRightHand)
      getLcm()->publish("IROBOT_RIGHT_SPREAD", &msg);
    else
      getLcm()->publish("IROBOT_LEFT_SPREAD", &msg);
  }

  void onIrobotOpenButton() {
    irobothand::position_control_close_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.close_fraction = 0.0;
    int8_t valid[] = {mIrobotFingerEnabled[0], mIrobotFingerEnabled[1], mIrobotFingerEnabled[2], false};
    memcpy(msg.valid,valid, 4*sizeof(int8_t));
    if (mControlIrobotRightHand)
      getLcm()->publish("IROBOT_RIGHT_POSITION_CONTROL_CLOSE", &msg);
    else
      getLcm()->publish("IROBOT_LEFT_POSITION_CONTROL_CLOSE", &msg);
  }

  void onIrobotCloseButton() {
    irobothand::current_control_close_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.current_milliamps = 800.0;
    int8_t valid[] = {mIrobotFingerEnabled[0], mIrobotFingerEnabled[1], mIrobotFingerEnabled[2], false};
    memcpy(msg.valid,valid, 4*sizeof(int8_t));
    if (mControlIrobotRightHand)
      getLcm()->publish("IROBOT_RIGHT_CURRENT_CONTROL_CLOSE", &msg);
    else
      getLcm()->publish("IROBOT_LEFT_CURRENT_CONTROL_CLOSE", &msg);
  }

  void onIrobotCalibrateButton() {
    irobothand::calibrate_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.in_jig = mIrobotCalibrate != 0;
    std::string channel = (mControlIrobotRightHand==0) ?
      "IROBOT_LEFT_CALIBRATE" : "IROBOT_RIGHT_CALIBRATE";
    getLcm()->publish(channel , &msg);
  }


  //
  void onRobotiqPartialGraspButton() {
    robotiqhand::command_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.activate = 1;
    msg.do_move = 1;
    msg.mode = mRobotiqMode;
    msg.position = (int) mRobotiqClosePercent*254/100;
    msg.force = (int) mRobotiqForce*254/100 + 1;  //send 1 to 255, 0 is bad
    msg.velocity = (int) mRobotiqVelocity*254/100;
    if (mControlRobotiqRightHand)
      getLcm()->publish("ROBOTIQ_RIGHT_COMMAND", &msg);
    else
      getLcm()->publish("ROBOTIQ_LEFT_COMMAND", &msg);
  }

  void onRobotiqResetCalibrateButton() {
    robotiqhand::command_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.activate = 0;
    msg.do_move = 0;

    if (mControlRobotiqRightHand)
    {
      getLcm()->publish("ROBOTIQ_RIGHT_COMMAND", &msg);
      msg.activate = 1;
      getLcm()->publish("ROBOTIQ_RIGHT_COMMAND", &msg);
    }
    else
    {
      getLcm()->publish("ROBOTIQ_LEFT_COMMAND", &msg);
      msg.activate = 1;
      getLcm()->publish("ROBOTIQ_LEFT_COMMAND", &msg);
    }
  }

  void onRobotiqOpenButton() {
    robotiqhand::command_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.activate = 1;
    msg.do_move = 1;
    msg.mode = mRobotiqMode;
    msg.position = (int) 0*254/100;
    msg.force = (int) mRobotiqForce*254/100 + 1;
    msg.velocity = (int) mRobotiqVelocity*254/100;
    if (mControlRobotiqRightHand)
      getLcm()->publish("ROBOTIQ_RIGHT_COMMAND", &msg);
    else
      getLcm()->publish("ROBOTIQ_LEFT_COMMAND", &msg);
  }

  void onRobotiqCloseButton() {
    robotiqhand::command_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.activate = 1;
    msg.do_move = 1;
    msg.mode = mRobotiqMode;
    msg.position = (int) 254;//*254/100;
    msg.force = (int) mRobotiqForce*254/100+1;
    msg.velocity = (int) mRobotiqVelocity*254/100;
    if (mControlRobotiqRightHand){
      getLcm()->publish("ROBOTIQ_RIGHT_COMMAND", &msg);
    }else{
      getLcm()->publish("ROBOTIQ_LEFT_COMMAND", &msg);
    }
  }

  void publishMultisense(const double iSpinRate=-1000,
                         const double iFrameRate=-1,
                         const double iGain=-1, const int iAgc=-1,
                         const bool iLight=false) {
    multisense::command_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.rpm = iSpinRate;
    msg.fps = iFrameRate;
    msg.gain = iGain;
    msg.agc = iAgc;
    msg.leds_flash = false;
    msg.leds_duty_cycle = iLight ? 100 : 0;
    getLcm()->publish("MULTISENSE_COMMAND", &msg);
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
