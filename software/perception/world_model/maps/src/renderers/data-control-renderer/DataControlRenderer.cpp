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

#include <lcmtypes/multisense.hpp>


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
  int mDummyIntValue;
  double mDummyDoubleValue;

  Gtk::VBox* mRequestControlBox;
  Gtk::VBox* mAffControlBox;
  std::unordered_map<int, RequestControl::Ptr> mRequestControls;
  int mHandCameraFrameRate;
  int mCameraCompression;
  
  int mLeftGraspState;
  int mLeftGraspNameEnum;
  int mRightGraspState;
  int mRightGraspNameEnum;
  std::vector <std::string>  mGraspNames;
  
  bool mMinimalAffordances;
  int mControllerHeightMapMode;

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
      
    mGraspNames = { "cylindrical", "spherical", "prismatic" };

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

  /* TODO: no longer needed
  bool sendHeightMode() {
    drc::map_controller_command_t msg;
    msg.command = mControllerHeightMapMode;
    getLcm()->publish("MAP_CONTROLLER_COMMAND", &msg);
    return true;
  }
  */

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
               "CAMERA_LEFT", ChannelTypeAnonymous);
    addControl(drc::data_request_t::CAMERA_IMAGE_LHAND, "Camera L.Hand",
               "CAMERA_LHANDLEFT", ChannelTypeAnonymous);
    addControl(drc::data_request_t::CAMERA_IMAGE_RHAND, "Camera R.Hand",
               "CAMERA_RHANDLEFT", ChannelTypeAnonymous);
    addControl(drc::data_request_t::CAMERA_IMAGE_LCHEST, "Camera L.Chest",
               "CAMERA_LCHEST", ChannelTypeAnonymous);
    addControl(drc::data_request_t::CAMERA_IMAGE_RCHEST, "Camera R.Chest",
               "CAMERA_RCHEST", ChannelTypeAnonymous);
    mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    
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
    /*
    addControl(drc::data_request_t::DEPTH_MAP_SCENE, "Scene Depth",
               "MAP_DEPTH", ChannelTypeDepthImage);
    addControl(drc::data_request_t::OCTREE_SCENE, "Scene Points",
               "MAP_OCTREE", ChannelTypeAnonymous);
    */
    addControl(drc::data_request_t::DEPTH_MAP_WORKSPACE, "Workspace Depth",
               "MAP_DEPTH", ChannelTypeDepthImage);

    //mRequestControlBox->add(*Gtk::manage(new Gtk::HSeparator()));
    //addControl(drc::data_request_t::TERRAIN_COST, "Terrain Cost",
    //           "TERRAIN_DIST_MAP", ChannelTypeAnonymous);
    //addControl(drc::data_request_t::HEIGHT_MAP_DENSE, "*DENSE HEIGHT!!*",
    //           "MAP_DEPTH", ChannelTypeDepthImage);

    addControl(drc::data_request_t::DENSE_CLOUD_LHAND, "Dense Box L.Hand",
               "MAP_CLOUD", ChannelTypeDepthImage);
    addControl(drc::data_request_t::DENSE_CLOUD_RHAND, "Dense Box R.Hand",
               "MAP_CLOUD", ChannelTypeDepthImage);

    Gtk::Button* button = Gtk::manage(new Gtk::Button("Submit Request"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onDataRequestButton));
    mRequestControlBox->add(*button);
    notebook->append_page(*mRequestControlBox, "Pull");

    // for pushing data (e.g., affordances)
    mAffControlBox = Gtk::manage(new Gtk::VBox());
    Gtk::VBox* vbox = Gtk::manage(new Gtk::VBox());
    Gtk::Label* label =
      Gtk::manage(new Gtk::Label("Aff", Gtk::ALIGN_LEFT));
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
    addControl(drc::data_request_t::AFFORDANCE_LIST, "Affordance List",
               "AFFORDANCE_LIST", ChannelTypeAnonymous, mAffControlBox);
    button = Gtk::manage(new Gtk::Button("Pull"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onDataRequestButton));
    mAffControlBox->pack_start(*button,false,false);
    notebook->append_page(*mAffControlBox, "Affordances");

    // for sensor control
    Gtk::VBox* handControlBox = Gtk::manage(new Gtk::VBox());


    
    
    /* TODO: no longer needed
    mControllerHeightMapMode = drc::map_controller_command_t::FLAT_GROUND;
    labels = {"Flat Ground", "Full Heights", "Z Normals"};
    ids = {drc::map_controller_command_t::FLAT_GROUND,
           drc::map_controller_command_t::FULL_HEIGHTMAP,
           drc::map_controller_command_t::Z_NORMALS};
    addCombo("Controller Height Mode", mControllerHeightMapMode,
             labels, ids, handControlBox);
    button = Gtk::manage(new Gtk::Button("Submit Height Mode"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onControllerHeightMapMode));
    handControlBox->pack_start(*button, false, false);
    */
    
    //
    // grasp
    // left
    std::vector<int> ids = { 0, 1, 2 };
    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());
    Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
    mLeftGraspNameEnum = 0;
    addCombo("Left", mLeftGraspNameEnum, mGraspNames, ids, box);
    hbox->add(*box); 
    // Closed Amount
    box = Gtk::manage(new Gtk::HBox());
    mLeftGraspState =0;
    addSpin("Closed % L", mLeftGraspState, 0, 100, 10, box); 
    hbox->add(*box);
    // send button
    button = Gtk::manage(new Gtk::Button("Grasp"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onLeftGraspButton));
    hbox->add(*button);
    handControlBox->pack_start(*hbox, false, false);
    
    // right
    ids = { 0, 1, 2 };
    Gtk::HBox* hboxR = Gtk::manage(new Gtk::HBox());
    Gtk::HBox* boxR = Gtk::manage(new Gtk::HBox());
    mRightGraspNameEnum = 0;
    addCombo("Right", mRightGraspNameEnum, mGraspNames, ids, boxR);
    hboxR->add(*boxR); 
    // Closed Amount
    boxR = Gtk::manage(new Gtk::HBox());
    mRightGraspState = 0;
    addSpin("Closed % R", mRightGraspState, 0, 100, 10, boxR); 
    hboxR->add(*boxR);
    // send button
    button = Gtk::manage(new Gtk::Button("Grasp"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onRightGraspButton));
    hboxR->add(*button);
    handControlBox->pack_start(*hboxR, false, false);    
    notebook->append_page(*handControlBox, "Hands");    
    
    
    ///////////////////////////////////////////////////////////
    Gtk::Table* sensorControlTable = Gtk::manage(new Gtk::Table(5,3,false));
    Gtk::AttachOptions xOptions = Gtk::FILL | Gtk::EXPAND;
    Gtk::AttachOptions yOptions = Gtk::SHRINK;
    
    // maxing out at 5hz for safety
    mHandCameraFrameRate = -1;
    //addSpin("Hands Cam fps", mHandCameraFrameRate, -1, 10, 1, handControlBox); 
    mCameraCompression = 0;
    std::vector<std::string> labels = { "-", "Low", "Med", "High" };
    ids =
      { -1, drc::sensor_request_t::QUALITY_LOW,
        drc::sensor_request_t::QUALITY_MED,
        drc::sensor_request_t::QUALITY_HIGH};
    mLeftGraspNameEnum = 0;
    label = Gtk::manage(new Gtk::Label("Camera Quality", Gtk::ALIGN_RIGHT));
    Gtk::ComboBox* combo =
      gtkmm::RendererBase::createCombo(mCameraCompression, labels, ids);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onSendRatesControlButton));
    sensorControlTable->attach(*label, 0, 1, 0, 1, xOptions, yOptions);
    sensorControlTable->attach(*combo, 1, 2, 0, 1, xOptions, yOptions);
    sensorControlTable->attach(*button, 2, 3, 0, 1, xOptions, yOptions);
    
    label = Gtk::manage(new Gtk::Label("Head Cam fps", Gtk::ALIGN_RIGHT));
    mDummyIntValue = 5;
    Gtk::SpinButton* spin = gtkmm::RendererBase::createSpin(mDummyIntValue, 0, 10, 1);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,spin]{this->publishMultisense(-1000,spin->get_value(),-1);});
    sensorControlTable->attach(*label, 0, 1, 1, 2, xOptions, yOptions);
    sensorControlTable->attach(*spin, 1, 2, 1, 2, xOptions, yOptions);
    sensorControlTable->attach(*button, 2, 3, 1, 2, xOptions, yOptions);
    // Artificial limit added here - to limit LCM traffic
    
    label = Gtk::manage(new Gtk::Label("Head Cam Gain", Gtk::ALIGN_RIGHT));
    mDummyDoubleValue = 1.0;
    spin = gtkmm::RendererBase::createSpin(mDummyDoubleValue, 1.0, 8.0, 0.1);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,spin]{this->publishMultisense(-1000,-1,spin->get_value());});
    sensorControlTable->attach(*label, 0, 1, 2, 3, xOptions, yOptions);
    sensorControlTable->attach(*spin, 1, 2, 2, 3, xOptions, yOptions);
    sensorControlTable->attach(*button, 2, 3, 2, 3, xOptions, yOptions);
    
    // DRCSIM max: 60rpm | Real Sensor: 49rpm | Temporary Safety: 25
    label = Gtk::manage(new Gtk::Label("Spin Rate (rpm)", Gtk::ALIGN_RIGHT));
    mDummyIntValue = 5;
    spin = gtkmm::RendererBase::createSpin(mDummyIntValue, -25, 25, 1);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,spin]{this->publishMultisense(spin->get_value(),-1,-1);});
    sensorControlTable->attach(*label, 0, 1, 3, 4, xOptions, yOptions);
    sensorControlTable->attach(*spin, 1, 2, 3, 4, xOptions, yOptions);
    sensorControlTable->attach(*button, 2, 3, 3, 4, xOptions, yOptions);
    
    label = Gtk::manage(new Gtk::Label("Pitch (deg)", Gtk::ALIGN_RIGHT));
    mDummyIntValue = 45;
    spin = gtkmm::RendererBase::createSpin(mDummyIntValue, -90, 90, 5);
    button = Gtk::manage(new Gtk::Button("send"));
    button->signal_clicked().connect
      ([this,spin]{this->onHeadPitchControlButton(spin->get_value());});
    sensorControlTable->attach(*label, 0, 1, 4, 5, xOptions, yOptions);
    sensorControlTable->attach(*spin, 1, 2, 4, 5, xOptions, yOptions);
    sensorControlTable->attach(*button, 2, 3, 4, 5, xOptions, yOptions);
    
    
    /*
    addSpin("Pitch (deg)", mHeadPitchAngle, -90, 90, 5, sensorControlBox);
    button = Gtk::manage(new Gtk::Button("Submit Head Pitch"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &DataControlRenderer::onHeadPitchControlButton));
    sensorControlBox->pack_start(*button, false, false);
    */
    notebook->append_page(*sensorControlTable, "Sensors");

    container->add(*notebook);
    container->show_all();
  }

  void addControl(const int iId, const std::string& iLabel,
                  const std::string& iChannel, const ChannelType iChannelType,
                  Gtk::Box* iContainer=NULL) {
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
    if (iContainer == NULL)  mRequestControlBox->pack_start(*box,false,false);
    else iContainer->pack_start(*box,false,false);
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
    msg.spindle_rpm = -1;
    msg.head_fps = -1;
    msg.hand_fps = mHandCameraFrameRate;
    msg.camera_compression = mCameraCompression;
    getLcm()->publish("SENSOR_REQUEST", &msg);
    // TODO: set all to -1
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

  void onLeftGraspButton() {
    drc::sandia_simple_grasp_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.name = mGraspNames[ mLeftGraspNameEnum ];
    msg.closed_amount = (float) mLeftGraspState/100;
    getLcm()->publish("IROBOT_LEFT_SIMPLE_GRASP", &msg);
    getLcm()->publish("SANDIA_LEFT_SIMPLE_GRASP", &msg);
  }
  
  void onRightGraspButton() {
    drc::sandia_simple_grasp_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.name = mGraspNames[ mRightGraspNameEnum ];
    msg.closed_amount = (float) mRightGraspState/100;
    getLcm()->publish("IROBOT_RIGHT_SIMPLE_GRASP", &msg);
    getLcm()->publish("SANDIA_RIGHT_SIMPLE_GRASP", &msg);    
  }  

  void publishMultisense(const double iSpinRate, const double iFrameRate,
                         const double iGain) {
    multisense::command_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.rpm = iSpinRate;
    msg.fps = iFrameRate;
    msg.gain = iGain;
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
