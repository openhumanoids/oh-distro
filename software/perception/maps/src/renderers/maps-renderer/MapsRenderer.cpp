#include "MapsRenderer.hpp"

#include "MeshRenderer.hpp"

#include <bot_vis/viewer.h>

#include <drc_utils/Clock.hpp>

using namespace maps;

MapsRenderer::
MapsRenderer(BotViewer* iViewer, const int iPriority,
             const lcm_t* iLcm,
             const BotParam* iParam, const BotFrames* iFrames)
  : gtkmm::RendererBase("Maps", iViewer, iPriority, iLcm, iParam, iFrames) {

  // set up robot time clock
  drc::Clock::instance()->setLcm(getLcm());
  drc::Clock::instance()->setVerbose(false);

  // create and show ui widgets
  setupWidgets();

  // set up internal variables
  mBotWrapper.reset(new BotWrapper(getLcm(), getBotParam(), getBotFrames()));
  mMeshRenderer.reset(new MeshRenderer());
  mMeshRenderer->setBotObjects(getLcm(), getBotParam(), getBotFrames());
  mMeshRenderer->addCameraChannel("CAMERACHEST_LEFT");
  mMeshRenderer->addCameraChannel("CAMERACHEST_RIGHT");
  mMeshRenderer->addCameraChannel("CAMERA",true);
  //mMeshRenderer->setActiveCameraChannel("CAMERA_LEFT");
  mViewClient.setBotWrapper(mBotWrapper);
  mViewClient.addListener(this);
  mViewClient.addViewChannel("MAP_CONTROL_HEIGHT");
  mViewClient.addViewChannel("MAP_DEBUG");

  // start listening for view data
  mViewClient.start();
}

MapsRenderer::
~MapsRenderer() {
}


void MapsRenderer::
onAdjustBoxToggleChanged() {
  getBotEventHandler()->picking = mAdjustRequestBoxToggle->get_active();
}

void MapsRenderer::
setupWidgets() {
  Gtk::Container* container = getGtkContainer();
  Gtk::Notebook* notebook = Gtk::manage(new Gtk::Notebook());
  container->add(*notebook);
  std::vector<int> ids;
  std::vector<std::string> labels;

  // view list box (will fill in dynamically)
  {
    mViewListBox = Gtk::manage(new Gtk::VBox());
    Gtk::Button* clearButton = Gtk::manage(new Gtk::Button("Clear All"));
    clearButton->signal_clicked().connect
      (sigc::mem_fun(*this, &MapsRenderer::onClearViewsButton));
    mViewListBox->pack_start(*clearButton, false, false);
    notebook->append_page(*mViewListBox, "Views");
  }

  // simple request tab
  {
    Gtk::VBox* requestBox = Gtk::manage(new Gtk::VBox());
    notebook->append_page(*requestBox, "Request");

    Gdk::Color color;
    color.set_rgb_p(0.8,1.0,0.8);
    Gtk::HBox* box = Gtk::manage(new Gtk::HBox());
    Gtk::Button* button = Gtk::manage(new Gtk::Button("Create Box"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &MapsRenderer::onCreateBoxButton));
    box->pack_start(*button, false, false);
    mAdjustRequestBoxToggle = Gtk::manage(new Gtk::ToggleButton());
    mAdjustRequestBoxToggle->set_label("Adjust Box");
    mAdjustRequestBoxToggle->modify_bg(Gtk::STATE_ACTIVE, color);
    mAdjustRequestBoxToggle->modify_bg(Gtk::STATE_PRELIGHT, color);
    mAdjustRequestBoxToggle->signal_toggled().connect
      (sigc::mem_fun(*this, &MapsRenderer::onAdjustBoxToggleChanged));
    box->pack_start(*mAdjustRequestBoxToggle, false, false);
    mShowRequestBoxToggle = Gtk::manage(new Gtk::ToggleButton());
    mShowRequestBoxToggle->set_label("Show Box");
    mShowRequestBoxToggle->modify_bg(Gtk::STATE_ACTIVE, color);
    mShowRequestBoxToggle->modify_bg(Gtk::STATE_PRELIGHT, color);
    mShowRequestBox = false;
    bind(mShowRequestBoxToggle, "Show Request Box", mShowRequestBox);
    box->pack_start(*mShowRequestBoxToggle, false, false);
    requestBox->pack_start(*box, false, false);

    mRequestTimeWindow = 0;
    labels = { "Laser", "Stereo Head", "Stereo L.Hand", "Stereo R.Hand" };
    //ids = { drc::map_request_bbox_t::LASER,
    //        drc::map_request_bbox_t::STEREO_HEAD,
    //        drc::map_request_bbox_t::STEREO_LHAND,
    //        drc::map_request_bbox_t::STEREO_RHAND };
    //mRequestBoxDataSource = drc::map_request_bbox_t::LASER;
    //addCombo("Data Source", mRequestBoxDataSource, labels, ids, requestBox);
    /*
    addSpin("Time Window (s)", mRequestTimeWindow, 0, 30, 1, requestBox);
    mRequestRawScan = false;
    addCheck("Unfiltered Scan?", mRequestRawScan, requestBox);
    button = Gtk::manage(new Gtk::Button("Send Request"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &MapsRenderer::onSendBoxRequestButton));
    requestBox->pack_start(*button, false, false);
    mRequestBoxInit = false;
    */
  }

  notebook->show_all();

  // handler for updating ui widgets when views are added
  mViewDataDispatcher.connect
    (sigc::mem_fun(*this, &MapsRenderer::addViewWidgets));
}

void MapsRenderer::
notifyData(const int64_t iViewId) {
  addViewMetaData(iViewId);
  DataMap::const_iterator item = mViewData.find(iViewId);
  ViewClient::ViewPtr view = mViewClient.getView(iViewId);
  if ((item != mViewData.end()) && (view != NULL)) {
    Eigen::Isometry3f headToLocal;
    if (mBotWrapper->getTransform("head", "local", headToLocal,
                                  view->getUpdateTime())) {
      Eigen::Matrix3f rotMatx = headToLocal.linear();
      float theta = atan2(rotMatx(1,0), rotMatx(0,0));
      Eigen::Matrix3f rotation;
      rotation = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());
      headToLocal.linear() = rotation;
      item->second->setLatestTransform(headToLocal);
    }
  }
  requestDraw();
}

void MapsRenderer::
notifyCatalog(const bool iChanged) {
  if (!iChanged) return;
  std::vector<ViewClient::ViewPtr> views = mViewClient.getAllViews();
  DataMap::const_iterator iter;
  for (iter = mViewData.begin(); iter != mViewData.end(); ) {
    bool found = false;
    for (size_t i = 0; i < views.size(); ++i) {
      if (views[i]->getId() == iter->first) {
        found = true;
        break;
      }
    }
    if (!found) mViewData.erase(iter++);
    else ++iter;
  }
  requestDraw();
}

void MapsRenderer::
onCreateBoxButton() {
  Eigen::Isometry3f headToLocal;
  Eigen::Vector3f position(0,0,0);
  if (mBotWrapper->getTransform("head", "local", headToLocal)) {
    position = headToLocal.translation();
    position[0] += 0.5;
  }
  mInteractiveBox.createBox(position, 0.5);
  mRequestBoxInit = true;
  mShowRequestBoxToggle->set_active(true);
  requestDraw();
}

/*
void MapsRenderer::
onSendBoxRequestButton() {
  if (!mShowRequestBox) {
    std::cout << "No box shown, so request not sent" << std::endl;
    return;
  }
  sendBoxRequest();
}
*/

/*
void MapsRenderer::sendBoxRequest() {
  drc::map_request_bbox_t msg;
  Eigen::Vector3f position, scale;
  Eigen::Quaternionf orientation;
  mInteractiveBox.getBoxParameters(position, scale, orientation);
  msg.center[0] = (int16_t)(position[0]*128 + 0.5f);
  msg.center[1] = (int16_t)(position[1]*128 + 0.5f);
  msg.center[2] = (int16_t)(position[2]*128 + 0.5f);
  msg.size[0] = (uint8_t)(scale[0]*64 + 0.5f);
  msg.size[1] = (uint8_t)(scale[1]*64 + 0.5f);
  msg.size[2] = (uint8_t)(scale[2]*64 + 0.5f);
  msg.time_window = mRequestTimeWindow;
  Eigen::Vector3f rpy = orientation.matrix().eulerAngles(2,1,0);
  for (int i = 0; i < 3; ++i) msg.rpy[i] = rpy[i]*1800/acos(-1);
  msg.params = mRequestBoxDataSource;
  if (mRequestRawScan) msg.params |= drc::map_request_bbox_t::RAW_MASK;
  getLcm()->publish("MAP_REQUEST_BBOX", &msg);
  mAdjustRequestBoxToggle->set_active(false);
}
*/

void MapsRenderer::onClearViewsButton() {
  mViewClient.clearAll();
  mViewData.clear();
}

double MapsRenderer::
pickQuery(const double iRayStart[3], const double iRayDir[3]) {
  if (!mAdjustRequestBoxToggle->get_active()) {
    getBotEventHandler()->picking = 0;
    return -1;
  }
  else {
    return 0;
  }
}

bool MapsRenderer::
mousePress(const GdkEventButton* iEvent,
           const double iRayStart[3], const double iRayDir[3]) {
  if (mAdjustRequestBoxToggle->get_active()) {
    Eigen::Vector3f origin(iRayStart[0], iRayStart[1], iRayStart[2]);
    Eigen::Vector3f dir(iRayDir[0], iRayDir[1], iRayDir[2]);
    Eigen::Vector2f clickPt(iEvent->x, iEvent->y);
    return mInteractiveBox.mousePress(clickPt, iEvent->button, origin, dir);
  }
  return false;
}

bool MapsRenderer::
mouseRelease(const GdkEventButton* iEvent,
             const double iRayStart[3], const double iRayDir[3]) {
  if (mAdjustRequestBoxToggle->get_active()) {
    Eigen::Vector3f origin(iRayStart[0], iRayStart[1], iRayStart[2]);
    Eigen::Vector3f dir(iRayDir[0], iRayDir[1], iRayDir[2]);
    Eigen::Vector2f curPt(iEvent->x, iEvent->y);
    return mInteractiveBox.mouseRelease(curPt, iEvent->button, origin, dir);
  }
  return false;
}

bool MapsRenderer::
mouseMotion(const GdkEventMotion* iEvent,
            const double iRayStart[3], const double iRayDir[3]) {
  bool button1 = (iEvent->state & GDK_BUTTON1_MASK) != 0;
  bool button2 = (iEvent->state & GDK_BUTTON2_MASK) != 0;
  bool button3 = (iEvent->state & GDK_BUTTON3_MASK) != 0;

  if (mAdjustRequestBoxToggle->get_active()) {
    Eigen::Vector3f origin(iRayStart[0], iRayStart[1], iRayStart[2]);
    Eigen::Vector3f dir(iRayDir[0], iRayDir[1], iRayDir[2]);
    Eigen::Vector2f curPt(iEvent->x, iEvent->y);
    int buttonMask = (button3 << 2) | (button2 << 1) | button1;
    bool handled = mInteractiveBox.mouseMotion(curPt, buttonMask,
                                               origin, dir);
    if (handled) requestDraw();
    return handled;
  }
  return false;
}

void MapsRenderer::
getViewInfo() {
  glGetDoublev(GL_MODELVIEW_MATRIX, mModelViewGl);
  glGetDoublev(GL_PROJECTION_MATRIX, mProjectionGl);
  glGetIntegerv(GL_VIEWPORT, mViewportGl);
  mViewport.resize(4);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mModelViewMatrix(i,j) = mModelViewGl[4*j+i];
      mProjectionMatrix(i,j) = mProjectionGl[4*j+i];
    }
    mViewport[i] = mViewportGl[i];
  }
}

void MapsRenderer::
draw() {
  // draw each view
  std::vector<ViewClient::ViewPtr> views = mViewClient.getAllViews();
  for (size_t v = 0; v < views.size(); ++v) {
    int64_t id = views[v]->getId();
    addViewMetaData(id);
    ViewMetaData::Ptr data = mViewData[id];
    data->draw(views[v], mMeshRenderer);
  }

  // draw request box
  if (mRequestBoxInit) {
    getViewInfo();
    mInteractiveBox.setBoxValid(mShowRequestBox);
    mInteractiveBox.drawBox();
  }
}

void MapsRenderer::addViewMetaData(const int64_t iId) {
  // add new metadata if none exists for this id
  DataMap::const_iterator item = mViewData.find(iId);
  if (item != mViewData.end()) return;

  ViewMetaData::Ptr data(new ViewMetaData(this, iId));

  // create widget and add to list
  mViewDataDispatcher();

  // add to data list
  mViewData[iId] = data;
}

void MapsRenderer::addViewWidgets() {
  DataMap::const_iterator iter;
  for (iter = mViewData.begin(); iter != mViewData.end(); ++iter) {
    iter->second->addWidgets(mViewListBox);
  }
  mViewListBox->show_all();
}



// this is the single setup method exposed for integration with the viewer
void maps_renderer_setup(BotViewer* iViewer, const int iPriority,
                         const lcm_t* iLcm,
                         const BotParam* iParam,
                         const BotFrames* iFrames) {
  new maps::MapsRenderer(iViewer, iPriority, iLcm, iParam, iFrames);
}
