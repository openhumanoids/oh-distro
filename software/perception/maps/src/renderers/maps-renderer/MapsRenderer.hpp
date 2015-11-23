#ifndef _MapsRenderer_hpp_
#define _MapsRenderer_hpp_

#include <unordered_map>

#include <GL/gl.h>
#include <gtkmm.h>

#include <lcm/lcm-cpp.hpp>

//#include <lcmtypes/drc/world_box_t.hpp>

#include <gtkmm-renderer/RendererBase.hpp>
#include <maps/ViewClient.hpp>
#include <maps/BotWrapper.hpp>

#include "InteractiveBox.hpp"
#include "ViewMetaData.hpp"

namespace maps {

class MeshRenderer;

struct MapsRenderer : public gtkmm::RendererBase, ViewClient::Listener {
  friend class ViewMetaData;

  // view list parameters
  Gtk::VBox* mViewListBox;
  typedef std::unordered_map<int64_t,ViewMetaData::Ptr > DataMap;
  DataMap mViewData;
  Glib::Dispatcher mViewDataDispatcher;

  // request parameters
  int mRequestTimeWindow;
  bool mRequestRawScan;
  GLdouble mModelViewGl[16];
  GLdouble mProjectionGl[16];
  GLint mViewportGl[4];
  Eigen::Isometry3f mModelViewMatrix;
  Eigen::Matrix4f mProjectionMatrix;
  std::vector<int> mViewport;
  Gtk::ToggleButton* mAdjustRequestBoxToggle;
  Gtk::ToggleButton* mShowRequestBoxToggle;
  bool mShowRequestBox;
  bool mRequestBoxInit;
  int mRequestBoxDataSource;
  InteractiveBox mInteractiveBox;

  std::shared_ptr<MeshRenderer> mMeshRenderer;
  ViewClient mViewClient;
  BotWrapper::Ptr mBotWrapper;
  

  MapsRenderer(BotViewer* iViewer, const int iPriority,
               const lcm_t* iLcm,
               const BotParam* iParam, const BotFrames* iFrames);
  ~MapsRenderer();

  //void onBoxPointCloudRequest(const lcm::ReceiveBuffer* iBuf,
  //                            const std::string& iChannel,
  //                            const drc::world_box_t* iMessage);
  void onAdjustBoxToggleChanged();

  void setupWidgets();


  // view client callbacks
  void notifyData(const int64_t iViewId);
  void notifyCatalog(const bool iChanged);

  void onCreateBoxButton();
  //void onSendBoxRequestButton();
  //void sendBoxRequest();
  void onClearViewsButton();

  double pickQuery(const double iRayStart[3], const double iRayDir[3]);
  bool mousePress(const GdkEventButton* iEvent,
                  const double iRayStart[3], const double iRayDir[3]);
  bool mouseRelease(const GdkEventButton* iEvent,
                    const double iRayStart[3], const double iRayDir[3]);
  bool mouseMotion(const GdkEventMotion* iEvent,
                   const double iRayStart[3], const double iRayDir[3]);

  void getViewInfo();

  void draw();

  void addViewMetaData(const int64_t iId);
  void addViewWidgets();
};

}


#endif
