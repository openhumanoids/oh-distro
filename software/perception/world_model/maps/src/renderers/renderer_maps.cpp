#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>

#include <gdk/gdkkeysyms.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_vis/bot_vis.h>
#include <bot_core/camtrans.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/map_command_t.hpp>
#include <lcmtypes/drc/map_macro_t.hpp>

#include <drc_utils/Clock.hpp>

#include <unordered_map>

#include <octomap/octomap.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <image_utils/jpeg.h>
#include <image_utils/pixels.h>
#include <pointcloud_tools/filter_colorize.hpp>

#include <maps/ViewClient.hpp>
#include <maps/MapView.hpp>
#include <maps/Utils.hpp>

static const char* RENDERER_NAME = "Maps";

static const char* PARAM_INPUT_MODE = "Input Mode";
static const char* PARAM_REQUEST_TYPE = "Data Type";
static const char* PARAM_REQUEST_RES = "Resolution (m)";
static const char* PARAM_REQUEST_FREQ = "Frequency (Hz)";
static const char* PARAM_REQUEST_TIME = "Time Window (s)";
static const char* PARAM_DATA_REQUEST = "Send Request";
static const char* PARAM_MAP_COMMAND_TYPE = "Command";
static const char* PARAM_MAP_COMMAND = "Execute";

enum InputMode {
  INPUT_MODE_CAMERA,
  INPUT_MODE_RECT,
  INPUT_MODE_DEPTH
};

enum DataRequestType {
  REQUEST_TYPE_CLOUD,
  REQUEST_TYPE_OCTREE
};

enum MapCommand {
  MAP_COMMAND_CLEAR,
  MAP_COMMAND_SCAN_HIGH_RES
};

struct Frustum {
  std::vector<Eigen::Vector4f> mPlanes;
  float mNear;
  float mFar;
  Eigen::Vector3f mPos;
  Eigen::Vector3f mDir;
};

struct RendererMaps;

struct ViewMetaData {
  int64_t mId;
  bool mVisible;
  bool mRelative;
  Frustum mFrustum;
  Eigen::Vector3f mColor;
};

struct ViewWidget {
  int64_t mId;
  RendererMaps* mRenderer;
  GtkWidget* mBox;
};

struct RendererMaps {

  struct ViewClientListener : public maps::ViewClient::Listener {
    RendererMaps* mRenderer;
    bool mInitialized;

    void notifyData(const int64_t iViewId) {
      bot_viewer_request_redraw(mRenderer->mViewer);
    }
    void notifyCatalog(const bool iChanged) {
      if (!mInitialized || iChanged) {
        std::vector<maps::ViewClient::MapViewPtr> views =
          mRenderer->mViewClient.getAllViews();
        std::set<int64_t> catalogIds;
        for (size_t v = 0; v < views.size(); ++v) {
          int64_t id = views[v]->getSpec().mViewId;
          mRenderer->addViewMetaData(id);
          catalogIds.insert(id);
        }
        for (DataMap::iterator iter = mRenderer->mViewData.begin();
             iter != mRenderer->mViewData.end(); ) {
          int64_t id = iter->second.mId;
          if (catalogIds.find(id) == catalogIds.end()) {
            mRenderer->mViewData.erase(iter++);
            mRenderer->removeViewMetaData(id);
          }
          else {
            ++iter;
          }
        }
        mInitialized = true;
        bot_viewer_request_redraw(mRenderer->mViewer);
      }
    }
  };

  BotRenderer mRenderer;
  BotViewer* mViewer;
  boost::shared_ptr<lcm::LCM> mLcm;
  BotParam* mBotParam;
  BotFrames* mBotFrames;
  BotGtkParamWidget* mWidget;
  BotEventHandler mEventHandler;
  GtkWidget* mViewListWidget;

  typedef std::unordered_map<int64_t,ViewWidget> WidgetMap;
  WidgetMap mViewWidgets;

  typedef std::unordered_map<int64_t,ViewMetaData> DataMap;
  DataMap mViewData;
  maps::ViewClient mViewClient;
  ViewClientListener mViewClientListener;

  InputMode mInputMode;
  bool mDragging;
  Eigen::Vector2f mDragPoint1;
  Eigen::Vector2f mDragPoint2;
  float mBaseValue;
  int mWhichButton;
  Frustum mFrustum;
  bool mBoxValid;

  DataRequestType mRequestType;
  float mRequestFrequency;
  float mRequestResolution;
  float mRequestTimeWindow;

  MapCommand mMapCommand;

  BotCamTrans* mCamTrans;
  bot_core::image_t mCameraImage;

  RendererMaps() {
    mCamTrans = NULL;
    mBoxValid = false;
    mDragging = false;
    mCameraImage.size = 0;
  }

  void onCameraImage(const lcm::ReceiveBuffer* iBuf,
                     const std::string& iChannel,
                     const bot_core::image_t* iMessage) {
    mCameraImage = *iMessage;
    if (iMessage->pixelformat == PIXEL_FORMAT_MJPEG) {
      int stride = iMessage->width * 3;
      mCameraImage.data.resize(iMessage->height * stride);
      mCameraImage.pixelformat = 0;
      jpeg_decompress_8u_rgb(&iMessage->data[0], iMessage->size,
                             &mCameraImage.data[0],
                             iMessage->width, iMessage->height, stride);
    }
    else {
      // TODO: this will break unless rgb3
    }

    if (mCamTrans == NULL) {
      mCamTrans = bot_param_get_new_camtrans(mBotParam, iChannel.c_str());
      if (mCamTrans == NULL) {
        std::cout << "Error: RendererHeightMap: bad camtrans" << std::endl;
      }
    }

    bot_viewer_request_redraw(mViewer);
  }


  static void onParamWidgetChanged(BotGtkParamWidget* iWidget,
                                   const char *iName,
                                   RendererMaps *self) {
    self->mInputMode = (InputMode)
      bot_gtk_param_widget_get_enum(iWidget, PARAM_INPUT_MODE);
    self->mRequestType = (DataRequestType)
      bot_gtk_param_widget_get_enum(iWidget, PARAM_REQUEST_TYPE);
    self->mRequestFrequency = (float)
      bot_gtk_param_widget_get_double(iWidget, PARAM_REQUEST_FREQ);
    self->mRequestResolution = (float)
      bot_gtk_param_widget_get_double(iWidget, PARAM_REQUEST_RES);
    self->mRequestTimeWindow = (float)
      bot_gtk_param_widget_get_double(iWidget, PARAM_REQUEST_TIME);
    self->mMapCommand = (MapCommand)
      bot_gtk_param_widget_get_enum(iWidget, PARAM_MAP_COMMAND_TYPE);

    if (!strcmp(iName, PARAM_MAP_COMMAND)) {
      if (self->mMapCommand == MAP_COMMAND_CLEAR) {
        drc::map_command_t command;
        command.utime = drc::Clock::instance()->getCurrentTime();
        command.map_id = -1;
        command.command = drc::map_command_t::CLEAR;
        self->mLcm->publish("MAP_COMMAND", &command);
      }

      else if (self->mMapCommand == MAP_COMMAND_SCAN_HIGH_RES) {
        drc::map_macro_t macro;
        macro.utime = drc::Clock::instance()->getCurrentTime();
        macro.command = drc::map_macro_t::CREATE_DENSE_MAP;
        self->mLcm->publish("MAP_MACRO", &macro);
      }

      else {
        std::cout << "WARNING: BAD MAP COMMAND TYPE" << std::endl;
      }
    }

    if (!strcmp(iName, PARAM_DATA_REQUEST) && (self->mBoxValid)) {
      maps::MapView::Spec spec;
      spec.mMapId = -1;
      spec.mViewId = -1;
      spec.mActive = true;
      switch(self->mRequestType) {
      case REQUEST_TYPE_OCTREE:
        spec.mType = maps::MapView::Spec::TypeOctree;
        break;
      case REQUEST_TYPE_CLOUD:
        spec.mType = maps::MapView::Spec::TypeCloud;
        break;
      default:
        return;
      }
      spec.mResolution = self->mRequestResolution;
      spec.mFrequency = self->mRequestFrequency;
      spec.mTimeMin = -1;
      spec.mTimeMax = -1;
      spec.mRelativeTime = false;
      if (self->mRequestTimeWindow > 1e-3) {
        spec.mTimeMin = -self->mRequestTimeWindow*1e6;
        spec.mTimeMax = 0;
        spec.mRelativeTime = true;
      }
      spec.mClipPlanes = self->mFrustum.mPlanes;
      spec.mRelativeLocation = false;
      /* TODO
      spec.mRelativeLocation = self->mRequestRelativeLocation;
      if (spec.mRelativeLocation) {
        for (int i = 0; i < spec.mClipPlanes; ++i) {
          Eigen::Vector4f plane = spec.mClipPlanes[i];
          spec.mClipPlanes[i][3] = pos.dot(plane.head<3>()) + plane[3];
        }
      }
      */
      self->mViewClient.request(spec);
      bot_gtk_param_widget_set_enum(self->mWidget, PARAM_INPUT_MODE,
                                    INPUT_MODE_CAMERA);
      self->mBoxValid = false;
    }
  }

  static void draw(BotViewer *iViewer, BotRenderer *iRenderer) {
    RendererMaps *self = (RendererMaps*) iRenderer->user;

    // save state
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_ALL_ATTRIB_BITS);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glMatrixMode(GL_TEXTURE);
    glPushMatrix();

    // set rendering flags
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
    glEnable(GL_RESCALE_NORMAL);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // clouds
    std::vector<maps::ViewClient::MapViewPtr> views =
      self->mViewClient.getAllViews();
    for (size_t v = 0; v < views.size(); ++v) {
      maps::ViewClient::MapViewPtr view = views[v];

      // try to find ancillary data for this view; add if it doesn't exist
      int64_t id = view->getSpec().mViewId;
      self->addViewMetaData(id);
      ViewMetaData data = self->mViewData[id];

      // draw box
      if (data.mFrustum.mPlanes.size() > 0) {
        BotTrans trans;
        bot_frames_get_trans_with_utime(self->mBotFrames,
                                        "head", "local",
                                        self->mCameraImage.utime, &trans);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        if (data.mRelative) {
          glTranslatef(trans.trans_vec[0], trans.trans_vec[1],
                       trans.trans_vec[2]);
        }
        self->drawFrustum(data.mFrustum, data.mColor);
        glPopMatrix();
      }

      if (!data.mVisible) {
        continue;
      }

      // see if point cloud has any data
      maps::PointCloud::Ptr cloud = view->getAsPointCloud();
      if (cloud->size() == 0) {
        continue;
      }


      //
      // draw point cloud
      //

      glPointSize(3);
      glBegin(GL_POINTS);

      // if we should colorize
      if ((self->mCameraImage.size > 0) && (self->mCamTrans != NULL)) {
        double matx[16];
        bot_frames_get_trans_mat_4x4_with_utime(self->mBotFrames,
                                                "local", "CAMERA",
                                                self->mCameraImage.utime, matx);
        Eigen::Affine3d xform;
        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            xform(i,j) = matx[i*4+j];
          }
        }
        pcl::PointCloud<pcl::PointXYZRGB> cloudColor;
        FilterColorize::colorize<maps::PointCloud::PointType,
                                 pcl::PointXYZRGB>
          (*cloud, xform, self->mCameraImage,
           self->mCamTrans, cloudColor);
        for (size_t i = 0; i < cloudColor.size(); ++i) {
          const pcl::PointXYZRGB pt = cloudColor[i];
          if ((pt.r == 0) && (pt.g == 0) && (pt.b == 0)) {
            glColor3f(data.mColor[0], data.mColor[1], data.mColor[2]);
          }
          else {
            glColor3b(pt.r, pt.g, pt.b);
          }
          glVertex3f(pt.x, pt.y, pt.z);
        }
      }

      // otherwise choose single color
      else {
        glColor3f(data.mColor[0], data.mColor[1], data.mColor[2]);
        for (size_t i = 0; i < cloud->size(); ++i) {
          const maps::PointCloud::PointType pt = (*cloud)[i];
          glVertex3f(pt.x, pt.y, pt.z);
        }
      }

      glEnd();
    }


    // draw drag rectangle
    if (self->mDragging && self->mInputMode == INPUT_MODE_RECT) {
      int width = GTK_WIDGET(iViewer->gl_area)->allocation.width;
      int height = GTK_WIDGET(iViewer->gl_area)->allocation.height;
      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      glOrtho(0, width, height, 0, -1, 1);
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glLoadIdentity();
      glDisable(GL_DEPTH_TEST);

      glColor3f(0,0,1);
      glBegin(GL_LINE_LOOP);
      glVertex2f(self->mDragPoint1[0], self->mDragPoint1[1]);
      glVertex2f(self->mDragPoint2[0], self->mDragPoint1[1]);
      glVertex2f(self->mDragPoint2[0], self->mDragPoint2[1]);
      glVertex2f(self->mDragPoint1[0], self->mDragPoint2[1]);
      glEnd();

      glEnable(GL_DEPTH_TEST);
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }

    if (self->mBoxValid) {
      self->updateFrontAndBackPlanes(self->mFrustum);
      self->drawFrustum(self->mFrustum, Eigen::Vector3f(0,0,1));
    }

    // restore state
    glMatrixMode(GL_TEXTURE);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glPopClientAttrib();
    glPopAttrib();
  }

  void removeViewMetaData(const int64_t iId) {
    mViewData.erase(iId);
    WidgetMap::iterator item = mViewWidgets.find(iId);
    if (item != mViewWidgets.end()) {
      gtk_widget_destroy(item->second.mBox);
      mViewWidgets.erase(item);
    }
  }

  void addViewMetaData(const int64_t iId) {
    bool added = false;
    DataMap::const_iterator item = mViewData.find(iId);
    ViewMetaData data;
    if (item == mViewData.end()) {
      added = true;
      data.mId = iId;
      data.mVisible = true;
      data.mRelative = false;
      if (data.mId != 1) {
        data.mColor = Eigen::Vector3f((double)rand()/RAND_MAX,
                                      (double)rand()/RAND_MAX,
                                      (double)rand()/RAND_MAX);
      }
      else {
        data.mColor = Eigen::Vector3f(1,0,0);
      }
      mViewData[iId] = data;
    }

    maps::MapView::Ptr mapView = mViewClient.getView(iId);
    if (mapView != NULL) {
      const std::vector<Eigen::Vector4f>& planes =
        mapView->getSpec().mClipPlanes;
      if (planes.size() != data.mFrustum.mPlanes.size()) {
        mViewData[iId].mFrustum.mPlanes = mapView->getSpec().mClipPlanes;
        mViewData[iId].mRelative = mapView->getSpec().mRelativeLocation;
      }
    }


    if (added) {
      char name[256];
      sprintf(name, "%ld", iId);

      GtkBox *hb = GTK_BOX(gtk_hbox_new (FALSE, 0));

      ViewWidget viewWidget;
      viewWidget.mId = iId;
      viewWidget.mRenderer = this;
      viewWidget.mBox = GTK_WIDGET(hb);
      mViewWidgets[iId] = viewWidget;

      GtkWidget* cb = gtk_toggle_button_new_with_label("                ");
      GdkColor color, colorDim;
      color.red =   data.mColor[0]*65535;
      color.green = data.mColor[1]*65535;
      color.blue =  data.mColor[2]*65535;
      colorDim.red =   (data.mColor[0]+1)/2*65535;
      colorDim.green = (data.mColor[1]+1)/2*65535;
      colorDim.blue =  (data.mColor[2]+1)/2*65535;
      gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(cb), TRUE);
      gtk_widget_modify_bg(GTK_WIDGET(cb), GTK_STATE_ACTIVE, &color);
      gtk_widget_modify_bg(GTK_WIDGET(cb), GTK_STATE_NORMAL, &colorDim);
      gtk_box_pack_start(GTK_BOX(hb), cb, FALSE, FALSE, 0);
      g_signal_connect(G_OBJECT(cb), "toggled",
                       G_CALLBACK(&RendererMaps::onToggleButton),
                       &mViewWidgets[iId]);

      if (iId != 1) {
        GtkWidget* cancelButton = gtk_button_new_with_label("X");
        gtk_box_pack_start(GTK_BOX(hb), cancelButton, FALSE, FALSE, 0);
        g_signal_connect(G_OBJECT(cancelButton), "clicked",
                         G_CALLBACK(&RendererMaps::onCancelButton),
                         &mViewWidgets[iId]);
      }

      gtk_widget_show_all(GTK_WIDGET(hb));
      gtk_box_pack_start(GTK_BOX(mViewListWidget), GTK_WIDGET(hb),
                         TRUE, TRUE, 0);
    }
  }

  static void onToggleButton(GtkWidget *iButton, ViewWidget* data) {
    bool checked = gtk_toggle_button_get_active((GtkToggleButton*)iButton);
    int64_t id = data->mId;
    DataMap::iterator item = data->mRenderer->mViewData.find(id);
    if (item != data->mRenderer->mViewData.end()) {
      item->second.mVisible = checked;
      bot_viewer_request_redraw (data->mRenderer->mViewer);    
    }
  }

  static void onCancelButton(GtkWidget *iButton, ViewWidget* data) {
    int64_t id = data->mId;
    DataMap::iterator item = data->mRenderer->mViewData.find(id);
    if (item != data->mRenderer->mViewData.end()) {
      maps::MapView::Spec spec;
      spec.mMapId = 0;
      spec.mViewId = id;
      spec.mResolution = spec.mFrequency = 0;
      spec.mTimeMin = spec.mTimeMax = 0;
      spec.mActive = false;
      spec.mType = maps::MapView::Spec::TypeCloud;
      data->mRenderer->mViewClient.request(spec);
      bot_viewer_request_redraw (data->mRenderer->mViewer);
    }
    // TODO: guard thread safety for access to data pointer
  }

  static void drawFrustum(const Frustum& iFrustum,
                          const Eigen::Vector3f& iColor) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<std::vector<int> > faces;
    maps::Utils::polyhedronFromPlanes(iFrustum.mPlanes, vertices, faces);
    glColor3f(iColor[0], iColor[1], iColor[2]);
    glLineWidth(3);
    for (size_t i = 0; i < faces.size(); ++i) {
      glBegin(GL_LINE_LOOP);
      for (size_t j = 0; j < faces[i].size(); ++j) {
        Eigen::Vector3f pt = vertices[faces[i][j]];
        glVertex3f(pt[0], pt[1], pt[2]);
      }
      glEnd();
    }
  }

  static Eigen::Vector4f computePlane(Eigen::Vector3f& p1, Eigen::Vector3f& p2,
                                      Eigen::Vector3f& p3) {
    Eigen::Vector4f plane;
    Eigen::Vector3f d1 = p2-p1;
    Eigen::Vector3f d2 = p3-p1;
    Eigen::Vector3f normal = d1.cross(d2);
    normal.normalize();
    plane.head<3>() = normal;
    plane[3] = -normal.dot(p1);
    return plane;
  }

  void updateFrontAndBackPlanes(Frustum& ioFrustum) {
    Eigen::Vector4f plane;
    Eigen::Vector3f pt;
    plane.head<3>() = ioFrustum.mDir;
    plane[3] = -ioFrustum.mDir.dot(ioFrustum.mPos +
                                   ioFrustum.mDir*ioFrustum.mNear);
    ioFrustum.mPlanes[4] = plane;
    plane = -plane;
    plane[3] = ioFrustum.mDir.dot(ioFrustum.mPos +
                                  ioFrustum.mDir*ioFrustum.mFar);
    ioFrustum.mPlanes[5] = plane;
  }

  static Eigen::Vector3f intersect(const Eigen::Vector3f& iOrigin,
                                   const Eigen::Vector3f& iDir,
                                   const Eigen::Vector4f& iPlane) {
    Eigen::Vector3f normal = iPlane.head<3>();
    double t = -(iPlane[3] + normal.dot(iOrigin)) / normal.dot(iDir);
    return (iOrigin + iDir*t);
  }


  //
  // event stuff
  //

  static int keyPress(BotViewer* iViewer, BotEventHandler* iHandler, 
                      const GdkEventKey* iEvent) {
    RendererMaps* self = (RendererMaps*)iHandler->user;
    if ((self->mInputMode != INPUT_MODE_CAMERA) &&
        (iEvent->keyval == GDK_Escape)) {
      bot_gtk_param_widget_set_enum(self->mWidget, PARAM_INPUT_MODE,
                                    INPUT_MODE_CAMERA);
      return 0;
    }
    return 0;
  }

  static int mousePress(BotViewer* iViewer, BotEventHandler* iHandler,
                        const double iRayOrg[3], const double iRayDir[3],
                        const GdkEventButton* iEvent) {
    RendererMaps* self = (RendererMaps*)iHandler->user;

    if (self->mInputMode == INPUT_MODE_CAMERA) {
    }
    else if (self->mInputMode == INPUT_MODE_RECT) {
      if (iEvent->button == 1) {
        self->mDragging = true;
        self->mBoxValid = false;
        self->mDragPoint1[0] = iEvent->x;
        self->mDragPoint1[1] = iEvent->y;
        self->mDragPoint2 = self->mDragPoint1;
        bot_viewer_request_redraw(self->mViewer);
        return 1;
      }
      else {}
    }
    else if (self->mInputMode == INPUT_MODE_DEPTH) {
      if (self->mBoxValid) {
        if ((iEvent->button == 1) || (iEvent->button == 3)) {
          self->mDragging = true;
          self->mWhichButton = iEvent->button;
          self->mDragPoint1[0] = iEvent->x;
          self->mDragPoint1[1] = iEvent->y;
          self->mDragPoint2 = self->mDragPoint1;
          self->mBaseValue = (iEvent->button == 1) ? self->mFrustum.mNear :
            self->mFrustum.mFar;
          bot_viewer_request_redraw(self->mViewer);
          return 1;
        }
        else {}
      }
    }
    else {}
    bot_viewer_request_redraw(self->mViewer);
    return 0;
  }

  static int mouseRelease(BotViewer* iViewer, BotEventHandler* iHandler,
                          const double iRayOrg[3], const double iRayDir[3],
                          const GdkEventButton* iEvent) {
    RendererMaps* self = (RendererMaps*)iHandler->user;

    if (self->mInputMode == INPUT_MODE_CAMERA) {
    }
    else if (self->mInputMode == INPUT_MODE_RECT) {
      if (iEvent->button == 1) {
        self->mDragging = false;

        if ((self->mDragPoint2-self->mDragPoint1).norm() < 1) {
          bot_viewer_request_redraw(self->mViewer);
          return 0;
        }
        
        // get view info
        GLdouble modelViewGl[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelViewGl);
        GLdouble projGl[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projGl);
        GLint viewportGl[4];
        glGetIntegerv(GL_VIEWPORT, viewportGl);

        Eigen::Isometry3f modelView;
        Eigen::Matrix4f proj;
        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            modelView(i,j) = modelViewGl[4*j+i];
            proj(i,j) = projGl[4*j+i];
          }
        }

        // eye position and look direction
        Eigen::Vector3f pos =
          -modelView.linear().inverse()*modelView.translation();
        Eigen::Vector3f dir = -modelView.linear().block<1,3>(2,0);
        dir.normalize();
        self->mFrustum.mPos = pos;
        self->mFrustum.mDir = dir;

        // compute four corner rays
        int height = GTK_WIDGET(iViewer->gl_area)->allocation.height;
        double u1(self->mDragPoint1[0]), v1(height-self->mDragPoint1[1]);
        double u2(self->mDragPoint2[0]), v2(height-self->mDragPoint2[1]);
        double x,y,z;
        gluUnProject(u1,v1,0,modelViewGl,projGl,viewportGl,&x,&y,&z);
        Eigen::Vector3f p1(x,y,z);
        gluUnProject(u2,v1,0,modelViewGl,projGl,viewportGl,&x,&y,&z);
        Eigen::Vector3f p2(x,y,z);
        gluUnProject(u2,v2,0,modelViewGl,projGl,viewportGl,&x,&y,&z);
        Eigen::Vector3f p3(x,y,z);
        gluUnProject(u1,v2,0,modelViewGl,projGl,viewportGl,&x,&y,&z);
        Eigen::Vector3f p4(x,y,z);

        self->mFrustum.mNear = 5;
        self->mFrustum.mFar = 10;

        // first four planes
        self->mFrustum.mPlanes.resize(6);
        self->mFrustum.mPlanes[0] = self->computePlane(pos, p1, p2);
        self->mFrustum.mPlanes[1] = self->computePlane(pos, p2, p3);
        self->mFrustum.mPlanes[2] = self->computePlane(pos, p3, p4);
        self->mFrustum.mPlanes[3] = self->computePlane(pos, p4, p1);

        // front and back
        self->updateFrontAndBackPlanes(self->mFrustum);

        self->mBoxValid = true;

        bot_viewer_request_redraw(self->mViewer);
        return 1;
      }
      else {}
    }
    else if (self->mInputMode == INPUT_MODE_DEPTH) {
      self->mDragging = false;
    }
    else {}
    bot_viewer_request_redraw(self->mViewer);
    return 0;
  }

  static int mouseMotion(BotViewer* iViewer, BotEventHandler* iHandler,
                         const double iRayOrg[3], const double iRayDir[3],
                         const GdkEventMotion* iEvent) {
    RendererMaps* self = (RendererMaps*)iHandler->user;

    if (self->mInputMode == INPUT_MODE_CAMERA) {
    }
    else if (self->mInputMode == INPUT_MODE_RECT) {
      if (self->mDragging) {
        self->mDragPoint2[0] = iEvent->x;
        self->mDragPoint2[1] = iEvent->y;
        bot_viewer_request_redraw(self->mViewer);
        return 1;
      }
      else {}
    }
    else if (self->mInputMode == INPUT_MODE_DEPTH) {
      if (self->mBoxValid && self->mDragging) {
        self->mDragPoint2[0] = iEvent->x;
        self->mDragPoint2[1] = iEvent->y;
        float dist = self->mDragPoint2[1]-self->mDragPoint1[1];
        float scaledDist = dist/100;
        if (self->mWhichButton == 1) {
          self->mFrustum.mNear = std::max(0.0f, self->mBaseValue+scaledDist);
          bot_viewer_request_redraw(self->mViewer);
          return 1;
        }
        else if (self->mWhichButton == 3) {
          self->mFrustum.mFar = std::max(0.0f, self->mBaseValue+scaledDist);
          bot_viewer_request_redraw(self->mViewer);
          return 1;
        }
        else {}
      }
      else {}
    }
    else {}
    bot_viewer_request_redraw(self->mViewer);
    return 0;
  }

};


// TODO: move these inside

static void
maps_renderer_free (BotRenderer *renderer) {
  RendererMaps *self = (RendererMaps*) renderer;
  if (self->mCamTrans != NULL) {
    bot_camtrans_destroy(self->mCamTrans);
  }
  delete self;
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data) {
  RendererMaps *self = (RendererMaps*) user_data;
  bot_gtk_param_widget_load_from_key_file (self->mWidget, keyfile,
                                           RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data) {
    RendererMaps *self = (RendererMaps*)user_data;
    bot_gtk_param_widget_save_to_key_file (self->mWidget, keyfile,
                                           RENDERER_NAME);
}

void
maps_add_renderer_to_viewer(BotViewer* viewer,
                            int priority,
                            lcm_t* lcm,
                            BotParam* param,
                            BotFrames* frames) {
  RendererMaps *self = new RendererMaps();
  self->mViewer = viewer;
  self->mRenderer.draw = RendererMaps::draw;
  self->mRenderer.destroy = maps_renderer_free;
  self->mRenderer.name = (char*)RENDERER_NAME;
  self->mRenderer.user = self;
  self->mRenderer.enabled = 1;
  self->mRenderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
  self->mLcm.reset(new lcm::LCM(lcm));
  self->mBotParam = param;
  self->mBotFrames = frames;

  drc::Clock::instance()->setLcm(self->mLcm);

  // events
  BotEventHandler* handler = &self->mEventHandler;
  memset(handler, 0, sizeof(BotEventHandler));
  handler->name = (char*)RENDERER_NAME;
  handler->enabled = 1;
  handler->key_press = RendererMaps::keyPress;
  handler->mouse_press = RendererMaps::mousePress;
  handler->mouse_release = RendererMaps::mouseRelease;
  handler->mouse_motion = RendererMaps::mouseMotion;
  handler->user = self;
  bot_viewer_add_event_handler(viewer, &self->mEventHandler, priority);


  self->mWidget = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
  gtk_container_add (GTK_CONTAINER (self->mRenderer.widget),
                     GTK_WIDGET(self->mWidget));
  gtk_widget_show (GTK_WIDGET (self->mWidget));

  bot_gtk_param_widget_add_separator(self->mWidget, "view creation");

  bot_gtk_param_widget_add_enum(self->mWidget, PARAM_INPUT_MODE,
                                BOT_GTK_PARAM_WIDGET_MENU, INPUT_MODE_CAMERA,
                                "Move View", INPUT_MODE_CAMERA,
                                "Drag Rect", INPUT_MODE_RECT,
                                "Set Depth", INPUT_MODE_DEPTH, NULL);

  bot_gtk_param_widget_add_enum(self->mWidget, PARAM_REQUEST_TYPE,
                                BOT_GTK_PARAM_WIDGET_MENU, REQUEST_TYPE_CLOUD,
                                "Cloud", REQUEST_TYPE_CLOUD,
                                "Octree", REQUEST_TYPE_OCTREE, NULL);

  bot_gtk_param_widget_add_double(self->mWidget, PARAM_REQUEST_FREQ,
                                  BOT_GTK_PARAM_WIDGET_SPINBOX,
                                  0, 5, 0.1, 1);

  bot_gtk_param_widget_add_double(self->mWidget, PARAM_REQUEST_RES,
                                  BOT_GTK_PARAM_WIDGET_SPINBOX,
                                  0.01, 1, 0.01, 0.05);

  bot_gtk_param_widget_add_double(self->mWidget, PARAM_REQUEST_TIME,
                                  BOT_GTK_PARAM_WIDGET_SPINBOX,
                                  0, 10, 0.1, 0);

  bot_gtk_param_widget_add_buttons(self->mWidget, PARAM_DATA_REQUEST, NULL);

  bot_gtk_param_widget_add_separator(self->mWidget, "macro commands");

  bot_gtk_param_widget_add_enum(self->mWidget, PARAM_MAP_COMMAND_TYPE,
                                BOT_GTK_PARAM_WIDGET_MENU, MAP_COMMAND_CLEAR,
                                "Clear Map", MAP_COMMAND_CLEAR,
                                "High-Res Scan", MAP_COMMAND_SCAN_HIGH_RES,
                                NULL);
  bot_gtk_param_widget_add_buttons(self->mWidget, PARAM_MAP_COMMAND, NULL);

  bot_gtk_param_widget_add_separator(self->mWidget, "view list");
  self->mViewListWidget = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (self->mWidget), self->mViewListWidget);
  gtk_widget_show (self->mViewListWidget);
  
  g_signal_connect (G_OBJECT (self->mWidget), "changed",
                    G_CALLBACK (RendererMaps::onParamWidgetChanged),
                    self);


  g_signal_connect (G_OBJECT (viewer), "load-preferences",
                    G_CALLBACK (on_load_preferences), self);
  g_signal_connect (G_OBJECT (viewer), "save-preferences",
                    G_CALLBACK (on_save_preferences), self);

  self->mViewClient.setLcm(self->mLcm);
  self->mViewClient.setOctreeChannel("MAP_OCTREE");
  self->mViewClient.setCloudChannel("MAP_CLOUD");
  self->mViewClientListener.mRenderer = self;
  self->mViewClientListener.mInitialized = false;
  self->mViewClient.addListener(&self->mViewClientListener);
  self->mViewClient.start();

  self->mLcm->subscribe("CAMERALEFT", &RendererMaps::onCameraImage, self);

  std::cout << "Finished Setting Up Maps Renderer" << std::endl;

  bot_viewer_add_renderer(viewer, &self->mRenderer, priority);
}
