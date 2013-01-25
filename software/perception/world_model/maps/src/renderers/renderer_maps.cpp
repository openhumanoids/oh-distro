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

#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/map_octree_t.hpp>
#include <lcmtypes/drc/map_cloud_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>

#include <zlib.h>
#include <image_utils/jpeg.h>
#include <image_utils/pixels.h>

#include <unordered_map>

#include <octomap/octomap.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <maps/DataBlob.hpp>

static const char* RENDERER_NAME = "Maps";

static const char* PARAM_INPUT_MODE = "Input Mode";
static const char* PARAM_REQUEST_TYPE = "Data Type";
static const char* PARAM_REQUEST_RES = "Resolution m";
static const char* PARAM_REQUEST_FREQ = "Frequency Hz";
static const char* PARAM_DATA_REQUEST = "Send Request";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudType;

enum InputMode {
  INPUT_MODE_NONE,
  INPUT_MODE_RECT,
  INPUT_MODE_DEPTH
};

enum DataRequestType {
  REQUEST_TYPE_CLOUD,
  REQUEST_TYPE_OCTREE
};

struct Frustum {
  std::vector<Eigen::Vector4f> mPlanes;
  float mNear;
  float mFar;
  Eigen::Vector3f mPos;
  Eigen::Vector3f mDir;
  Eigen::Vector3f mRay1;
  Eigen::Vector3f mRay2;
  Eigen::Vector3f mRay3;
  Eigen::Vector3f mRay4;
};

struct CloudData {
  int mId;
  PointCloudType::Ptr mCloud;
  Frustum mFrustum;
  Eigen::Vector3f mColor;
};


static const double kBoxPoints[8][3] = {
  {-0.5,-0.5,-0.5}, {0.5,-0.5,-0.5}, {0.5,0.5,-0.5}, {-0.5,0.5,-0.5},
  {-0.5,-0.5,0.5}, {0.5,-0.5,0.5}, {0.5,0.5,0.5}, {-0.5,0.5,0.5}
};
static const int kBoxQuads[6][4] = {
  {3,2,1,0}, {4,5,6,7}, {0,4,7,3}, {1,2,6,5}, {0,1,5,4}, {3,7,6,2}
};


struct RendererMaps {

  BotRenderer mRenderer;
  BotViewer* mViewer;
  boost::shared_ptr<lcm::LCM> mLcm;
  BotParam* mBotParam;
  BotFrames* mBotFrames;
  BotGtkParamWidget* mWidget;
  BotEventHandler mEventHandler;

  typedef std::unordered_map<int64_t,CloudData> DataMap;
  DataMap mClouds;

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

  BotCamTrans* mCamTrans;

  RendererMaps() {
    mCamTrans = NULL;
    mBoxValid = false;
    mDragging = false;
  }

  void onCloud(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const drc::map_cloud_t* iMessage) {

    // transform from cloud to reference coords
    Eigen::Affine3f matx;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        matx(i,j) = iMessage->transform[i][j];
      }
    }

    // data blob
    maps::DataBlob::Spec spec;
    const drc::map_blob_t& msgBlob = iMessage->blob;
    spec.mDimensions.resize(msgBlob.dimensions.size());
    std::copy(msgBlob.dimensions.begin(), msgBlob.dimensions.end(),
              spec.mDimensions.begin());
    spec.mStrideBytes.resize(msgBlob.stride_bytes.size());
    std::copy(msgBlob.stride_bytes.begin(), msgBlob.stride_bytes.end(),
              spec.mStrideBytes.begin());
    switch (msgBlob.compression) {
    case drc::map_blob_t::UNCOMPRESSED:
      spec.mCompressionType = maps::DataBlob::CompressionTypeNone;
      break;
    case drc::map_blob_t::ZLIB:
      spec.mCompressionType = maps::DataBlob::CompressionTypeZlib;
      break;
    default:
      return;
    }
    switch (msgBlob.data_type) {
    case drc::map_blob_t::UINT8:
      spec.mDataType = maps::DataBlob::DataTypeUint8;
      break;
    case drc::map_blob_t::UINT16:
      spec.mDataType = maps::DataBlob::DataTypeUint16;
      break;
    case drc::map_blob_t::FLOAT32:
      spec.mDataType = maps::DataBlob::DataTypeFloat32;
      break;
    default:
      return;
    }
    maps::DataBlob blob;
    blob.setData(msgBlob.data, spec);

    // convert to point cloud
    blob.convertTo(maps::DataBlob::CompressionTypeNone,
                   maps::DataBlob::DataTypeFloat32);
    float* raw = (float*)(&blob.getBytes()[0]);
    PointCloudType::Ptr cloud(new PointCloudType());
    cloud->resize(spec.mDimensions[1]);
    for (size_t i = 0; i < cloud->size(); ++i) {
      (*cloud)[i].x = raw[i*3 + 0];
      (*cloud)[i].y = raw[i*3 + 1];
      (*cloud)[i].z = raw[i*3 + 2];
    }    

    // transform and add
    // TODO: separate some of this out
    pcl::transformPointCloud(*cloud, *cloud, matx);
    DataMap::iterator item = mClouds.find(iMessage->view_id);
    if (item == mClouds.end()) {
      CloudData data;
      data.mId = iMessage->view_id;
      data.mCloud = cloud;
      if (data.mId != 1) {
        data.mColor = Eigen::Vector3f((double)rand()/RAND_MAX,
                                      (double)rand()/RAND_MAX,
                                      (double)rand()/RAND_MAX);
      }
      else {
        data.mColor = Eigen::Vector3f(0,1,0);
      }
      mClouds[iMessage->view_id] = data;
    }
    else {
      item->second.mCloud = cloud;
    }
  }

  void onOctree(const lcm::ReceiveBuffer* iBuf,
                const std::string& iChannel,
                const drc::map_octree_t* iMessage) {

    // TODO: move heavy lifting out of lcm thread

    // transform from reference to octree coords
    Eigen::Isometry3f matx;
    Eigen::Quaternionf q(iMessage->transform.rotation.w,
                         iMessage->transform.rotation.x,
                         iMessage->transform.rotation.y,
                         iMessage->transform.rotation.z);
    Eigen::Vector3f pos(iMessage->transform.translation.x,
                        iMessage->transform.translation.y,
                        iMessage->transform.translation.z);
    matx.linear() = q.matrix();
    matx.translation() = pos;
    std::string str(iMessage->data.begin(), iMessage->data.end());
    std::stringstream ss(str);
    octomap::OcTree oct(0.1);
    oct.readBinary(ss);
    PointCloudType::Ptr cloud(new PointCloudType());
    octomap::OcTree::leaf_iterator iter = oct.begin_leafs();
    for (; iter != oct.end_leafs(); ++iter) {
      if (oct.isNodeOccupied(*iter)) {
        PointCloudType::PointType pt(iter.getX(), iter.getY(), iter.getZ());
        cloud->push_back(pt);
      }
    }
    pcl::transformPointCloud(*cloud, *cloud, matx);
    DataMap::iterator item = mClouds.find(iMessage->view_id);
    if (item == mClouds.end()) {
      CloudData data;
      data.mId = iMessage->view_id;
      data.mCloud = cloud;
      if (data.mId != 1) {
        data.mColor = Eigen::Vector3f((double)rand()/RAND_MAX,
                                      (double)rand()/RAND_MAX,
                                      (double)rand()/RAND_MAX);
      }
      else {
        data.mColor = Eigen::Vector3f(1,0,0);
      }
      mClouds[iMessage->view_id] = data;
    }
    else {
      item->second.mCloud = cloud;
    }
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

    if (!strcmp(iName, PARAM_DATA_REQUEST) && (self->mBoxValid)) {
      drc::map_request_t request;
      request.map_id = 0;
      request.view_id = ((int64_t)rand() << 31) + rand();
      switch(self->mRequestType) {
      case REQUEST_TYPE_OCTREE:
        request.type = drc::map_request_t::OCTREE;
        break;
      case REQUEST_TYPE_CLOUD:
        request.type = drc::map_request_t::CLOUD;
        break;
      default:
        return;
      }
      request.resolution = self->mRequestResolution;
      request.frequency = self->mRequestFrequency;
      request.time_min = -1;
      request.time_max = -1;
      request.num_clip_planes = 6;
      request.clip_planes.resize(request.num_clip_planes);
      for (int i = 0; i < 6; ++i) {
        request.clip_planes[i].resize(4);
        for (int j = 0; j < 4; ++j) {
          request.clip_planes[i][j] = self->mFrustum.mPlanes[i][j];
        }
      }
      self->mLcm->publish("MAP_REQUEST", &request);
      bot_gtk_param_widget_set_enum(self->mWidget, PARAM_INPUT_MODE,
                                    INPUT_MODE_NONE);
      CloudData data;
      data.mId = request.view_id;
      data.mColor = Eigen::Vector3f((double)rand()/RAND_MAX,
                                    (double)rand()/RAND_MAX,
                                    (double)rand()/RAND_MAX);
      data.mFrustum = self->mFrustum;
      self->mClouds[data.mId] = data;
      self->mBoxValid = false;
    }
  
    bot_viewer_request_redraw(self->mViewer);
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

    // TODO: make separate objects and render methods

    // clouds
    for (DataMap::const_iterator iter = self->mClouds.begin();
         iter != self->mClouds.end(); ++iter) {
      const CloudData& data = iter->second;
      if ((data.mCloud == NULL) || (data.mCloud->size() == 0)) {
        continue;
      }
      const PointCloudType& cloud = *data.mCloud;
      glColor3f(data.mColor[0], data.mColor[1], data.mColor[2]);
      glPointSize(3);
      glBegin(GL_POINTS);
      for (size_t i = 0; i < cloud.size(); ++i) {
        const PointCloudType::PointType pt = cloud[i];
        glVertex3f(pt.x, pt.y, pt.z);
      }
      glEnd();
      if (data.mFrustum.mPlanes.size() > 0) {
        self->drawFrustum(data.mFrustum, data.mColor);
      }
    }


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

  static void drawFrustum(const Frustum& iFrustum,
                          const Eigen::Vector3f& iColor) {
    // intersect each ray with front and back plane
    std::vector<Eigen::Vector3f> p(8);
    p[0] = intersect(iFrustum.mPos, iFrustum.mRay1, iFrustum.mPlanes[4]);
    p[1] = intersect(iFrustum.mPos, iFrustum.mRay2, iFrustum.mPlanes[4]);
    p[2] = intersect(iFrustum.mPos, iFrustum.mRay3, iFrustum.mPlanes[4]);
    p[3] = intersect(iFrustum.mPos, iFrustum.mRay4, iFrustum.mPlanes[4]);
    p[4] = intersect(iFrustum.mPos, iFrustum.mRay1, iFrustum.mPlanes[5]);
    p[5] = intersect(iFrustum.mPos, iFrustum.mRay2, iFrustum.mPlanes[5]);
    p[6] = intersect(iFrustum.mPos, iFrustum.mRay3, iFrustum.mPlanes[5]);
    p[7] = intersect(iFrustum.mPos, iFrustum.mRay4, iFrustum.mPlanes[5]);
    glColor3f(iColor[0], iColor[1], iColor[2]);
    for (int i = 0; i < 6; ++i) {
      glBegin(GL_LINE_LOOP);
      for (int j = 0; j < 4; ++j) {
        glVertex3f(p[kBoxQuads[i][j]][0], p[kBoxQuads[i][j]][1],
                   p[kBoxQuads[i][j]][2]);
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
    if ((self->mInputMode != INPUT_MODE_NONE) &&
        (iEvent->keyval == GDK_Escape)) {
      bot_gtk_param_widget_set_enum(self->mWidget, PARAM_INPUT_MODE,
                                    INPUT_MODE_NONE);
      return 1;
    }
    return 0;
  }

  static int mousePress(BotViewer* iViewer, BotEventHandler* iHandler,
                        const double iRayOrg[3], const double iRayDir[3],
                        const GdkEventButton* iEvent) {
    RendererMaps* self = (RendererMaps*)iHandler->user;

    if (self->mInputMode == INPUT_MODE_NONE) {
    }
    else if (self->mInputMode == INPUT_MODE_RECT) {
      if (iEvent->button == 1) {
        self->mDragging = true;
        self->mBoxValid = false;
        self->mDragPoint1[0] = iEvent->x;
        self->mDragPoint1[1] = iEvent->y;
        self->mDragPoint2 = self->mDragPoint1;
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
          return 1;
        }
        else {}
      }
    }
    else {}
    return 0;
  }

  static int mouseRelease(BotViewer* iViewer, BotEventHandler* iHandler,
                          const double iRayOrg[3], const double iRayDir[3],
                          const GdkEventButton* iEvent) {
    RendererMaps* self = (RendererMaps*)iHandler->user;

    if (self->mInputMode == INPUT_MODE_NONE) {
    }
    else if (self->mInputMode == INPUT_MODE_RECT) {
      if (iEvent->button == 1) {
        self->mDragging = false;

        if ((self->mDragPoint2-self->mDragPoint1).norm() < 1) {
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
        self->mFrustum.mRay1 = p1-pos;
        self->mFrustum.mRay2 = p2-pos;
        self->mFrustum.mRay3 = p3-pos;
        self->mFrustum.mRay4 = p4-pos;
        self->mFrustum.mRay1.normalize();
        self->mFrustum.mRay2.normalize();
        self->mFrustum.mRay3.normalize();
        self->mFrustum.mRay4.normalize();

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

        return 1;
      }
      else {}
    }
    else if (self->mInputMode == INPUT_MODE_DEPTH) {
      self->mDragging = false;
    }
    else {}
    return 0;
  }

  static int mouseMotion(BotViewer* iViewer, BotEventHandler* iHandler,
                         const double iRayOrg[3], const double iRayDir[3],
                         const GdkEventMotion* iEvent) {
    RendererMaps* self = (RendererMaps*)iHandler->user;

    if (self->mInputMode == INPUT_MODE_NONE) {
    }
    else if (self->mInputMode == INPUT_MODE_RECT) {
      if (self->mDragging) {
        self->mDragPoint2[0] = iEvent->x;
        self->mDragPoint2[1] = iEvent->y;
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
          return 1;
        }
        else if (self->mWhichButton == 3) {
          self->mFrustum.mFar = std::max(0.0f, self->mBaseValue+scaledDist);
          return 1;
        }
        else {}
      }
      else {}
    }
    else {}
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

  // events
  BotEventHandler* handler = &self->mEventHandler;
  handler->name = (char*)RENDERER_NAME;
  handler->enabled = 1;
  handler->picking = 0;
  handler->pick_query = NULL;
  handler->key_press = RendererMaps::keyPress;
  handler->hover_query = NULL;
  handler->mouse_press = RendererMaps::mousePress;
  handler->mouse_release = RendererMaps::mouseRelease;
  handler->mouse_motion = RendererMaps::mouseMotion;
  handler->user = self;
  bot_viewer_add_event_handler(viewer, &self->mEventHandler, priority);


  self->mWidget = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
  gtk_container_add (GTK_CONTAINER (self->mRenderer.widget),
                     GTK_WIDGET(self->mWidget));
  gtk_widget_show (GTK_WIDGET (self->mWidget));

  bot_gtk_param_widget_add_enum(self->mWidget, PARAM_INPUT_MODE,
                                BOT_GTK_PARAM_WIDGET_MENU, INPUT_MODE_NONE,
                                "None", INPUT_MODE_NONE,
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

  bot_gtk_param_widget_add_buttons(self->mWidget, PARAM_DATA_REQUEST, NULL);
  
  g_signal_connect (G_OBJECT (self->mWidget), "changed",
                    G_CALLBACK (RendererMaps::onParamWidgetChanged),
                    self);


  // save widget modes:
  g_signal_connect (G_OBJECT (viewer), "load-preferences",
                    G_CALLBACK (on_load_preferences), self);
  g_signal_connect (G_OBJECT (viewer), "save-preferences",
                    G_CALLBACK (on_save_preferences), self);

  self->mLcm->subscribe("MAP_OCTREE", &RendererMaps::onOctree, self);
  self->mLcm->subscribe("MAP_CLOUD", &RendererMaps::onCloud, self);

  std::cout << "Finished Setting Up Maps Renderer" << std::endl;

  bot_viewer_add_renderer(viewer, &self->mRenderer, priority);
}
