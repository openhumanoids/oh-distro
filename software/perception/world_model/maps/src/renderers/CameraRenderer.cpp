#include <iostream>
#include <sstream>
#include <unordered_map>
#include <list>

#include <gtkmm.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm-cpp.hpp>
#include <bot_core/camtrans.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <image_utils/jpeg.h>
#include <lcmtypes/bot_core/image_t.hpp>
#include <bot_vis/viewer.h>

#include <drc_utils/Clock.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <gtkmm-renderer/RendererBase.hpp>

namespace drc {

class CameraRenderer : public gtkmm::RendererBase {
protected:
  enum ImagePlacement {
    ImagePlacementHide,
    ImagePlacementTopLeft,
    ImagePlacementTopCenter,
    ImagePlacementTopRight,
  };

  struct CameraState {
    CameraRenderer* mRenderer;
    std::string mChannel;
    std::string mName;
    std::string mCoordFrame;
    BotWrapper::Ptr mBotWrapper;
    BotCamTrans* mCamTrans;
    lcm::Subscription* mSubscription;
    Eigen::Isometry3f mPose;
    bot_core::image_t mImage;
    int mPlacement;
    double mScale;
    bool mNativeImage;
    double mZoomFactor;
    Eigen::Vector2f mImageCenter;

    GLuint mTextureId;
    bool mTextureValid;
    bool mTextureInit;

    GLdouble mModelViewGl[16];
    GLdouble mProjectionGl[16];
    GLint mViewportGl[4];

    typedef std::shared_ptr<CameraState> Ptr;


    CameraState(const std::string& iChannel,
                CameraRenderer* iRenderer) {
      mRenderer = iRenderer;
      mChannel = iChannel;
      mBotWrapper = mRenderer->mBotWrapper;
      BotParam* param = mBotWrapper->getBotParam();
      char* name =
        bot_param_get_camera_name_from_lcm_channel(param, mChannel.c_str());
      mName = name;
      free(name);
      char* coordFrame = bot_param_get_camera_coord_frame(param, mName.c_str());
      mCoordFrame = coordFrame;
      free(coordFrame);
      mCamTrans = bot_param_get_new_camtrans(param, mName.c_str());
      mSubscription =
        mBotWrapper->getLcm()->subscribe(mChannel, &CameraState::onImage, this);
      mTextureValid = false;
      mTextureInit = false;
      mImage.size = 0;
      mPlacement = ImagePlacementHide;
      mScale = 1.0/3;
      mNativeImage = true;
      mZoomFactor = 1.0;
      mImageCenter<<0,0;
    }

    ~CameraState() {
      if (mSubscription != NULL) {
        mBotWrapper->getLcm()->unsubscribe(mSubscription);
      }
      if (mCamTrans != NULL) {
        bot_camtrans_destroy(mCamTrans);
      }
    }

    void onImage(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const bot_core::image_t* iMessage) {
      mImage = *iMessage;

      // make sure it's rgb3
      if (iMessage->pixelformat == bot_core::image_t::PIXEL_FORMAT_MJPEG) {
        int stride = iMessage->width * 3;
        mImage.data.resize(iMessage->height * stride);
        mImage.size = mImage.data.size();
        mImage.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
        if (0 != jpeg_decompress_8u_rgb(&iMessage->data[0], iMessage->size,
                                        &mImage.data[0], iMessage->width,
                                        iMessage->height, stride)) {
          std::cout << "Error decoding jpeg" << std::endl;
        }
      }
      else if (iMessage->pixelformat == bot_core::image_t::PIXEL_FORMAT_GRAY) {
        int numBytes = mImage.data.size();
        std::vector<uint8_t> bytes(numBytes*3);
        uint8_t* out = bytes.data();
        for (int i = 0; i < mImage.height; ++i) {
          for (int j = 0; j < mImage.width; ++j) {
            uint8_t val = mImage.data[i*mImage.width+j];
            for (int k = 0; k < 3; ++k, ++out) {
              *out = val;
            }
          }
        }
        mImage.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
        mImage.row_stride = 3*mImage.width;
        mImage.size = bytes.size();
        mImage.data = bytes;
      }

      mBotWrapper->getTransform(mCoordFrame, "local", mPose, mImage.utime);
      mTextureValid = false;

      mRenderer->requestDraw();
    }

    bool getImageCoords(const Eigen::Vector2f& iClick,
                        Eigen::Vector2f& oCoords) {
      if (mImage.size == 0) return false;
      double x,y,z;
      gluUnProject(iClick[0], mViewportGl[3]-iClick[1], 0,
                   mModelViewGl, mProjectionGl, mViewportGl, &x,&y,&z);
      oCoords << x*mImage.width, y*mImage.height;
      return true;
    }

    void draw() {
      if (mImage.size == 0) return;
      if (mPlacement == ImagePlacementHide) return;

      if (!mTextureInit) {
        glGenTextures(1, &mTextureId);
        mTextureInit = true;
      }

      // setup
      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT, viewport);
      glMatrixMode(GL_TEXTURE);
      glPushMatrix();
      glLoadIdentity();
      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0, viewport[2], 0, viewport[3]);
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glLoadIdentity();
      glTranslatef(0, viewport[3], 0);
      glScalef(1, -1, 1);
      glEnable(GL_DEPTH_TEST);

      // set sizes
      float vpWidth = viewport[2] - viewport[0];
      //float vpHeight = viewport[3] - viewport[1];
      float aspect = mImage.width / (float)mImage.height;
      float width, height;
      width = vpWidth*mScale;
      height = width/aspect;

      // determine placement of upper left image corner
      Eigen::Vector2f corner(0,0);
      switch (mPlacement) {
      case ImagePlacementTopLeft:   corner<<0,0; break;
      case ImagePlacementTopCenter: corner<<vpWidth*(1-mScale)/2,0; break;
      case ImagePlacementTopRight:  corner<<vpWidth-width,0; break;
      default:  break;
      }
      glPushMatrix();
      glTranslatef(corner[0], corner[1], 0.99);  // TODO: better way than .99?

      // scale up to width and height
      glScalef(width, height, 1);

      // get transform for unprojection
      glGetDoublev(GL_MODELVIEW_MATRIX, mModelViewGl);
      glGetDoublev(GL_PROJECTION_MATRIX, mProjectionGl);
      glGetIntegerv(GL_VIEWPORT, mViewportGl);

      // draw texture
      glColor3f(1,1,1);
      if (!mTextureValid) {
        glBindTexture(GL_TEXTURE_2D, mTextureId);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        if (mImage.pixelformat == bot_core::image_t::PIXEL_FORMAT_RGB) {
          glTexImage2D(GL_TEXTURE_2D, 0, 3, mImage.width, mImage.height,
                       0, GL_RGB, GL_UNSIGNED_BYTE, mImage.data.data());
        }
        else {
          glTexImage2D(GL_TEXTURE_2D, 0, 1, mImage.width, mImage.height,
                       0, GL_LUMINANCE, GL_UNSIGNED_BYTE, mImage.data.data());
        }

        mTextureValid = true;
      }
      glMatrixMode(GL_TEXTURE);
      glTranslatef(-0.5*mZoomFactor,mZoomFactor-0.5*mZoomFactor,0);
      glScalef(1/mZoomFactor,1/mZoomFactor,1);
      glTranslatef(-0.5,-0.5,0);
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, mTextureId);
      glBegin(GL_QUADS);
      glTexCoord2i(0,0);
      glVertex2i(0,0);
      glTexCoord2i(0,1);
      glVertex2i(0,1);
      glTexCoord2i(1,1);
      glVertex2i(1,1);
      glTexCoord2i(1,0);
      glVertex2i(1,0);
      glEnd();
      glBindTexture (GL_TEXTURE_2D, 0);
      glDisable(GL_TEXTURE_2D);

      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();

      // restore state
      glMatrixMode(GL_TEXTURE);
      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }
  };

protected:
  BotWrapper::Ptr mBotWrapper;
  std::vector<CameraState::Ptr> mCameraStates;
  int mActiveCamera;
  bool mDragging;
  double mScaleBase;
  double mZoomBase;
  Eigen::Vector2f mDragPoint1;
  Eigen::Vector2f mDragPoint2;
  
public:

  CameraRenderer(BotViewer* iViewer, const int iPriority,
                 const lcm_t* iLcm,
                 const BotParam* iParam, const BotFrames* iFrames)
    : gtkmm::RendererBase("Atlas Cameras", iViewer, iPriority,
                          iLcm, iParam, iFrames) {

    // set up bot objects
    drc::Clock::instance()->setLcm(getLcm());
    drc::Clock::instance()->setVerbose(false);
    mBotWrapper.reset(new BotWrapper(getLcm(), getBotParam(), getBotFrames()));

    // create and show ui widgets
    setupWidgets();

    // initialize internal variables
    mActiveCamera = -1;
  }

  ~CameraRenderer() {
  }

  void setupWidgets() {
    Gtk::Container* container = getGtkContainer();

    CameraState::Ptr cam;
    cam.reset(new CameraState("CAMERA_LEFT", this));
    cam->mPlacement = ImagePlacementTopCenter;
    mCameraStates.push_back(cam);

    cam.reset(new CameraState("CAMERACHEST_LEFT", this));
    cam->mPlacement = ImagePlacementTopLeft;
    mCameraStates.push_back(cam);

    cam.reset(new CameraState("CAMERACHEST_RIGHT", this));
    cam->mPlacement = ImagePlacementTopRight;
    mCameraStates.push_back(cam);

    container->show_all();
  }

  Gtk::ComboBox* createCombo(std::vector<std::string>& iLabels,
                             std::vector<int>& iIndices) {
    struct ComboColumns : public Gtk::TreeModel::ColumnRecord {
      Gtk::TreeModelColumn<int> mId;
      Gtk::TreeModelColumn<Glib::ustring> mLabel;
      ComboColumns() { add(mId); add(mLabel); }
    };
    ComboColumns columns;
    Glib::RefPtr<Gtk::ListStore> treeModel = Gtk::ListStore::create(columns);
    for (size_t i = 0; i < iIndices.size(); ++i) {
      const Gtk::TreeModel::Row& row = *(treeModel->append());
      row[columns.mId] = iIndices[i];
      row[columns.mLabel] = iLabels[i];
    }
    Gtk::ComboBox* combo = Gtk::manage(new Gtk::ComboBox());
    combo->set_model(treeModel);
    combo->pack_start(columns.mLabel);
    return combo;
  }

  int whichImageHit(const Eigen::Vector2f& iPt) {
    int idx = 0;
    int bestImage = -1;
    float minDistance = 1e10;
    for (auto iter = mCameraStates.begin();
         iter != mCameraStates.end(); ++iter, ++idx) {
      auto cam = *iter;
      Eigen::Vector2f imagePoint;
      if (!cam->getImageCoords(iPt, imagePoint)) continue;
      if ((imagePoint[0] >= 0) && (imagePoint[0] <= cam->mImage.width-1) &&
          (imagePoint[1] >= 0) && (imagePoint[1] <= cam->mImage.height-1)) {
        float distance = (imagePoint - Eigen::Vector2f(0.5,0.5)).norm();
        if (distance < minDistance) {
          minDistance = distance;
          bestImage = idx;
        }
      }
    }
    return bestImage;
  }

  double pickQuery(const double iRayStart[3], const double iRayDir[3]) {
    /* TODO: how to enable?
    int whichImage = whichImageHit(Eigen::Vector2f(iEvent->x, iEvent->y));
    getBotEventHandler()->picking = 0;
    return -1;
    */
    return 0;
  }

  bool mousePress(const GdkEventButton* iEvent,
                  const double iRayStart[3], const double iRayDir[3]) {
    if (mDragging) return false;

    int whichImage = whichImageHit(Eigen::Vector2f(iEvent->x, iEvent->y));
    if (whichImage < 0) return false;

    mActiveCamera = whichImage;
    auto cam = mCameraStates[whichImage];
    mDragging = true;
    mDragPoint1 << iEvent->x,iEvent->y;
    mDragPoint2 = mDragPoint1;
    if (iEvent->button == 2) {
      mScaleBase = cam->mScale;
    }
    if (iEvent->button == 3) {
      mZoomBase = cam->mZoomFactor;
    }
    return true;
  }

  bool mouseRelease(const GdkEventButton* iEvent,
                    const double iRayStart[3], const double iRayDir[3]) {
    mDragging = false;
    mActiveCamera = -1;
    return true;
  }

  bool mouseMotion(const GdkEventMotion* iEvent,
                   const double iRayStart[3], const double iRayDir[3]) {
    if (mActiveCamera < 0) {
      return false;
    }
    bool button1 = (iEvent->state & GDK_BUTTON1_MASK) != 0;
    bool button2 = (iEvent->state & GDK_BUTTON2_MASK) != 0;
    bool button3 = (iEvent->state & GDK_BUTTON3_MASK) != 0;
    mDragPoint2 << iEvent->x,iEvent->y;
    auto cam = mCameraStates[mActiveCamera];
    if (button1) {
    }
    if (button2) {
      double dist = mDragPoint2[1] - mDragPoint1[1];
      double scale = pow(2,4*dist/cam->mImage.height);
      cam->mScale = std::min(std::max(mScaleBase*scale, 0.1),10.0);
    }
    if (button3) {
      double dist = mDragPoint2[1] - mDragPoint1[1];
      double scale = pow(2,4*dist/cam->mImage.height);
      cam->mZoomFactor = std::min(std::max(mZoomBase*scale, 1.0),10.0);
    }
    requestDraw();
    return true;
  }

  void draw() {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_ALL_ATTRIB_BITS);

    // draw each camera image
    // TODO: depth ordering according to which one is active
    for (auto iter = mCameraStates.begin();
         iter != mCameraStates.end(); ++iter) {
      (*iter)->draw();
    }

    glPopClientAttrib();
    glPopAttrib();

  }

};

}


// this is the single setup method exposed for integration with the viewer
void camera_renderer_setup(BotViewer* iViewer, const int iPriority,
                           const lcm_t* iLcm,
                           const BotParam* iParam,
                           const BotFrames* iFrames) {
  new drc::CameraRenderer(iViewer, iPriority, iLcm, iParam, iFrames);
}
