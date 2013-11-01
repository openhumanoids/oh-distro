#include <iostream>
#include <sstream>
#include <unordered_map>
#include <list>
#include <fstream>

#include <gtkmm.h>

#define GL_GLEXT_PROTOTYPES
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
    BotCamTrans* mVirtualCamTrans;
    lcm::Subscription* mSubscription;
    Eigen::Isometry3f mPose;
    bot_core::image_t mImage;
    int mImageWidth;
    int mImageHeight;
    int mPlacement;
    double mScale;
    bool mNativeImage;
    double mZoomFactor;
    Eigen::Vector2f mImageCenter;
    Eigen::Matrix3d mRotation;

    GLuint mTextureId;
    bool mTextureValid;
    bool mTextureInit;
    bool mTexCoordsValid;
    GLuint mVertexBufferId;
    GLuint mFaceBufferId;
    GLuint mTexCoordBufferId;
    std::vector<float> mVertexBuffer;
    std::vector<float> mTexCoordBuffer;
    std::vector<uint32_t> mFaceBuffer;

    GLdouble mModelViewGl[16];
    GLdouble mProjectionGl[16];
    GLint mViewportGl[4];

    static constexpr int kVirtualImageWidth = 200;
    static constexpr int kVirtualImageHeight = 200;
    static constexpr double kNominalFocalLengthFactor = 1.0;

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
      mImageWidth = mImageHeight = 0;
      mPlacement = ImagePlacementHide;
      mScale = 1.0/3;
      mNativeImage = true;
      mZoomFactor = 1.0;
      mImageCenter << 0,0;
      mRotation = Eigen::Matrix3d::Identity();
      mTexCoordsValid = false;
      mVirtualCamTrans = NULL;
    }

    ~CameraState() {
      if (mSubscription != NULL) {
        mBotWrapper->getLcm()->unsubscribe(mSubscription);
      }
      if (mCamTrans != NULL) {
        bot_camtrans_destroy(mCamTrans);
      }
      if (mVirtualCamTrans != NULL) {
        bot_camtrans_destroy(mVirtualCamTrans);
      }
    }

    void onImage(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const bot_core::image_t* iMessage) {
      mImage = *iMessage;
      mImageWidth = mImage.width;
      mImageHeight = mImage.height;
      if (!mNativeImage) {
        mImageWidth = kVirtualImageWidth;
        mImageHeight = kVirtualImageHeight;
      }

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

      mBotWrapper->getTransform(mCoordFrame, "local", mPose, mImage.utime);
      mTextureValid = false;

      mRenderer->requestDraw();
    }

    bool getImageBoxCoords(const Eigen::Vector2f& iClick,
                           Eigen::Vector2f& oCoords) {
      if (mImage.size == 0) return false;
      double x,y,z;
      gluUnProject(iClick[0], mViewportGl[3]-iClick[1], 0,
                   mModelViewGl, mProjectionGl, mViewportGl, &x,&y,&z);
      oCoords << x*mImageWidth, y*mImageHeight;
      return true;
    }

    bool getImageCoords(const Eigen::Vector2f& iClick,
                        Eigen::Vector2f& oCoords) {
      if (mImage.size == 0) return false;
      double x,y,z;
      gluUnProject(iClick[0], mViewportGl[3]-iClick[1], 0,
                   mModelViewGl, mProjectionGl, mViewportGl, &x,&y,&z);
      if (mNativeImage) {
        x = (x-0.5)/mZoomFactor+0.5-mImageCenter[0]/mImageWidth;
        y = (y-0.5)/mZoomFactor+0.5-mImageCenter[1]/mImageHeight;
        oCoords << x*mImageWidth, y*mImageHeight;
      }
      else {
        Eigen::Vector3d ray;
        if ((mVirtualCamTrans != NULL) &&
            (0 == bot_camtrans_unproject_pixel
             (mVirtualCamTrans, x*mImageWidth, y*mImageHeight, ray.data()))) {
          ray = mRotation*ray;
          Eigen::Vector3d pix;
          if (0 == bot_camtrans_project_point(mCamTrans, ray.data(),
                                              pix.data())) {
            oCoords << pix[0],pix[1];
          }
        }
      }
      return true;
    }

    void createVirtualCamera() {
      double focalLength =
        mImageWidth*kNominalFocalLengthFactor*mZoomFactor;
      BotDistortionObj* nullDistortion = bot_null_distortion_create();
      if (mVirtualCamTrans != NULL) bot_camtrans_destroy(mVirtualCamTrans);
      mVirtualCamTrans = bot_camtrans_new
        ("temp", mImageWidth, mImageHeight, focalLength, focalLength,
         mImageWidth/2.0, mImageHeight/2.0, 0, nullDistortion);

      // determine rotation from image center
      double cx = bot_camtrans_get_principal_x(mCamTrans);
      double cy = bot_camtrans_get_principal_y(mCamTrans);
      Eigen::Vector3d rz;
      bot_camtrans_unproject_pixel(mCamTrans, cx-mImageCenter[0],
                                   cy-mImageCenter[1], rz.data());
      Eigen::Vector3d rx = Eigen::Vector3d::UnitY().cross(rz);
      Eigen::Vector3d ry = rz.cross(rx);
      mRotation.col(0) = rx.normalized();
      mRotation.col(1) = ry.normalized();
      mRotation.col(2) = rz.normalized();
    }


    void draw() {
      if (mImage.size == 0) return;
      if (mPlacement == ImagePlacementHide) return;

      if (!mTextureInit) {
        glGenTextures(1, &mTextureId);
        glGenBuffers(1, &mVertexBufferId);
        glGenBuffers(1, &mTexCoordBufferId);
        glGenBuffers(1, &mFaceBufferId);
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
      float aspect = mImageWidth / (float)mImageHeight;
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
      glBindTexture(GL_TEXTURE_2D, mTextureId);
      if (!mTextureValid) {
        GLfloat color[] = {0,0,1};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
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
      if (mNativeImage) {
        glMatrixMode(GL_TEXTURE);
        glTranslatef(-mImageCenter[0]/mImageWidth,
                     -mImageCenter[1]/mImageHeight,0);
        glTranslatef(0.5,0.5,0);
        glScalef(1/mZoomFactor,1/mZoomFactor,1);
        glTranslatef(-0.5,-0.5,0);
        glEnable(GL_TEXTURE_2D);
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
      }
      else {
        drawWarpedTexture();
      }
      glBindTexture(GL_TEXTURE_2D, 0);
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

    void drawWarpedTexture() {
      if (!mTexCoordsValid) {
        createVirtualCamera();
        mTexCoordsValid = true;        
        mVertexBuffer.resize(mImageWidth*mImageHeight*2);
        mTexCoordBuffer.resize(mImageWidth*mImageHeight*2);
        std::fill(mTexCoordBuffer.begin(), mTexCoordBuffer.end(), -1);
        Eigen::Vector3d ray;
        for (int i = 0, pos = 0; i < mImageHeight; ++i) {
          for (int j = 0; j < mImageWidth; ++j, ++pos) {
            mVertexBuffer[2*pos+0] = (double)j/mImageWidth;
            mVertexBuffer[2*pos+1] = (double)i/mImageHeight;
            if (0 != bot_camtrans_unproject_pixel(mVirtualCamTrans, j, i,
                                                  ray.data())) continue;
            ray = mRotation*ray;
            Eigen::Vector3d pix;
            if (0 != bot_camtrans_project_point(mCamTrans, ray.data(),
                                                pix.data())) continue;
            if (pix[2] < 0) continue;
            mTexCoordBuffer[2*pos+0] = pix[0]/mImage.width;
            mTexCoordBuffer[2*pos+1] = pix[1]/mImage.height;
          }
        }

        // create triangles
        mFaceBuffer.resize((mImageWidth-1)*(mImageHeight-1)*6);
        for (int i = 0; i < mImageHeight-1; ++i) {
          for (int j = 0; j < mImageWidth-1; ++j) {
            int faceInd = i*(mImageWidth-1)+j;
            int vertexInd = i*mImageWidth+j;
            mFaceBuffer[6*faceInd+0] = vertexInd;
            mFaceBuffer[6*faceInd+1] = vertexInd + mImageWidth + 1;
            mFaceBuffer[6*faceInd+2] = vertexInd + 1;
            mFaceBuffer[6*faceInd+3] = vertexInd;
            mFaceBuffer[6*faceInd+4] = vertexInd + mImageWidth;
            mFaceBuffer[6*faceInd+5] = vertexInd + mImageWidth + 1;
          }
        }
      }

      // attributes
      glPushAttrib(GL_ALL_ATTRIB_BITS);
      glPushClientAttrib(GL_ALL_ATTRIB_BITS);
      glEnable(GL_TEXTURE_2D);

      // set up vertices
      glBindBuffer(GL_ARRAY_BUFFER, mVertexBufferId);
      glBufferData(GL_ARRAY_BUFFER, mVertexBuffer.size()*sizeof(float),
                   mVertexBuffer.data(), GL_STATIC_DRAW);
      glEnableClientState(GL_VERTEX_ARRAY);
      glVertexPointer(2, GL_FLOAT, 0, 0);

      // set up texture coordinates
      glBindBuffer(GL_ARRAY_BUFFER, mTexCoordBufferId);
      glBufferData(GL_ARRAY_BUFFER, mTexCoordBuffer.size()*sizeof(float),
                   mTexCoordBuffer.data(), GL_STATIC_DRAW);
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);
      glTexCoordPointer(2, GL_FLOAT, 0, 0);

      // set up triangles
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mFaceBufferId);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                   mFaceBuffer.size()*sizeof(uint32_t),
                   mFaceBuffer.data(), GL_STATIC_DRAW);

      // draw
      glDrawElements(GL_TRIANGLES, mFaceBuffer.size(), GL_UNSIGNED_INT, 0);

      // clean up
      glDisableClientState(GL_COLOR_ARRAY);
      glDisableClientState(GL_VERTEX_ARRAY);
      glDisableClientState(GL_TEXTURE_COORD_ARRAY);
      glPopClientAttrib();
      glPopAttrib();
    }

  };

protected:
  BotWrapper::Ptr mBotWrapper;
  std::vector<CameraState::Ptr> mCameraStates;
  int mActiveCamera;
  bool mDragging;
  double mScaleBase;
  double mZoomBase;
  Eigen::Vector2f mTranslateBase;
  Eigen::Vector2f mDragPoint1;
  Eigen::Vector2f mDragPoint2;
  
  GLdouble mModelViewMainGl[16];
  GLdouble mProjectionMainGl[16];
  GLint mViewportMainGl[4];

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
    cam->mNativeImage = false;
    mCameraStates.push_back(cam);

    cam.reset(new CameraState("CAMERACHEST_RIGHT", this));
    cam->mPlacement = ImagePlacementTopRight;
    cam->mNativeImage = false;
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
      if (!cam->getImageBoxCoords(iPt, imagePoint)) continue;
      if ((imagePoint[0] >= 0) && (imagePoint[0] <= cam->mImageWidth-1) &&
          (imagePoint[1] >= 0) && (imagePoint[1] <= cam->mImageHeight-1)) {
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
    Eigen::Vector3d pt3d(iRayStart[0]+iRayDir[0], iRayStart[1]+iRayDir[1],
                         iRayStart[2]+iRayDir[2]);
    double x, y, z;
    gluProject(pt3d[0], pt3d[1], pt3d[2], mModelViewMainGl, mProjectionMainGl,
               mViewportMainGl, &x, &y, &z);
    y = mViewportMainGl[3]-y;
    int whichImage = whichImageHit(Eigen::Vector2f(x, y));
    if (whichImage>=0) {
      getBotEventHandler()->picking = 1;
      return 0;
    }
    else {
      getBotEventHandler()->picking = 0;
      return -1;
    }
  }

  bool mousePress(const GdkEventButton* iEvent,
                  const double iRayStart[3], const double iRayDir[3]) {
    int whichImage = whichImageHit(Eigen::Vector2f(iEvent->x, iEvent->y));
    if (whichImage < 0) return false;
    auto cam = mCameraStates[whichImage];

    // triple click (TODO: ignore double click)
    if (iEvent->type == GDK_3BUTTON_PRESS) {
      double cx = bot_camtrans_get_principal_x(cam->mCamTrans);
      double cy = bot_camtrans_get_principal_y(cam->mCamTrans);
      Eigen::Vector2f pt(0,0);
      if (cam->getImageCoords(mDragPoint1, pt)) {
        cam->mImageCenter << cx-pt[0],cy-pt[1];
      }
    }

    // double click
    if (iEvent->type == GDK_2BUTTON_PRESS) {
      switch (iEvent->button) {
      case 1: cam->mImageCenter << 0,0; break;
      case 2: cam->mScale = 1.0/3; break;
      case 3: cam->mZoomFactor = 1; break;
      default: return false;
      }
      cam->mTexCoordsValid = false;
      requestDraw();
      return true;
    }

    // click and hold
    if (mDragging) return false;
    mActiveCamera = whichImage;
    mDragging = true;
    mDragPoint1 << iEvent->x,iEvent->y;
    mDragPoint2 = mDragPoint1;
    if (iEvent->button == 1) {
      mTranslateBase = cam->mImageCenter;
    }
    if (iEvent->button == 2) {
      mScaleBase = cam->mScale;
    }
    if (iEvent->button == 3) {
      mZoomBase = cam->mZoomFactor;
    } 
    cam->mTexCoordsValid = false;
    requestDraw();
    return true;
  }

  bool mouseRelease(const GdkEventButton* iEvent,
                    const double iRayStart[3], const double iRayDir[3]) {
    mDragging = false;
    mActiveCamera = -1;
    getBotEventHandler()->picking = 0;
    requestDraw();
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
    int viewPortSize = abs(cam->mViewportGl[3]-cam->mViewportGl[2]);
    if (button1) {
      Eigen::Vector2f p1(0,0), p2(0,0);
      if (!cam->getImageCoords(mDragPoint1, p1)) return false;
      if (!cam->getImageCoords(mDragPoint2, p2)) return false;
      Eigen::Vector2f offset = p2-p1;
      cam->mImageCenter = mTranslateBase + offset;
    }
    if (button2) {
      double dist = mDragPoint2[1] - mDragPoint1[1];
      double scale = pow(2,dist/viewPortSize);
      cam->mScale = std::min(std::max(mScaleBase*scale, 0.1),10.0);
    }
    if (button3) {
      double dist = mDragPoint2[1] - mDragPoint1[1];
      double scale = pow(2,dist/viewPortSize);
      cam->mZoomFactor = std::min(std::max(mZoomBase*scale, 0.1),10.0);
    }
    if (button1 || button2 || button3) cam->mTexCoordsValid = false;
    requestDraw();
    return true;
  }

  void draw() {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_ALL_ATTRIB_BITS);

    glGetDoublev(GL_MODELVIEW_MATRIX, mModelViewMainGl);
    glGetDoublev(GL_PROJECTION_MATRIX, mProjectionMainGl);
    glGetIntegerv(GL_VIEWPORT, mViewportMainGl);

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
void atlas_camera_renderer_setup(BotViewer* iViewer, const int iPriority,
                                 const lcm_t* iLcm,
                                 const BotParam* iParam,
                                 const BotFrames* iFrames) {
  new drc::CameraRenderer(iViewer, iPriority, iLcm, iParam, iFrames);
}
