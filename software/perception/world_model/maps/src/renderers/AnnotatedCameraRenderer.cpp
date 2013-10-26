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
#include <gtkmm-renderer/RendererBase.hpp>

#include <maps/BotWrapper.hpp>

#include <bot_lcmgl_client/lcmgl.h>

namespace maps {

class AnnotatedCameraRenderer : public gtkmm::RendererBase {
protected:
  enum ImagePlacement {
    ImagePlacementHide,
    ImagePlacementTopLeft,
    ImagePlacementTopCenter,
    ImagePlacementTopRight,
  };

  struct CameraState {
    AnnotatedCameraRenderer* mRenderer;
    std::string mChannel;
    std::string mName;
    std::string mCoordFrame;
    BotWrapper::Ptr mBotWrapper;
    BotCamTrans* mCamTrans;
    lcm::Subscription* mSubscription;
    Eigen::Isometry3f mPose;
    bot_core::image_t mImage;
    int mPlacement;
    bool mUpright;
    double mScale;

    GLuint mTextureId;
    bool mTextureValid;
    bool mTextureInit;

    GLdouble mModelViewGl[16];
    GLdouble mProjectionGl[16];
    GLint mViewportGl[4];

    typedef std::shared_ptr<CameraState> Ptr;


    CameraState(const std::string& iChannel,
                AnnotatedCameraRenderer* iRenderer) {
      mRenderer = iRenderer;
      mChannel = iChannel + "LEFT";
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
      mUpright = false;
      mScale = 0.3;
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
      float vpHeight = viewport[3] - viewport[1];
      float aspect = mImage.width / (float)mImage.height;
      float width, height;
      if (vpWidth/aspect > vpHeight) {
        height = vpHeight*mScale;
        width = height*aspect;
      }
      else {
        width = vpWidth*mScale;
        height = width/aspect;
      }

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

      // flip to upright if necessary
      if (mUpright) {
        Eigen::Isometry3f localToCamera = mPose.inverse();
        double theta = atan2(localToCamera(1,2), localToCamera(0,2));
        theta = -theta*180/acos(-1) - 90;
        glTranslatef(width/2, height/2, 0);
        glRotatef(theta, 0, 0, 1);
        glTranslatef(-width/2, -height/2, 0);
      }

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

      glPopMatrix();

      // restore state
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
    }
  };

  struct Annotation {
    int mWhichCamera;  // invalid = -1
    Eigen::Vector2f mDragPoint1;
    Eigen::Vector2f mDragPoint2;
    bool mDragging;
    Eigen::Vector3f mColor;
    bool mValid;

    Annotation() {
      mWhichCamera = -1;
      mDragging = false;
      mColor << 0,0,0;
      mValid = false;
    }

    bool mousePress(const Eigen::Vector2f& iClickPoint) {
      mValid = false;
      mDragPoint1 = mDragPoint2 = iClickPoint;
      mDragging = true;
      return true;
    }

    bool mouseMotion(const Eigen::Vector2f& iCurPoint) {
      if (!mDragging) return false;
      mDragPoint2 = iCurPoint;
      return true;
    }

    bool mouseRelease(const Eigen::Vector2f& iCurPoint) {
      if (!mDragging) return false;
      if ((mDragPoint1-mDragPoint2).norm() > 5) mValid = true;
      mDragging = false;
      return true;
    }

    void setColor(const Eigen::Vector3f& iColor) {
      mColor = iColor;
    }

    void draw() {
      if (!mDragging && !mValid) return;

      // set some state
      GLint viewport[4];
      glGetIntegerv(GL_VIEWPORT, viewport);
      //float vpWidth = viewport[2]-viewport[0];
      float vpHeight = viewport[3]-viewport[1];
      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(viewport[0], viewport[2], viewport[1], viewport[3]);
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glLoadIdentity();
      glEnable(GL_DEPTH_TEST);

      // TODO: just draw in mouse coords?
      // TODO: check gluOrtho and height subtraction logic
      glColor3f(mColor[0], mColor[1], mColor[2]);
      glPointSize(5);
      glLineWidth(3);
      glBegin(GL_POINTS);
      glVertex3f(mDragPoint1[0], vpHeight-mDragPoint1[1],1);
      glEnd();
      glBegin(GL_LINES);
      glVertex3f(mDragPoint1[0], vpHeight-mDragPoint1[1],1);
      glVertex3f(mDragPoint2[0], vpHeight-mDragPoint2[1],1);
      glEnd();

      // reset state
      // TODO: stack underflow here?
      glMatrixMode(GL_MODELVIEW);
      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
    }
  };

  struct Primitive {
    std::string mName;
    Annotation mAnnotation1;
    Annotation mAnnotation2;
    Eigen::Vector3f mOrigin;
    Eigen::Vector3f mDirection;  // has length
    Eigen::Vector3f mColor;
    float mWidth;
    bool mValid;
    Gtk::HBox* mBox;
    Gtk::Container* mParent;
    bool mVisible;
    Gtk::ToggleButton* mVisibleToggle;
    AnnotatedCameraRenderer* mRenderer;

    typedef std::shared_ptr<Primitive> Ptr;

    Primitive(AnnotatedCameraRenderer* iRenderer) {
      mRenderer = iRenderer;
      mName = "Unnamed";
      mColor << 0,0,0;
      mWidth = 2;
      mValid = false;
      mVisible = false;
      mBox = Gtk::manage(new Gtk::HBox());
      mParent = NULL;
    }

    ~Primitive() {
      if (mParent != NULL) {
        mParent->remove(*mBox);
        mParent->show_all();
      }
    }

    Gtk::Container* setupWidgets(Gtk::Box* iContainer) {
      mVisibleToggle = Gtk::manage(new Gtk::ToggleButton("             "));
      Gdk::Color color;
      color.set_rgb_p(mColor[0], mColor[1], mColor[2]);
      mVisibleToggle->modify_bg(Gtk::STATE_ACTIVE, color);
      mVisibleToggle->signal_toggled().connect
        (sigc::mem_fun(*this, &Primitive::onToggleVisible));
      mVisibleToggle->set_active(true);
      mBox->pack_start(*mVisibleToggle, false, false);
      Gtk::Label* label = Gtk::manage(new Gtk::Label(mName, Gtk::ALIGN_LEFT));
      mBox->pack_start(*label, false, false);
      Gtk::Button* button = Gtk::manage(new Gtk::Button("X"));
      button->signal_clicked().connect
        (sigc::mem_fun(*this, &Primitive::onDelete));
      mBox->pack_start(*button, false, false);
      iContainer->pack_start(*mBox,false,false);
      mParent = iContainer;
      return mBox;
    }

    void onToggleVisible() {
      mVisible = mVisibleToggle->get_active();
      mRenderer->requestDraw();
    }

    void onDelete() {
      if (mRenderer != NULL) {
        for (auto iter = mRenderer->mPrimitives.begin();
             iter != mRenderer->mPrimitives.end(); ++iter) {
          auto primitive = *iter;
          if (primitive.get() == this) {
            mRenderer->mPrimitives.erase(iter);
            return;
          }            
        }
      }
    }

    void draw() {
      if (!mVisible || !mValid) return;
      glColor3f(mColor[0], mColor[1], mColor[2]);
      glPointSize(3);
      glLineWidth(mWidth);
      glBegin(GL_POINTS);
      glVertex3fv(mOrigin.data());
      glEnd();
      glBegin(GL_LINES);
      Eigen::Vector3f endPoint = mOrigin + mDirection;
      glVertex3fv(mOrigin.data());
      glVertex3fv(endPoint.data());
      glEnd();
    }
  };

protected:
  BotWrapper::Ptr mBotWrapper;
  std::vector<CameraState::Ptr> mCameraStates;
  int mActiveCamera;
  bool mInteracting;
  bool mDragging;
  std::list<Primitive::Ptr> mPrimitives;
  Gtk::VBox* mPrimitivesBox;
  Gtk::ToggleButton* mAddNewPrimitiveToggle;
  Annotation mAnnotation1;
  Annotation mAnnotation2;
  std::vector<Eigen::Vector3f> mColorList;
  int mNextColorId;

  bot_lcmgl_t* mLcmGl;
  
public:

  AnnotatedCameraRenderer(BotViewer* iViewer, const int iPriority,
                          const lcm_t* iLcm,
                          const BotParam* iParam, const BotFrames* iFrames)
    : gtkmm::RendererBase("Annotated Camera", iViewer, iPriority,
                          iLcm, iParam, iFrames) {

    // set up bot objects
    drc::Clock::instance()->setLcm(getLcm());
    drc::Clock::instance()->setVerbose(false);
    mBotWrapper.reset(new BotWrapper(getLcm(), getBotParam(), getBotFrames()));

    // create and show ui widgets
    setupWidgets();

    // initialize internal variables
    mActiveCamera = -1;
    mInteracting = false;
    mAnnotation1.setColor(Eigen::Vector3f(1,0,0));
    mAnnotation2.setColor(Eigen::Vector3f(0,1,0));
    mNextColorId = 0;
    mColorList.resize(8);
    mColorList[0] << 1,0,0;
    mColorList[1] << 0,1,0;
    mColorList[2] << 0,0,1;
    mColorList[3] << 1,1,0;
    mColorList[4] << 1,0,1;
    mColorList[5] << 0,1,1;
    mColorList[6] << 1,0.5,0;
    mColorList[7] << 0,1,0.5;

    mLcmGl = bot_lcmgl_init(mBotWrapper->getLcm()->getUnderlyingLCM(),
                            "image-annotation");
  }

  ~AnnotatedCameraRenderer() {
    bot_lcmgl_destroy(mLcmGl);
  }

  void setupWidgets() {
    Gtk::Container* container = getGtkContainer();
    Gtk::Notebook* notebook = Gtk::manage(new Gtk::Notebook());

    Gtk::VBox* page = Gtk::manage(new Gtk::VBox());
    addCameraWidget("CAMERA", page);
    addCameraWidget("CAMERA_LHAND", page);
    addCameraWidget("CAMERA_RHAND", page);
    notebook->append_page(*page, "Cameras");

    page = Gtk::manage(new Gtk::VBox());
    addPrimitivesWidget(page);
    notebook->append_page(*page, "Primitives");

    container->add(*notebook);
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

  void addCameraWidget(const std::string& iCameraName,
                       Gtk::Container* iContainer) {
    CameraState::Ptr cam(new CameraState(iCameraName, this));
    mCameraStates.push_back(cam);

    Gtk::VBox* vbox = Gtk::manage(new Gtk::VBox());
    Gtk::Label* label = Gtk::manage(new Gtk::Label(iCameraName,
                                                   Gtk::ALIGN_LEFT));
    vbox->pack_start(*label,false,false);
    std::vector<int> ids = { ImagePlacementHide, ImagePlacementTopLeft,
                             ImagePlacementTopCenter, ImagePlacementTopRight };
    std::vector<std::string> labels = { "Hide", "Top Left", "Top Center",
                                        "Top Right" };
    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());
    label = Gtk::manage(new Gtk::Label("Where",Gtk::ALIGN_LEFT));
    hbox->add(*label);
    Gtk::ComboBox* combo = createCombo(labels, ids);
    bind(combo, iCameraName + ".Where", cam->mPlacement);
    combo->set_active(cam->mPlacement);
    hbox->add(*combo);
    vbox->pack_start(*hbox,false,false);

    hbox = Gtk::manage(new Gtk::HBox());
    label = Gtk::manage(new Gtk::Label("Scale", Gtk::ALIGN_LEFT));
    hbox->add(*label);
    Gtk::HScale* slider = Gtk::manage(new Gtk::HScale(0.1, 1.0, 0.01));
    slider->set_digits(2);
    slider->set_value_pos(Gtk::POS_LEFT);
    bind(slider, iCameraName + ".Scale", cam->mScale);
    slider->set_value(cam->mScale);
    hbox->add(*slider);
    vbox->pack_start(*hbox,false,false);

    hbox = Gtk::manage(new Gtk::HBox());
    Gtk::CheckButton* check = Gtk::manage(new Gtk::CheckButton("Upright"));
    bind(check, iCameraName + ".Upright", cam->mUpright);
    check->set_active(cam->mUpright);
    hbox->add(*check);
    vbox->pack_start(*hbox,false,false);

    iContainer->add(*vbox);
  }

  void addPrimitivesWidget(Gtk::Container* iContainer) {
    mPrimitivesBox = Gtk::manage(new Gtk::VBox());
    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());

    Gdk::Color color;
    color.set_rgb_p(0.8,1.0,0.8);
    mAddNewPrimitiveToggle = Gtk::manage(new Gtk::ToggleButton("Add New"));
    mAddNewPrimitiveToggle->set_active(false);
    mAddNewPrimitiveToggle->modify_bg(Gtk::STATE_ACTIVE, color);
    mAddNewPrimitiveToggle->modify_bg(Gtk::STATE_PRELIGHT, color);
    mAddNewPrimitiveToggle->signal_toggled().connect
      (sigc::mem_fun(*this, &AnnotatedCameraRenderer::onAddNewPrimitive));
    hbox->pack_start(*mAddNewPrimitiveToggle, false, false);

    Gtk::Button* button = Gtk::manage(new Gtk::Button("Push LCMGL"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &AnnotatedCameraRenderer::onPushLines));
    hbox->pack_start(*button, false, false);

    button = Gtk::manage(new Gtk::Button("Clear All"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &AnnotatedCameraRenderer::onClearPrimitives));
    hbox->pack_start(*button, false, false);

    mPrimitivesBox->pack_start(*hbox, false, false);
    iContainer->add(*mPrimitivesBox);
    // later add primitive widgets
  }

  void onClearPrimitives() {
    mPrimitives.clear();
  }

  void onPushLines() { 
    bot_lcmgl_t* lcmgl = mLcmGl;

    for (auto iter = mPrimitives.begin(); iter != mPrimitives.end(); ++iter) {
      auto primitive = *iter;
      bot_lcmgl_color3f(lcmgl, primitive->mColor[0], primitive->mColor[1],
                        primitive->mColor[2]);
      bot_lcmgl_line_width(lcmgl, 3);
      bot_lcmgl_begin(lcmgl, LCMGL_LINES);
      Eigen::Vector3f p1 = primitive->mOrigin;
      Eigen::Vector3f p2 = p1 + primitive->mDirection;
      bot_lcmgl_vertex3f(lcmgl, p1[0], p1[1], p1[2]);
      bot_lcmgl_vertex3f(lcmgl, p2[0], p2[1], p2[2]);
      bot_lcmgl_end(lcmgl);
      bot_lcmgl_point_size(lcmgl, 4);
      bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
      bot_lcmgl_vertex3f(lcmgl, p1[0], p1[1], p1[2]);      
      bot_lcmgl_end(lcmgl);
    }

    bot_lcmgl_switch_buffer(lcmgl);
  }

  void onAddNewPrimitive() {
    mInteracting = mAddNewPrimitiveToggle->get_active();
    getBotEventHandler()->picking = mInteracting;
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
    if (!mAddNewPrimitiveToggle->get_active()) {
      getBotEventHandler()->picking = 0;
      return -1;
    }
    return 0;
  }

  bool mousePress(const GdkEventButton* iEvent,
                  const double iRayStart[3], const double iRayDir[3]) {
    if (!mInteracting) return false;
    int whichImage = whichImageHit(Eigen::Vector2f(iEvent->x, iEvent->y));
    if (whichImage < 0) return false;
    if (iEvent->button == 1) {
      mAnnotation1.mWhichCamera = whichImage;
      return mAnnotation1.mousePress(Eigen::Vector2f(iEvent->x, iEvent->y));
    }
    else if (iEvent->button == 3) {
      mAnnotation2.mWhichCamera = whichImage;
      return mAnnotation2.mousePress(Eigen::Vector2f(iEvent->x, iEvent->y));
    }
    return false;
  }

  bool mouseRelease(const GdkEventButton* iEvent,
                    const double iRayStart[3], const double iRayDir[3]) {
    if (!mInteracting) return false;
    bool handled = false;
    if (iEvent->button == 1) {
      handled =
        mAnnotation1.mouseRelease(Eigen::Vector2f(iEvent->x, iEvent->y));
    }
    else if (iEvent->button == 3) {
      handled =
        mAnnotation2.mouseRelease(Eigen::Vector2f(iEvent->x, iEvent->y));
    }
    if (handled) {
      if (mAnnotation1.mValid && mAnnotation2.mValid &&
          (mAnnotation1.mWhichCamera != mAnnotation2.mWhichCamera)) {
        Primitive::Ptr primitive(new Primitive(this));
        primitive->mAnnotation1 = mAnnotation1;
        primitive->mAnnotation2 = mAnnotation2;
        if (doIntersection(primitive)) {
          primitive->mName = "Unnamed";
          int colorId = mPrimitives.size();

          //primitive->mColor = mColorList[mNextColorId % mColorList.size()];
          primitive->mColor = mColorList[colorId];
          ++mNextColorId;
          primitive->mVisible = true;
          primitive->mValid = true;
          Gtk::Container* container = primitive->setupWidgets(mPrimitivesBox);
          mPrimitives.push_back(primitive);
          mAddNewPrimitiveToggle->set_active(false);
          mAnnotation1.mValid = mAnnotation2.mValid = false;
          container->show_all();
          requestDraw();
        }
      }
    }
    return handled;
  }

  bool mouseMotion(const GdkEventMotion* iEvent,
                   const double iRayStart[3], const double iRayDir[3]) {
    if (!mInteracting) return false;
    bool button1 = (iEvent->state & GDK_BUTTON1_MASK) != 0;
    bool button3 = (iEvent->state & GDK_BUTTON3_MASK) != 0;
    bool handled = false;
    if (button1) {
      handled = mAnnotation1.mouseMotion(Eigen::Vector2f(iEvent->x, iEvent->y));
    }
    else if (button3) {
      handled = mAnnotation2.mouseMotion(Eigen::Vector2f(iEvent->x, iEvent->y));
    }
    if (handled) {
      requestDraw();
    }
    return handled;
  }

  bool doIntersection(Primitive::Ptr& ioPrimitive) {
    // mouse coords
    Eigen::Vector2f pix[4];
    pix[0] = ioPrimitive->mAnnotation1.mDragPoint1;
    pix[1] = ioPrimitive->mAnnotation1.mDragPoint2;
    pix[2] = ioPrimitive->mAnnotation2.mDragPoint1;
    pix[3] = ioPrimitive->mAnnotation2.mDragPoint2;

    // pixel coords
    auto cam1 = mCameraStates[ioPrimitive->mAnnotation1.mWhichCamera];
    auto cam2 = mCameraStates[ioPrimitive->mAnnotation2.mWhichCamera];
    cam1->getImageCoords(pix[0], pix[0]);
    cam1->getImageCoords(pix[1], pix[1]);
    cam2->getImageCoords(pix[2], pix[2]);
    cam2->getImageCoords(pix[3], pix[3]);

    // rays in camera space
    Eigen::Vector3f rays[4];
    double ray[3];
    bot_camtrans_unproject_pixel(cam1->mCamTrans, pix[0][0], pix[0][1], ray);
    rays[0] << ray[0],ray[1],ray[2];
    bot_camtrans_unproject_pixel(cam1->mCamTrans, pix[1][0], pix[1][1], ray);
    rays[1] << ray[0],ray[1],ray[2];
    bot_camtrans_unproject_pixel(cam2->mCamTrans, pix[2][0], pix[2][1], ray);
    rays[2] << ray[0],ray[1],ray[2];
    bot_camtrans_unproject_pixel(cam2->mCamTrans, pix[3][0], pix[3][1], ray);
    rays[3] << ray[0],ray[1],ray[2];

    // rays in local frame
    rays[0] = cam1->mPose.linear()*rays[0];
    rays[1] = cam1->mPose.linear()*rays[1];
    rays[2] = cam2->mPose.linear()*rays[2];
    rays[3] = cam2->mPose.linear()*rays[3];

    // origins
    Eigen::Vector3f origin1 = cam1->mPose.translation();
    Eigen::Vector3f origin2 = cam2->mPose.translation();

    // plane of camera 2
    Eigen::Vector4f plane;
    Eigen::Vector3f planeNormal = rays[2].cross(rays[3]);
    planeNormal.normalize();
    plane.head<3>() = planeNormal;
    plane[3] = -planeNormal.dot(origin2);

    // intersect two rays of camera 1 with plane
    float t1 = -(plane[3]+planeNormal.dot(origin1)) / planeNormal.dot(rays[0]);
    //TODO if (t1 < 0) return false;
    Eigen::Vector3f p1 = origin1 + rays[0]*t1;
    float t2 = -(plane[3]+planeNormal.dot(origin1)) / planeNormal.dot(rays[1]);
    // TODO if (t2 < 0) return false;
    Eigen::Vector3f p2 = origin1 + rays[1]*t2;

    ioPrimitive->mOrigin = p1;
    ioPrimitive->mDirection = p2-p1;
    ioPrimitive->mDirection.normalize();
    ioPrimitive->mOrigin -= ioPrimitive->mDirection*1.0;
    ioPrimitive->mDirection *= 5.0;

    return true;
  }

  void draw() {
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_ALL_ATTRIB_BITS);

    // draw each camera image
    for (auto iter = mCameraStates.begin();
         iter != mCameraStates.end(); ++iter) {
      (*iter)->draw();
    }

    mAnnotation1.draw();
    mAnnotation2.draw();

    glPopClientAttrib();
    glPopAttrib();

    for (auto iter = mPrimitives.begin(); iter != mPrimitives.end(); ++iter) {
      (*iter)->draw();
    }

  }

};

}


// this is the single setup method exposed for integration with the viewer
void annotated_camera_renderer_setup(BotViewer* iViewer, const int iPriority,
                                     const lcm_t* iLcm,
                                     const BotParam* iParam,
                                     const BotFrames* iFrames) {
  new maps::AnnotatedCameraRenderer(iViewer, iPriority, iLcm, iParam, iFrames);
}
