#include "MeshRenderer.hpp"

#include <iostream>
#include <thread>
#include <mutex>
#include <unordered_map>

// image related includes
#include <lcm/lcm-cpp.hpp>
#include <bot_core/camtrans.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <image_utils/jpeg.h>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>

#include <maps/BotWrapper.hpp>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>


using namespace maps;

struct MeshRenderer::InternalState {

  // struct to hold camera subscriptions for texture mapping
  struct CameraSubscription {
    typedef std::shared_ptr<CameraSubscription> Ptr;

    std::string mChannel;
    lcm::Subscription* mSubscription;
    lcm::Subscription* mSubscriptionMultiple;
    bool mUseMultipleImages;
    std::string mCoordFrame;
    Eigen::Isometry3f mLocalToCamera;
    bot_core::image_t mImage; 
    BotCamTrans* mCamTrans;
    Eigen::Matrix4f mProjectionMatrix;
    GLuint mTextureId;
    InternalState* mState;
    bool mIdealPinhole;
    int mImageWidth;
    int mImageHeight;

    CameraSubscription(InternalState* iState) {
      mSubscription = NULL;
      mSubscriptionMultiple = NULL;
      mUseMultipleImages = false;
      mCamTrans = NULL;
      mImage.size = 0;
      mTextureId = 0;
      mIdealPinhole = true;
      mState = iState;
    }

    ~CameraSubscription() {
      if (mState->mBotWrapper->getLcm() != NULL) {
        if (mSubscription != NULL) {
          mState->mBotWrapper->getLcm()->unsubscribe(mSubscription);
        }
        if (mSubscriptionMultiple != NULL) {
          mState->mBotWrapper->getLcm()->unsubscribe(mSubscriptionMultiple);
        }
      }
      if (mCamTrans != NULL) {
        bot_camtrans_destroy(mCamTrans);
      }
    }
  };

  struct MultiTexture {
    int mWidth;
    int mHeight;
    std::vector<int> mVerticalOffsets;
    std::vector<uint8_t> mData;
    GLuint mTextureId;
    std::vector<float> mTexCoords;
    bool mCoordsNeedUpdate;
    bool mImagesNeedUpdate;

    MultiTexture() {
      mCoordsNeedUpdate = true;
      mImagesNeedUpdate = true;
    }
  };

  BotWrapper::Ptr mBotWrapper;

  // camera data
  std::unordered_map<std::string, CameraSubscription::Ptr> mCameraSubscriptions;
  std::string mActiveCameraChannel;
  MultiTexture mMultiTexture;

  // external texture map data
  bot_core::image_t mTextureImage;
  Eigen::Projective3f mLocalToTexture;

  // appearance controls
  ColorMode mColorMode;
  MeshMode mMeshMode;
  Eigen::Vector3f mColor;
  float mColorAlpha;
  float mScaleMinZ;
  float mScaleMaxZ;
  float mPointSize;
  float mLineWidth;
  Eigen::Vector3f mRangeOrigin;
  Eigen::Vector3f mNormalZero;

  // gl state
  GLuint mVertexBufferId;
  GLuint mNormalBufferId;
  GLuint mColorBufferId;
  GLuint mTexCoordBufferId;
  GLuint mFaceBufferId;
  GLuint mGenericTextureId;
  bool mFirstDraw;

  // mesh data
  Eigen::Projective3f mTransform;
  std::vector<float> mVertexBuffer;
  std::vector<float> mNormalBuffer;
  std::vector<uint32_t> mFaceBuffer;
  std::vector<uint8_t> mColorBuffer;
  float mMinZ;
  float mMaxZ;

  bool mNeedsUpdate;
  std::mutex mMutex;


  void onCameraImages(const lcm::ReceiveBuffer* iBuf,
                      const std::string& iChannel,
                      const bot_core::images_t* iMessage) {
    for (int i = 0; i < iMessage->n_images; ++i) {
      if (iMessage->image_types[i] == bot_core::images_t::LEFT) {
        return onCameraImage(iBuf, iChannel, &(iMessage->images[i]));
      }
    }
  }

  void onCameraImage(const lcm::ReceiveBuffer* iBuf,
                     const std::string& iChannel,
                     const bot_core::image_t* iMessage) {
    auto item = mCameraSubscriptions.find(iChannel);
    if (item == mCameraSubscriptions.end()) return;
    CameraSubscription::Ptr sub = item->second;

    const bot_core::image_t& img = *iMessage;
    sub->mImage = img;
    if (img.pixelformat == bot_core::image_t::PIXEL_FORMAT_MJPEG) {
      int stride = img.width * 3;
      sub->mImage.data.resize(img.height * stride);
      sub->mImage.pixelformat = 0;
      jpeg_decompress_8u_rgb(&img.data[0], img.size, &sub->mImage.data[0],
                             img.width, img.height, stride);
    }
    else {
      // TODO: this will break unless rgb3
    }

    mBotWrapper->getTransform("local", sub->mCoordFrame,
                              sub->mLocalToCamera, sub->mImage.utime);

    mNeedsUpdate = true;
    mMultiTexture.mImagesNeedUpdate = true;
  }

  void remapPixelValues(const bot_core::image_t& iImage,
                        std::vector<uint8_t>& oBytes) {
    int width(iImage.width), height(iImage.height);
    oBytes.resize(width * height);
    float scale2 = 255/(mScaleMaxZ - mScaleMinZ);
    uint8_t* out = &(oBytes[0]);

    // grayscale case; simple remapping
    if (iImage.pixelformat == bot_core::image_t::PIXEL_FORMAT_GRAY) {
      for (int i = 0; i < height; ++i) {
        const uint8_t* in = &(iImage.data[0]) + i*iImage.row_stride;
        for (int j = 0; j < width; ++j, ++in, ++out) {
          float val = *in/255.0f;
          float mapVal = (val - mScaleMinZ)*scale2;
          if (mapVal > 1) mapVal = 1;
          else if (mapVal < 0) mapVal = 0;
          *out = (uint8_t)mapVal;
        }
      }
    }

    // float case
    else if (iImage.pixelformat ==
             bot_core::image_t::PIXEL_FORMAT_FLOAT_GRAY32) {

      // find extremal values
      float minVal(1e10), maxVal(-1e10);
      for (int i = 0; i < height; ++i) {
        float* in = (float*)(&iImage.data[0] + i*iImage.row_stride);
        for (int j = 0; j < width; ++j, ++in) {
          minVal = std::min(minVal, *in);
          maxVal = std::max(maxVal, *in);
        }
      }

      // remap
      float scale1 = 1/(maxVal-minVal);
      for (int i = 0; i < height; ++i) {
        float* in = (float*)(&iImage.data[0] + i*iImage.row_stride);
        for (int j = 0; j < width; ++j, ++in, ++out) {
          float val = (*in - minVal) * scale1;
          float mapVal = (val - mScaleMinZ) * scale2;
          if (mapVal > 255) mapVal = 255;
          else if (mapVal < 0) mapVal = 0;
          *out = (uint8_t)mapVal;
        }
      }
    }
  }

};

MeshRenderer::
MeshRenderer() {
  mState.reset(new InternalState());

  mState->mNeedsUpdate = true;  //TODO: REPLACE WITH NOTIFY?
  mState->mFirstDraw = true;

  setColorMode(ColorModeFlat);
  setMeshMode(MeshModeWireframe);
  setColor(1, 0.5, 0);
  setColorAlpha(1.0);
  setScaleRange(0, 1);
  setPointSize(3);
  setLineWidth(1);
  setRangeOrigin(Eigen::Vector3f(0,0,0));
  setNormalZero(Eigen::Vector3f(1,0,0));
}

MeshRenderer::
~MeshRenderer() {
}

void MeshRenderer::
setBotObjects(const std::shared_ptr<lcm::LCM> iLcm,
              const BotParam* iParam, const BotFrames* iFrames) {
  mState->mBotWrapper.reset(new BotWrapper(iLcm, iParam, iFrames));
  for (auto item : mState->mCameraSubscriptions) {
    addCameraChannel(item.second->mChannel, item.second->mUseMultipleImages);
  }
}

void MeshRenderer::
addCameraChannel(const std::string& iChannel, const bool iMultipleImages) {
  auto sub = InternalState::CameraSubscription::Ptr
    (new InternalState::CameraSubscription(mState.get()));
  sub->mChannel = iChannel;
  sub->mUseMultipleImages = iMultipleImages;
  if (iMultipleImages) {
    sub->mChannel += "_LEFT";
  }
  if (mState->mBotWrapper->getLcm() != NULL) {
    sub->mSubscription =
      mState->mBotWrapper->getLcm()->subscribe
      (sub->mChannel, &InternalState::onCameraImage, mState.get());
    if (iMultipleImages) {
      sub->mSubscriptionMultiple =
        mState->mBotWrapper->getLcm()->subscribe
        (iChannel, &InternalState::onCameraImages, mState.get());
    }
  }

  BotParam* botParam = mState->mBotWrapper->getBotParam();
  sub->mCamTrans = bot_param_get_new_camtrans(botParam, sub->mChannel.c_str());
  if (sub->mCamTrans == NULL) {
    std::cout << "Error: cannot get camtrans for " <<
      sub->mChannel << std::endl;
    return;
  }
  double K00 = bot_camtrans_get_focal_length_x(sub->mCamTrans);
  double K11 = bot_camtrans_get_focal_length_y(sub->mCamTrans);
  double K01 = bot_camtrans_get_skew(sub->mCamTrans);
  double K02 = bot_camtrans_get_principal_x(sub->mCamTrans);
  double K12 = bot_camtrans_get_principal_y(sub->mCamTrans);
  sub->mProjectionMatrix = Eigen::Matrix4f::Zero();
  sub->mProjectionMatrix(0,0) = K00;
  sub->mProjectionMatrix(0,1) = K01;
  sub->mProjectionMatrix(0,2) = K02;
  sub->mProjectionMatrix(1,1) = K11;
  sub->mProjectionMatrix(1,2) = K12;
  sub->mProjectionMatrix(2,3) = 1;
  sub->mProjectionMatrix(3,2) = 1;
  sub->mImageWidth = bot_camtrans_get_width(sub->mCamTrans);
  sub->mImageHeight = bot_camtrans_get_height(sub->mCamTrans);

  std::string key("cameras.");
  key += (sub->mChannel + ".coord_frame");
  char* val = NULL;
  if (bot_param_get_str(botParam, key.c_str(), &val) == 0) {
    sub->mCoordFrame = val;
    free(val);
  }

  // determine whether this is an ideal pinhole projection
  key = std::string("cameras.");
  key += (sub->mChannel + ".intrinsic_cal.distortion_model");
  val = NULL;
  if (bot_param_get_str(botParam, key.c_str(), &val) == 0) {
    if (strcmp(val, "plumb_bob") != 0) {
      sub->mIdealPinhole = false;
    }
    else {
      // TODO: check whether the distortion params are zero
    }
    free(val);
  }

  mState->mCameraSubscriptions[iChannel] = sub;

  std::vector<InternalState::CameraSubscription::Ptr> subs;
  subs.reserve(mState->mCameraSubscriptions.size());
  for (auto it : mState->mCameraSubscriptions) subs.push_back(it.second);

  // vertical offsets
  InternalState::MultiTexture& multiTex = mState->mMultiTexture;
  multiTex.mVerticalOffsets.resize(subs.size());
  multiTex.mWidth = multiTex.mHeight = 0;
  for (size_t i = 0; i < multiTex.mVerticalOffsets.size(); ++i) {
    multiTex.mVerticalOffsets[i] = multiTex.mHeight;
    multiTex.mHeight += subs[i]->mImageHeight;
    multiTex.mWidth = std::max(multiTex.mWidth, subs[i]->mImageWidth);
  }
}

void MeshRenderer::
setActiveCameraChannel(const std::string& iChannel) {
  mState->mActiveCameraChannel = iChannel;
}

void MeshRenderer::
setColorMode(const ColorMode& iMode) {
  mState->mColorMode = iMode;
  mState->mNeedsUpdate = true;
}

void MeshRenderer::
setMeshMode(const MeshMode& iMode) {
  mState->mMeshMode = iMode;
  mState->mNeedsUpdate = true;
}

void MeshRenderer::
setColor(const float iR, const float iG, const float iB) {
  mState->mColor = Eigen::Vector3f(iR, iG, iB);
  mState->mNeedsUpdate = true;
}

void MeshRenderer::
setColorAlpha(const float iAlpha) {
  mState->mColorAlpha = iAlpha;
  mState->mNeedsUpdate = true;
}

void MeshRenderer::
setScaleRange(const float iMinZ, const float iMaxZ) {
  mState->mScaleMinZ = iMinZ;
  mState->mScaleMaxZ = iMaxZ;
  mState->mNeedsUpdate = true;
}

void MeshRenderer::
setPointSize(const float iSize) {
  mState->mPointSize = iSize;
}

void MeshRenderer::
setLineWidth(const float iWidth) {
  mState->mLineWidth = iWidth;
}

void MeshRenderer::
setRangeOrigin(const Eigen::Vector3f& iOrigin) {
  mState->mRangeOrigin = iOrigin;
}

void MeshRenderer::
setNormalZero(const Eigen::Vector3f& iZero) {
  mState->mNormalZero = iZero.normalized();
}


void MeshRenderer::
setTexture(const int iWidth, const int iHeight, const int iStrideBytes,
           const int iFormat, const uint8_t* iData,
           const Eigen::Projective3f& iTransform) {
  mState->mTextureImage.width = iWidth;
  mState->mTextureImage.height = iHeight;
  mState->mTextureImage.row_stride = iStrideBytes;
  mState->mTextureImage.pixelformat = iFormat;
  mState->mTextureImage.size = iHeight*iStrideBytes;
  mState->mTextureImage.data =
    std::vector<uint8_t>(iData, iData+iHeight*iStrideBytes);
  mState->mLocalToTexture = iTransform;
  mState->mNeedsUpdate = true;
}

void MeshRenderer::
setData(const std::vector<Eigen::Vector3f>& iVertices,
        const std::vector<Eigen::Vector3i>& iFaces,
        const Eigen::Projective3f& iTransform) {
  return setData(iVertices, std::vector<Eigen::Vector3f>(), iFaces, iTransform);
}

void MeshRenderer::
setData(const std::vector<Eigen::Vector3f>& iVertices,
        const std::vector<Eigen::Vector3f>& iNormals,
        const std::vector<Eigen::Vector3i>& iFaces,
        const Eigen::Projective3f& iTransform) { 
  std::lock_guard<std::mutex> lock(mState->mMutex);

  // vertices
  mState->mVertexBuffer.resize(iVertices.size()*3);
  mState->mMinZ = 1e10;
  mState->mMaxZ = -1e10;
  for (size_t i = 0; i < iVertices.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      mState->mVertexBuffer[3*i+k] = iVertices[i][k];
    }
  }

  // normals
  mState->mNormalBuffer.resize(0);
  if (iNormals.size() == iVertices.size()) {
    mState->mNormalBuffer.resize(iNormals.size()*3);
    for (size_t i = 0; i < iNormals.size(); ++i) {
      for (int k = 0; k < 3; ++k) {
        mState->mNormalBuffer[3*i+k] = iNormals[i][k];
      }
    }
  }

  // faces
  mState->mFaceBuffer.resize(iFaces.size()*3);
  for (size_t i = 0; i < iFaces.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      mState->mFaceBuffer[3*i+k] = iFaces[i][k];
    }
  }

  // compute z bounds from vertices if there are no faces defined
  if (iFaces.size()==0) {
    for (size_t i = 0; i < iVertices.size(); ++i) {
      float z = mState->mVertexBuffer[3*i+2];
      mState->mMinZ = std::min(mState->mMinZ, z);
      mState->mMaxZ = std::max(mState->mMaxZ, z);
    }
  }    

  // compute z bounds only from vertices belonging to faces
  else {
    for (size_t i = 0; i < iFaces.size(); ++i) {
      for (int k = 0; k < 3; ++k) {
        float z = iVertices[mState->mFaceBuffer[3*i+k]][2];
        mState->mMinZ = std::min(mState->mMinZ, z);
        mState->mMaxZ = std::max(mState->mMaxZ, z);
      }
    }
  }

  // allocate color buffer (will be filled in when drawing)
  mState->mColorBuffer.resize(iVertices.size()*4);

  mState->mTransform = iTransform;
  mState->mNeedsUpdate = true;
  mState->mMultiTexture.mCoordsNeedUpdate = true;
}

void MeshRenderer::
draw() {
  std::lock_guard<std::mutex> lock(mState->mMutex);
  if (mState->mNeedsUpdate) {
    // TODO: do updates, separate method?
    mState->mNeedsUpdate = false;
  }

  // save state
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glPushClientAttrib(GL_ALL_ATTRIB_BITS);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();

  // set rendering flags
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
  glEnable (GL_RESCALE_NORMAL);
  glEnable(GL_TEXTURE_2D);
  if ((mState->mMeshMode == MeshModeFilled) ||
      (mState->mMeshMode == MeshModeShaded)) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  }
  else {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  }

  // transform
  glMatrixMode(GL_MODELVIEW);
  glMultMatrixf(mState->mTransform.data());

  // assign all buffer ids the first time through
  if (mState->mFirstDraw) {
    glGenBuffers(1, &mState->mVertexBufferId);
    glGenBuffers(1, &mState->mNormalBufferId);
    glGenBuffers(1, &mState->mColorBufferId);
    glGenBuffers(1, &mState->mTexCoordBufferId);
    glGenBuffers(1, &mState->mFaceBufferId);
    glGenTextures(1, &mState->mGenericTextureId);
    glGenTextures(1, &mState->mMultiTexture.mTextureId);
    mState->mFirstDraw = false;
  }
  for (auto item : mState->mCameraSubscriptions) {
    if (item.second->mTextureId == 0) {
      glGenTextures(1, &item.second->mTextureId);
    }
  }

  // create vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, mState->mVertexBufferId);
  glBufferData(GL_ARRAY_BUFFER, mState->mVertexBuffer.size()*sizeof(float),
               &mState->mVertexBuffer[0], GL_STATIC_DRAW);
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, 0);

  // create normal buffer
  /* TODO
  if (mState->mNormalBuffer.size() > 0) {
    glBindBuffer(GL_ARRAY_BUFFER, mState->mNormalBufferId);
    glBufferData(GL_ARRAY_BUFFER, mState->mNormalBuffer.size()*sizeof(float),
                 &mState->mNormalBuffer[0], GL_STATIC_DRAW);
    glEnableClientState(GL_NORMAL_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, 0);
  }
  */

  // determine which camera channel is active
  InternalState::CameraSubscription::Ptr sub;
  auto item = mState->mCameraSubscriptions.find(mState->mActiveCameraChannel);
  if (item != mState->mCameraSubscriptions.end()) sub = item->second;

  // create color buffer
  if (mState->mColorMode == ColorModeFlat) {
    glColor4f(mState->mColor[0], mState->mColor[1], mState->mColor[2],
              mState->mColorAlpha);
  }
  else if ((mState->mColorMode == ColorModeHeight) ||
           (mState->mColorMode == ColorModeRange) ||
           (mState->mColorMode == ColorModeNormal)) {
    int numVertices = mState->mColorBuffer.size()/4;
    Eigen::Vector3f color = mState->mColor;
    float invDenom1 = 1/(mState->mMaxZ - mState->mMinZ);
    float invDenom2 = 1/(mState->mScaleMaxZ - mState->mScaleMinZ);
    float valueMin(1e10), valueMax(-1e10), invValueDenom(1);
    std::vector<float> values;
    bool modeNormals = (mState->mColorMode == ColorModeNormal) &&
      (mState->mNormalBuffer.size() > 0);
    if (mState->mColorMode == ColorModeRange) {
      values.resize(numVertices);
      Eigen::Vector4f originHomog(0,0,0,1);
      originHomog.head<3>() = mState->mRangeOrigin;
      originHomog = mState->mTransform.inverse()*originHomog;
      Eigen::Vector3f origin = originHomog.head<3>()/originHomog[3];
      for (int i = 0; i < numVertices; ++i) {
        Eigen::Vector3f pt(mState->mVertexBuffer[3*i+0],
                           mState->mVertexBuffer[3*i+1],
                           mState->mVertexBuffer[3*i+2]);
        float range = (pt - origin).norm();
        if (range > 1e10) continue;
        valueMin = std::min(valueMin, range);
        valueMax = std::max(valueMax, range);
        values[i] = range;
      }
      invValueDenom = 1/(valueMax-valueMin);
    }
    else if (modeNormals) {
      values.resize(numVertices);
      for (int i = 0; i < numVertices; ++i) {
        Eigen::Vector3f normal(mState->mNormalBuffer[3*i+0],
                               mState->mNormalBuffer[3*i+1],
                               mState->mNormalBuffer[3*i+2]);
        float len = normal.norm();
        if (len < 0.01) continue;
        float dot = normal.dot(mState->mNormalZero)/len;
        if (dot < 0) dot = -dot;
        float angle = acos(dot);
        //valueMin = std::min(valueMin, angle);
        //valueMax = std::max(valueMax, angle);
        values[i] = angle;
      }
      valueMin = 0;
      valueMax = acos(-1)/2;
      invValueDenom = 1/(valueMax-valueMin);
    }
    for (int i = 0; i < numVertices; ++i) {
      if (mState->mColorMode == ColorModeHeight) {
        float z = (mState->mVertexBuffer[3*i+2] - mState->mMinZ) * invDenom1;
        float w = (z-mState->mScaleMinZ) * invDenom2;
        float* col = bot_color_util_jet(w);
        color = Eigen::Vector3f(col[0], col[1], col[2]);
      }
      else if ((mState->mColorMode == ColorModeRange) || modeNormals) {
        float z = (values[i] - valueMin) * invValueDenom;
        float w = (z - mState->mScaleMinZ) * invDenom2;
        float* col = bot_color_util_jet(w);
        color = Eigen::Vector3f(col[0], col[1], col[2]);
      }
      mState->mColorBuffer[4*i+0] = color[0]*255;
      mState->mColorBuffer[4*i+1] = color[1]*255;
      mState->mColorBuffer[4*i+2] = color[2]*255;
      mState->mColorBuffer[4*i+3] = mState->mColorAlpha*255;
    }
    glBindBuffer(GL_ARRAY_BUFFER, mState->mColorBufferId);
    glBufferData(GL_ARRAY_BUFFER, mState->mColorBuffer.size(),
                 &mState->mColorBuffer[0], GL_STATIC_DRAW);
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, 0);
  }

  else if ((mState->mColorMode == ColorModeCamera) &&
           (sub != NULL) && (sub->mImage.size>0)) {
    // texture coordinates transformation
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glScalef(1.0/sub->mImage.width, 1.0/sub->mImage.height,1);
    glMultMatrixf(sub->mProjectionMatrix.data());
    glMultMatrixf(sub->mLocalToCamera.data());
    glMultMatrixf(mState->mTransform.data());

    // draw texture
    GLfloat color[] = {mState->mColor[0], mState->mColor[1], mState->mColor[2],
                       mState->mColorAlpha};
    glColor4ub(255,255,255,255*mState->mColorAlpha);
    glBindTexture(GL_TEXTURE_2D, sub->mTextureId);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, sub->mImage.width, sub->mImage.height,
                 0, GL_RGB, GL_UNSIGNED_BYTE, &sub->mImage.data[0]);
    glBindBuffer(GL_ARRAY_BUFFER, mState->mTexCoordBufferId);
    glBufferData(GL_ARRAY_BUFFER, mState->mVertexBuffer.size()*sizeof(float),
                 &mState->mVertexBuffer[0], GL_STATIC_DRAW);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(3, GL_FLOAT, 0, 0);
  }

  else if (mState->mColorMode == ColorModeCamera) {
    // camera info vector
    std::vector<InternalState::CameraSubscription::Ptr> subs;
    subs.reserve(mState->mCameraSubscriptions.size());
    for (auto it : mState->mCameraSubscriptions) subs.push_back(it.second);

    // set up texture image
    InternalState::MultiTexture& multiTex = mState->mMultiTexture;
    if (multiTex.mImagesNeedUpdate) {
      int strideBytes = multiTex.mWidth*3;
      multiTex.mData.resize(strideBytes*multiTex.mHeight);
      std::fill(multiTex.mData.begin(), multiTex.mData.end(), 0);
      // TODO: fill with background color instead of black
      int outOffset = 0;
      for (size_t i = 0; i < subs.size(); ++i) {
        bot_core::image_t& img = subs[i]->mImage;
        if (img.size == 0) {
          outOffset += strideBytes*subs[i]->mImageHeight;
          continue;
        }
        int inOffset = 0;
        for (int j = 0; j < img.height; ++j) {
          memcpy(multiTex.mData.data()+outOffset, img.data.data()+inOffset,
                 img.width*3);
          outOffset += strideBytes;
          inOffset += img.row_stride;
        }
      }
      multiTex.mImagesNeedUpdate = false;
    }

    // project each point
    // TODO: optimize inner loop
    std::vector<float>& texCoords = mState->mMultiTexture.mTexCoords;
    if (multiTex.mCoordsNeedUpdate) {
      int n = mState->mVertexBuffer.size()/3;
      texCoords.resize(2*n);
      for (int i = 0; i < n; ++i) {
        Eigen::Vector4f ptRaw(0,0,0,1);
        for (int k = 0; k < 3; ++k) ptRaw[k] = mState->mVertexBuffer[3*i+k];
        Eigen::Vector4f prod = mState->mTransform*ptRaw;
        Eigen::Vector3f ptLocal = prod.head<3>()/prod[3];
        double minDist = 1e10;
        int bestIndex = -1;
        Eigen::Vector2f bestCoord(-1,-1);
        for (size_t j = 0; j < subs.size(); ++j) {
          if (subs[j]->mImage.size == 0) continue;
          Eigen::Vector3f pt = subs[j]->mLocalToCamera*ptLocal;
          double in[] = {pt[0], pt[1], pt[2]};
          double pix[3];
          if (0 == bot_camtrans_project_point(subs[j]->mCamTrans, in, pix)) {
            double dx = pix[0]-subs[j]->mImageWidth/2;
            double dy = pix[1]-subs[j]->mImageHeight/2;
            double dist = dx*dx + dy*dy;
            if (dist < minDist) {
              minDist = dist;
              bestIndex = j;
              bestCoord << pix[0],pix[1];
            }
          }
        }
        texCoords[2*i+0] = bestCoord[0];
        texCoords[2*i+1] = bestCoord[1];
        if (bestIndex >= 0) {
          texCoords[2*i+1] += multiTex.mVerticalOffsets[bestIndex];
          texCoords[2*i+0] /= multiTex.mWidth;
          texCoords[2*i+1] /= multiTex.mHeight;
        }
      }
      multiTex.mCoordsNeedUpdate = false;
    }

    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    GLfloat color[] = {mState->mColor[0], mState->mColor[1], mState->mColor[2],
                       mState->mColorAlpha};
    glColor4ub(255,255,255,255*mState->mColorAlpha);
    glBindTexture(GL_TEXTURE_2D, multiTex.mTextureId);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, multiTex.mWidth, multiTex.mHeight,
                 0, GL_RGB, GL_UNSIGNED_BYTE, multiTex.mData.data());
    glBindBuffer(GL_ARRAY_BUFFER, mState->mTexCoordBufferId);
    glBufferData(GL_ARRAY_BUFFER, texCoords.size()*sizeof(float),
                 texCoords.data(), GL_STATIC_DRAW);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, 0);
  }

  else if ((mState->mColorMode == ColorModeTexture) &&
           (mState->mTextureImage.size>0)) {
    // texture coordinates transformation
    int width = mState->mTextureImage.width;
    int height = mState->mTextureImage.height;
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glScalef(1.0/width, 1.0/height, 1);
    glMultMatrixf(mState->mLocalToTexture.data());
    glMultMatrixf(mState->mTransform.data());

    // draw texture
    GLfloat color[] = {mState->mColor[0], mState->mColor[1], mState->mColor[2],
                       mState->mColorAlpha};
    glColor4ub(255,255,255,255*mState->mColorAlpha);
    glBindTexture(GL_TEXTURE_2D, mState->mGenericTextureId);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    switch (mState->mTextureImage.pixelformat) {
    case bot_core::image_t::PIXEL_FORMAT_RGB:
      glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB,
                   GL_UNSIGNED_BYTE, &mState->mTextureImage.data[0]);
      break;
    case bot_core::image_t::PIXEL_FORMAT_GRAY:
      glTexImage2D(GL_TEXTURE_2D, 0, 1, width, height, 0, GL_LUMINANCE,
                   GL_FLOAT, &mState->mTextureImage.data[0]);
      break;
    case bot_core::image_t::PIXEL_FORMAT_FLOAT_GRAY32:
      glTexImage2D(GL_TEXTURE_2D, 0, 1, width, height, 0, GL_LUMINANCE,
                   GL_UNSIGNED_BYTE, &mState->mTextureImage.data[0]);
      break;
    default:
      std::cout << "Error: invalid texture type" << std::endl;
      break;
    }
    glBindBuffer(GL_ARRAY_BUFFER, mState->mTexCoordBufferId);
    glBufferData(GL_ARRAY_BUFFER, mState->mVertexBuffer.size()*sizeof(float),
                 &mState->mVertexBuffer[0], GL_STATIC_DRAW);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(3, GL_FLOAT, 0, 0);    
  }

  // treat texture as color map
  // TODO: this could be much more efficient
  else if ((mState->mColorMode == ColorModeMap) &&
           (mState->mTextureImage.size>0)) {
    // fill in colormap indices
    std::vector<uint8_t> bytes;
    mState->remapPixelValues(mState->mTextureImage, bytes);

    // create colormap itself
    int numColors = 256;
    std::vector<float> rMap(numColors), gMap(numColors), bMap(numColors);
    for (int i = 0; i < numColors; ++i) {
      float* jet = bot_color_util_jet(i/255.0f);
      rMap[i] = jet[0];
      gMap[i] = jet[1];
      bMap[i] = jet[2];
    }
    glPixelTransferi(GL_MAP_COLOR, GL_TRUE);
    glPixelMapfv(GL_PIXEL_MAP_I_TO_R, numColors, &rMap[0]);
    glPixelMapfv(GL_PIXEL_MAP_I_TO_G, numColors, &gMap[0]);
    glPixelMapfv(GL_PIXEL_MAP_I_TO_B, numColors, &bMap[0]);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
    // draw texture
    int width = mState->mTextureImage.width;
    int height = mState->mTextureImage.height;
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glScalef(1.0/width, 1.0/height, 1);
    glMultMatrixf(mState->mLocalToTexture.data());
    glMultMatrixf(mState->mTransform.data());
    GLfloat color[] = {mState->mColor[0], mState->mColor[1], mState->mColor[2],
                       mState->mColorAlpha};
    glColor4ub(255,255,255,255*mState->mColorAlpha);
    glBindTexture(GL_TEXTURE_2D, mState->mGenericTextureId);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_COLOR_INDEX,
                 GL_UNSIGNED_BYTE, &bytes[0]);
    glBindBuffer(GL_ARRAY_BUFFER, mState->mTexCoordBufferId);
    glBufferData(GL_ARRAY_BUFFER, mState->mVertexBuffer.size()*sizeof(float),
                 &mState->mVertexBuffer[0], GL_STATIC_DRAW);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(3, GL_FLOAT, 0, 0);
  }

  // create index buffer and draw as triangles
  if (mState->mMeshMode != MeshModePoints) {
    glLineWidth(mState->mLineWidth);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mState->mFaceBufferId);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 mState->mFaceBuffer.size()*sizeof(uint32_t),
                 &mState->mFaceBuffer[0], GL_STATIC_DRAW);
    glDrawElements(GL_TRIANGLES, mState->mFaceBuffer.size(),
                   GL_UNSIGNED_INT, 0);
  }

  // draw as points
  else {
    glPointSize(mState->mPointSize);
    glDrawArrays(GL_POINTS, 0, mState->mVertexBuffer.size()/3);
  }

  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);

  // restore state
  glMatrixMode(GL_TEXTURE);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glPopClientAttrib();
  glPopAttrib();
}
