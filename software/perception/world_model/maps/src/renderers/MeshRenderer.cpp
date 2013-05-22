#include "MeshRenderer.hpp"

#include <thread>

// image related includes
#include <lcm/lcm-cpp.hpp>
#include <bot_core/camtrans.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <image_utils/jpeg.h>
#include <lcmtypes/bot_core/image_t.hpp>

#include <maps/BotWrapper.hpp>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>

using namespace maps;

struct MeshRenderer::InternalState {
  // camera data
  std::string mCameraChannel;
  std::string mCameraFrame;
  lcm::Subscription* mCameraSubscription;
  bot_core::image_t mCameraImage; 
  Eigen::Isometry3f mLocalToCamera;

  BotWrapper::Ptr mBotWrapper;
  BotCamTrans* mCamTrans;
  Eigen::Matrix4f mProjectionMatrix;

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
  GLuint mCameraTextureId;
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


  void onCameraImage(const lcm::ReceiveBuffer* iBuf,
                     const std::string& iChannel,
                     const bot_core::image_t* iMessage) {
    mCameraImage = *iMessage;
    if (iMessage->pixelformat == bot_core::image_t::PIXEL_FORMAT_MJPEG) {
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
      mCamTrans = bot_param_get_new_camtrans
        (mBotWrapper->getBotParam(), mCameraChannel.c_str());
      if (mCamTrans == NULL) {
        std::cout << "Error: cannot get camtrans for " <<
          mCameraChannel << std::endl;
        return;
      }
      double K00 = bot_camtrans_get_focal_length_x(mCamTrans);
      double K11 = bot_camtrans_get_focal_length_y(mCamTrans);
      double K01 = bot_camtrans_get_skew(mCamTrans);
      double K02 = bot_camtrans_get_principal_x(mCamTrans);
      double K12 = bot_camtrans_get_principal_y(mCamTrans);
      mProjectionMatrix = Eigen::Matrix4f::Zero();
      mProjectionMatrix(0,0) = K00;
      mProjectionMatrix(0,1) = K01;
      mProjectionMatrix(0,2) = K02;
      mProjectionMatrix(1,1) = K11;
      mProjectionMatrix(1,2) = K12;
      mProjectionMatrix(2,3) = 1;
      mProjectionMatrix(3,2) = 1;

      std::string key("cameras.");
      key += (mCameraChannel + ".coord_frame");
      char* val = NULL;
      if (bot_param_get_str(mBotWrapper->getBotParam(),
                            key.c_str(), &val) == 0) {
        mCameraFrame = val;
        free(val);
      }
    }

    mBotWrapper->getTransform("local", mCameraFrame,
                              mLocalToCamera, mCameraImage.utime);

    mNeedsUpdate = true;
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

  mState->mCameraSubscription = NULL;
  mState->mCamTrans = NULL;
  mState->mCameraImage.size = 0;
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
  if (mState->mBotWrapper->getLcm() != NULL) {
    if (mState->mCameraSubscription != NULL) {
      mState->mBotWrapper->getLcm()->unsubscribe(mState->mCameraSubscription);
    }
  }
  if (mState->mCamTrans != NULL) {
    bot_camtrans_destroy(mState->mCamTrans);
  }
}

void MeshRenderer::
setBotObjects(const std::shared_ptr<lcm::LCM> iLcm,
              const BotParam* iParam, const BotFrames* iFrames) {
  mState->mBotWrapper.reset(new BotWrapper(iLcm, iParam, iFrames));
  setCameraChannel(mState->mCameraChannel);
}

void MeshRenderer::
setCameraChannel(const std::string& iChannel) {
  mState->mCameraChannel = iChannel;
  if (mState->mCameraSubscription != NULL) {
    mState->mBotWrapper->getLcm()->unsubscribe(mState->mCameraSubscription);
  }
  if (mState->mBotWrapper->getLcm() != NULL) {
    mState->mCameraSubscription =
      mState->mBotWrapper->getLcm()->subscribe
      (mState->mCameraChannel, &InternalState::onCameraImage, mState.get());
  }
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
  glEnable(GL_TEXTURE_2D);
  glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
  glEnable (GL_RESCALE_NORMAL);
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
    glGenTextures(1, &mState->mCameraTextureId);
    glGenTextures(1, &mState->mGenericTextureId);
    mState->mFirstDraw = false;
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
      for (int i = 0; i < numVertices; ++i) {
        Eigen::Vector3f pt(mState->mVertexBuffer[3*i+0],
                           mState->mVertexBuffer[3*i+1],
                           mState->mVertexBuffer[3*i+2]);
        float range = (pt - mState->mRangeOrigin).norm();
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
        valueMin = std::min(valueMin, angle);
        valueMax = std::max(valueMax, angle);
        values[i] = angle;
      }
      invValueDenom = 1/(valueMax-valueMin);
    }
    for (int i = 0; i < numVertices; ++i) {
      if (mState->mColorMode == ColorModeHeight) {
        float z = (mState->mVertexBuffer[3*i+2] - mState->mMinZ) * invDenom1;
        float w = (z-mState->mScaleMinZ) * invDenom2;
        float* col = bot_color_util_jet(w);
        color = Eigen::Vector3f(col[0], col[1], col[2]);
      }
      if ((mState->mColorMode == ColorModeRange) || modeNormals) {
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
           (mState->mCameraImage.size>0)) {
    // texture coordinates transformation
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glScalef(1.0/mState->mCameraImage.width,
             1.0/mState->mCameraImage.height,1);
    glMultMatrixf(mState->mProjectionMatrix.data());
    glMultMatrixf(mState->mLocalToCamera.data());
    glMultMatrixf(mState->mTransform.data());

    // draw texture
    GLfloat color[] = {mState->mColor[0], mState->mColor[1], mState->mColor[2],
                       mState->mColorAlpha};
    glColor4ub(255,255,255,255*mState->mColorAlpha);
    glBindTexture(GL_TEXTURE_2D, mState->mCameraTextureId);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, color);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, mState->mCameraImage.width,
                 mState->mCameraImage.height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 &mState->mCameraImage.data[0]);
    glBindBuffer(GL_ARRAY_BUFFER, mState->mTexCoordBufferId);
    glBufferData(GL_ARRAY_BUFFER, mState->mVertexBuffer.size()*sizeof(float),
                 &mState->mVertexBuffer[0], GL_STATIC_DRAW);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(3, GL_FLOAT, 0, 0);
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
