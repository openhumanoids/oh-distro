#include "MeshRenderer.hpp"

#include <boost/thread/mutex.hpp>

// image related includes
#include <lcm/lcm-cpp.hpp>
#include <bot_core/camtrans.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <image_utils/jpeg.h>
#include <image_utils/pixels.h>
#include <lcmtypes/bot_core/image_t.hpp>

#include <maps/BotFramesWrapper.hpp>

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>

using namespace maps;

struct MeshRenderer::InternalState {
  // camera data
  boost::shared_ptr<lcm::LCM> mLcm;
  std::string mCameraChannel;
  std::string mCameraFrame;
  lcm::Subscription* mCameraSubscription;
  bot_core::image_t mCameraImage; 
  BotParam* mBotParam;
  boost::shared_ptr<BotFramesWrapper> mBotFrames;
  BotCamTrans* mCamTrans;
  Eigen::Matrix4f mProjectionMatrix;

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

  // gl state
  GLuint mVertexBufferId;
  GLuint mNormalBufferId;
  GLuint mColorBufferId;
  GLuint mTexCoordBufferId;
  GLuint mFaceBufferId;
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
  boost::mutex mMutex;


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
      mCamTrans = bot_param_get_new_camtrans(mBotParam, mCameraChannel.c_str());
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
      if (bot_param_get_str(mBotParam, key.c_str(), &val) == 0) {
        mCameraFrame = val;
        free(val);
      }
    }

    mNeedsUpdate = true;
  }
};

MeshRenderer::
MeshRenderer() {
  mState.reset(new InternalState());

  mState->mCameraSubscription = NULL;
  mState->mCamTrans = NULL;
  mState->mBotParam = NULL;
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
}

MeshRenderer::
~MeshRenderer() {
  if (mState->mLcm != NULL) {
    if (mState->mCameraSubscription != NULL) {
      mState->mLcm->unsubscribe(mState->mCameraSubscription);
    }
  }
  if (mState->mCamTrans != NULL) {
    bot_camtrans_destroy(mState->mCamTrans);
  }
}

void MeshRenderer::
setLcm(const boost::shared_ptr<lcm::LCM> iLcm) {
  mState->mLcm = iLcm;
  if (mState->mBotParam == NULL) {
    mState->mBotParam =
      bot_param_get_global(mState->mLcm->getUnderlyingLCM(),0);
  }
  mState->mBotFrames.reset(new BotFramesWrapper(mState->mBotParam));
  mState->mBotFrames->setLcm(mState->mLcm);
  setCameraChannel(mState->mCameraChannel);
}

void MeshRenderer::
setBotParam(const BotParam* iBotParam) {
  mState->mBotParam = (BotParam*)iBotParam;
}

void MeshRenderer::
setCameraChannel(const std::string& iChannel) {
  mState->mCameraChannel = iChannel;
  if (mState->mCameraSubscription != NULL) {
    mState->mLcm->unsubscribe(mState->mCameraSubscription);
  }
  if (mState->mLcm != NULL) {
    mState->mCameraSubscription =
      mState->mLcm->subscribe(mState->mCameraChannel,
                              &InternalState::onCameraImage, mState.get());
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
  boost::mutex::scoped_lock lock(mState->mMutex);

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
  boost::mutex::scoped_lock lock(mState->mMutex);
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
    mState->mFirstDraw = false;
  }

  // create vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, mState->mVertexBufferId);
  glBufferData(GL_ARRAY_BUFFER, mState->mVertexBuffer.size()*sizeof(float),
               &mState->mVertexBuffer[0], GL_STATIC_DRAW);
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, 0, 0);

  // create normal buffer
  if (mState->mNormalBuffer.size() > 0) {
    glBindBuffer(GL_ARRAY_BUFFER, mState->mNormalBufferId);
    glBufferData(GL_ARRAY_BUFFER, mState->mNormalBuffer.size()*sizeof(float),
                 &mState->mNormalBuffer[0], GL_STATIC_DRAW);
    glEnableClientState(GL_NORMAL_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, 0);
  }

  // create color buffer
  if ((mState->mColorMode != ColorModeCamera) &&
      (mState->mColorMode != ColorModeTexture)) {
    int numVertices = mState->mColorBuffer.size()/4;
    Eigen::Vector3f color = mState->mColor;
    float invDenom1 = 1/(mState->mMaxZ - mState->mMinZ);
    float invDenom2 = 1/(mState->mScaleMaxZ - mState->mScaleMinZ);
    float rangeMin(1e10), rangeMax(-1e10), invRangeDenom(1);
    std::vector<float> ranges;
    if (mState->mColorMode == ColorModeRange) {
      ranges.resize(numVertices);
      for (int i = 0; i < numVertices; ++i) {
        Eigen::Vector3f pt(mState->mVertexBuffer[3*i+0],
                           mState->mVertexBuffer[3*i+1],
                           mState->mVertexBuffer[3*i+2]);
        float range = (pt - mState->mRangeOrigin).norm();
        rangeMin = std::min(rangeMin, range);
        rangeMax = std::max(rangeMax, range);
        ranges[i] = range;
      }
      invRangeDenom = 1/(rangeMax-rangeMin);
    }
    for (int i = 0; i < numVertices; ++i) {
      if (mState->mColorMode == ColorModeHeight) {
        float z = (mState->mVertexBuffer[3*i+2] - mState->mMinZ) * invDenom1;
        float w = (z-mState->mScaleMinZ) * invDenom2;
        float* col = bot_color_util_jet(w);
        color = Eigen::Vector3f(col[0], col[1], col[2]);
      }
      if (mState->mColorMode == ColorModeRange) {
        float z = (ranges[i] - rangeMin) * invRangeDenom;
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

  else if (mState->mCameraImage.size) {
    // texture coordinates transformation
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glScalef(1.0/mState->mCameraImage.width,
             1.0/mState->mCameraImage.height,1);
    glMultMatrixf(mState->mProjectionMatrix.data());
    Eigen::Isometry3f localToCam;
    mState->mBotFrames->getTransform("local", mState->mCameraFrame,
                                     mState->mCameraImage.utime, localToCam);
    glMultMatrixf(localToCam.data());
    glMultMatrixf(mState->mTransform.data());

    // draw texture
    glColor4ub(255,255,255,255*mState->mColorAlpha);
    glBindTexture(GL_TEXTURE_2D, mState->mCameraTextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, mState->mCameraImage.width,
                 mState->mCameraImage.height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                 &mState->mCameraImage.data[0]);
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
