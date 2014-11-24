#include "StereoHandler.hpp"

#include <maps/BotWrapper.hpp>
#include <maps/DepthImage.hpp>
#include <maps/DepthImageView.hpp>
#include <maps/Utils.hpp>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include <lcmtypes/drc/map_request_t.hpp>

#include <bot_core/camtrans.h>
#include <bot_param/param_util.h>

#include <opencv2/opencv.hpp>
#include <zlib.h>

#include <thread>
#include <mutex>

using namespace maps;

struct StereoHandler::Imp {
  BotWrapper::Ptr mBotWrapper;
  bot_core::image_t mCurrentDepth;
  Eigen::Isometry3d mCurrentPose;
  BotCamTrans* mCamTrans;
  Eigen::Matrix3f mCalibMatrix;
  std::string mCameraFrame;
  float mDisparityFactor;
  std::mutex mDataMutex;

  Imp(const BotWrapper::Ptr& iBotWrapper, const std::string& iCameraBaseName) {
    mBotWrapper = iBotWrapper;
    auto lcm = mBotWrapper->getLcm();
    mCurrentDepth.size = 0;
    mCurrentPose = Eigen::Isometry3d::Identity();

    std::string cameraName = iCameraBaseName + "_LEFT";
    mCamTrans = bot_param_get_new_camtrans
      (mBotWrapper->getBotParam(), cameraName.c_str());
    if (mCamTrans != NULL) {
      double k00 = bot_camtrans_get_focal_length_x(mCamTrans);
      double k11 = bot_camtrans_get_focal_length_y(mCamTrans);
      double k01 = bot_camtrans_get_skew(mCamTrans);
      double k02 = bot_camtrans_get_principal_x(mCamTrans);
      double k12 = bot_camtrans_get_principal_y(mCamTrans);
      mCalibMatrix << k00,k01,k02, 0,k11,k12, 0,0,1;
      std::string key("cameras.");
      key += (cameraName + ".coord_frame");
      char* val = NULL;
      if (bot_param_get_str(mBotWrapper->getBotParam(),key.c_str(),&val) == 0) {
        mCameraFrame = val;
        free(val);
      }

      // baseline
      Eigen::Isometry3f rightToLeft;
      double baseline = 0.04;
      if (mBotWrapper->getTransform(iCameraBaseName+"_RIGHT",
                                    iCameraBaseName+"_LEFT", rightToLeft)) {
        baseline = rightToLeft.translation().norm();
      }
      mDisparityFactor = 1/k00/baseline;

      lcm->subscribe(iCameraBaseName, &Imp::onImages, this);

      std::cout << "successfully set up stereo camera " <<
        iCameraBaseName << std::endl;
    }
    if (mCamTrans == NULL) {
      std::cout << "error: could not set up stereo camera " <<
        iCameraBaseName << std::endl;
    }
  }

  ~Imp() {
    bot_camtrans_destroy(mCamTrans);
  }

  void onImages(const lcm::ReceiveBuffer* iBuffer, const std::string& iChannel,
                const bot_core::images_t* iMessage) {
    for (int i = 0; i < iMessage->n_images; ++i) {
      if ((iMessage->image_types[i] == bot_core::images_t::DISPARITY) ||
          (iMessage->image_types[i] == bot_core::images_t::DISPARITY_ZIPPED)) {
        std::unique_lock<std::mutex> lock(mDataMutex);
        mCurrentDepth = iMessage->images[i];
        mBotWrapper->getTransform(mCameraFrame, "local", mCurrentPose,
                                  mCurrentDepth.utime);
        return;
      }
    }
  }

  bool cropPoint(const Eigen::Vector3f& iPoint,
                 const std::vector<Eigen::Vector4f>& iBoundPlanes) {
    for (size_t k = 0; k < iBoundPlanes.size(); ++k) {
      const Eigen::Vector4f& plane = iBoundPlanes[k];
      float dist = plane[0]*iPoint[0] + plane[1]*iPoint[1] + plane[2]*iPoint[2];
      if (dist < -plane[3]) return false;
    }
    return true;
  }

  bool removeSmall(cv::Mat& ioImage, const uint16_t iValueThresh,
                   const int iSizeThresh) {

    const int w = ioImage.cols;
    const int h = ioImage.rows;

    // allocate output labels image
    std::vector<int> labelImage(w*h);

    // allocate equivalences table
    std::vector<int> equivalences(w*h);
    std::fill(equivalences.begin(), equivalences.end(), 0);

    // utility function
    auto collapse = [&](const int iIndex) {
      int out = iIndex;
      while (equivalences[out] != out) out = equivalences[out];
      return out;
    };

    int curLabel = 0;

    // loop over image pixels
    for (int i = 0; i < h; ++i) {
      int* labels = labelImage.data() + i*w;
      const uint16_t* im = ioImage.ptr<uint16_t>(i);
      for (int j = 0; j < w; ++j, ++im, ++labels) {

        // ignore pixels below thresh
        if (*im < iValueThresh) continue;

        // look at neighbors (for 4-connectedness)
        int labelAbove = (i>0) ? collapse(labels[-w]) : 0;
        int labelLeft = (j>0) ? collapse(labels[-1]) : 0;

        if (labelAbove>0) {
          *labels = labelAbove;
          if ((labelLeft>0) && (labelAbove!=labelLeft)) {
            equivalences[labelLeft] = labelAbove;
          }
        }
        else {
          if (labelLeft>0) {
            *labels = labelLeft;
          }
          else {
            ++curLabel;
            *labels = curLabel;
            equivalences[curLabel] = curLabel;
          }
        }
      }
    }

    // collapse label equivalences
    ++curLabel;
    for (int i = 0; i < curLabel; ++i) {
      equivalences[i] = collapse(i);
    }

    // remap labels image and count connected components
    std::vector<int> counts(curLabel);
    std::fill(counts.begin(), counts.end(), 0);
    for (int i = 0; i < w*h; ++i) {
      int& label = labelImage[i];
      label = equivalences[label];
      ++counts[label];
    }

    // remap once more
    std::vector<uint8_t> valid(counts.size());
    for (int i = 0; i < curLabel; ++i) {
      valid[i] = (counts[i] < iSizeThresh) ? 0 : 1;
    }
    const int* labels = labelImage.data();
    for (int i = 0; i < h; ++i) {
      uint16_t* im = ioImage.ptr<uint16_t>(i);
      for (int j = 0; j < w; ++j, ++im, ++labels) {
        if (valid[*labels] == 0) *im = 0;
      }
    }

    return true;
  }


  DepthImageView::Ptr getLatestDepthImage(bool iDoFilter) {
    bot_core::image_t img;
    Eigen::Isometry3d pose;
    {
      std::unique_lock<std::mutex> lock(mDataMutex);
      img = mCurrentDepth;
      pose = mCurrentPose;
    }
    if (img.size == 0) return NULL;

    // find and convert disparity image message
    cv::Mat disparityMat;
    int w = img.width;
    int h = img.height;
    cv::Mat orig;
    std::vector<uint8_t> buf;
    if (img.data.size() != w*h*2) {
      buf.resize(w*h*2);
      unsigned long len = buf.size();
      uncompress(buf.data(), &len, img.data.data(), img.data.size());
      orig = cv::Mat(h, w, CV_16UC1, buf.data());
    }
    else {
      orig = cv::Mat(h, w, CV_16UC1, (void*)img.data.data());
    }
    if (iDoFilter) {
      float depthThresh = 10.0f;
      int sizeThresh = 100;
      removeSmall(orig, 16/mDisparityFactor/depthThresh, sizeThresh);
    }
    orig.convertTo(disparityMat, CV_32F, mDisparityFactor/16);

    // copy disparity data
    DepthImage depthImage;
    depthImage.setSize(w, h);
    depthImage.setOrthographic(false);
    depthImage.setPose(pose.cast<float>());
    depthImage.setCalib(mCalibMatrix);
    std::vector<float> dispData(w*h);
    std::copy(disparityMat.ptr<float>(), disparityMat.ptr<float>()+w*h,
              dispData.begin());
    depthImage.setData(dispData, DepthImage::TypeDisparity);

    // wrap in view and return
    DepthImageView::Ptr view(new DepthImageView());
    view->set(depthImage);
    view->setUpdateTime(img.utime);
    return view;
  }

  DepthImageView::Ptr
  getDepthImageView(const std::vector<Eigen::Vector4f>& iBoundPlanes) {
    DepthImageView::Ptr view = getLatestDepthImage(true);
    if (view == NULL) return view;

    // project bound polyhedron onto camera
    DepthImage::Ptr img = view->getDepthImage();
    int w(img->getWidth()), h(img->getHeight());
    std::vector<Eigen::Vector3f> vertices;
    std::vector<std::vector<int> > faces;
    Utils::polyhedronFromPlanes(iBoundPlanes, vertices, faces);
    Eigen::Isometry3f localToCamera = img->getPose().inverse();
    Eigen::Vector2i minPt(1000000,1000000), maxPt(-1000000,-1000000);
    for (size_t i = 0; i < vertices.size(); ++i) {
      Eigen::Vector3f pt = localToCamera*vertices[i];
      double inPt[] = { pt[0], pt[1], pt[2] };
      double proj[3];
      bot_camtrans_project_point(mCamTrans, inPt, proj);
      minPt[0] = std::min(minPt[0], (int)floor(proj[0]));
      minPt[1] = std::min(minPt[1], (int)floor(proj[1]));
      maxPt[0] = std::max(maxPt[0], (int)ceil(proj[0]));
      maxPt[1] = std::max(maxPt[1], (int)ceil(proj[1]));
    }
    minPt[0] = std::max(0, minPt[0]);
    minPt[1] = std::max(0, minPt[1]);
    maxPt[0] = std::min(w-1, maxPt[0]);
    maxPt[1] = std::min(h-1, maxPt[1]);
    if ((minPt[0] >= maxPt[0]) || (minPt[1] >= maxPt[1])) {
      std::cout << "Warning: requested volume is outside image" << std::endl;
      return NULL;
    }

    // form output depth image
    DepthImage depthImage;
    Eigen::Vector2i newSize = maxPt - minPt + Eigen::Vector2i(1,1);
    depthImage.setSize(newSize[0], newSize[1]);
    depthImage.setOrthographic(false);
    depthImage.setPose(img->getPose());
    Eigen::Matrix3f calib = mCalibMatrix;
    calib(0,2) -= minPt[0];
    calib(1,2) -= minPt[1];
    depthImage.setCalib(calib);

    // scale down if necessary
    // TODO

    // crop and copy disparity data
    cv::Mat disparityMat(h,w,CV_32FC1,
                         (void*)img->getData(DepthImage::TypeDisparity).data());
    cv::Rect bounds(minPt[0], minPt[1], newSize[0], newSize[1]);
    cv::Mat disparitySub = disparityMat(bounds);
    std::vector<float> dispData(newSize[0]*newSize[1]);
    float* outPtr = &dispData[0];
    for (int i = 0; i < newSize[1]; ++i) {
      float* inPtr = disparitySub.ptr<float>(i);
      for (int j = 0; j < newSize[0]; ++j, ++inPtr, ++outPtr) {
        float dispVal = *inPtr;
        if (dispVal > 4000) {
          *outPtr = 0;
        }
        else {
          Eigen::Vector3f pt(j,i,dispVal*mDisparityFactor);
          Eigen::Vector3f ptLocal =
            depthImage.unproject(pt, DepthImage::TypeDisparity);
          if (!cropPoint(ptLocal, iBoundPlanes)) *outPtr = 0;
          else *outPtr = pt[2];
        }
      }
    }
    depthImage.setData(dispData, DepthImage::TypeDisparity);

    // wrap in view and return
    view->set(depthImage);
    return view;
  }

  DepthImageView::Ptr getDepthImageView(const drc::map_request_t& iRequest) {
    // get latest depth map as view
    DepthImageView::Ptr depthView = getLatestDepthImage(true);
    if (depthView == NULL) return depthView;

    // convert to point cloud in local frame
    maps::PointCloud::Ptr cloud = depthView->getAsPointCloud();

    // set up transformation
    Eigen::Projective3f projector;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        projector(i,j) = iRequest.transform[i][j];
      }
    }
    Eigen::Isometry3f headToLocal, torsoToLocal;
    if (mBotWrapper->getTransform("utorso", "local", torsoToLocal) &&
        mBotWrapper->getTransform("head", "local", headToLocal)) {
      float theta = atan2(torsoToLocal(1,0), torsoToLocal(0,0));
      Eigen::Matrix3f rotation;
      rotation = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());
      headToLocal.linear() = rotation;
      projector = projector*headToLocal.inverse();
    }

    // reproject to depth map according to given specifications
    DepthImageView::Ptr view(new DepthImageView());
    view->setSize(iRequest.width, iRequest.height);
    view->getDepthImage()->setAccumulationMethod
      ((DepthImage::AccumulationMethod)iRequest.accum_type);
    view->setTransform(projector);
    view->set(cloud);

    return view;
  }


};

StereoHandler::
StereoHandler(const BotWrapper::Ptr& iBotWrapper, const std::string& iCameraBaseName) {
  mImp.reset(new Imp(iBotWrapper, iCameraBaseName));
}

StereoHandler::
~StereoHandler() {
}

bool StereoHandler::
isGood() const {
  return mImp->mCamTrans != NULL;
}

DepthImageView::Ptr StereoHandler::
getDepthImageView(const std::vector<Eigen::Vector4f>& iBoundPlanes) {
  return mImp->getDepthImageView(iBoundPlanes);
}

DepthImageView::Ptr StereoHandler::
getDepthImageView(const drc::map_request_t& iRequest) {
  return mImp->getDepthImageView(iRequest);
}
