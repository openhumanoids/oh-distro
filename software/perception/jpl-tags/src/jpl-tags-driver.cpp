#include <jpl-tags/fiducial_stereo.h>
#include <opencv2/opencv.hpp>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/multisense/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>

#include <bot_core/camtrans.h>
#include <bot_param/param_util.h>


struct State {
  drc::BotWrapper::Ptr mBotWrapper;
  drc::LcmWrapper::Ptr mLcmWrapper;
  fiducial_stereo_t* mDetector;

  std::string mCameraChannel;

  State() {
    mDetector = NULL;
    mCameraChannel = "CAMERA";
  }

  ~State() {
    if (mDetector != NULL) fiducial_stereo_free(mDetector);
  }

  void populate(BotCamTrans* iCam, fiducial_stereo_cam_model_t& oCam) {
    oCam.cols = bot_camtrans_get_width(iCam);
    oCam.rows = bot_camtrans_get_height(iCam);
    oCam.focal_length_x = bot_camtrans_get_focal_length_x(iCam);
    oCam.focal_length_y = bot_camtrans_get_focal_length_y(iCam);
    oCam.image_center_x = bot_camtrans_get_principal_x(iCam);
    oCam.image_center_y = bot_camtrans_get_principal_y(iCam);
  }

  void copyTransform(const Eigen::Isometry3d& iTransform,
                     double oTransform[4][4]) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        oTransform[i][j] = iTransform(i,j);
      }
    }
  }

  void setup() {
    mBotWrapper.reset(new drc::BotWrapper());
    mLcmWrapper.reset(new drc::LcmWrapper(mBotWrapper->getLcm()));

    mLcmWrapper->get()->subscribe(mCameraChannel, &State::onCamera, this);

    mDetector = fiducial_stereo_alloc();
    fiducial_stereo_init(mDetector);

    // populate stereo camera data
    fiducial_stereo_cam_model_t leftModel, rightModel;
    BotCamTrans* leftCam =
      bot_param_get_new_camtrans(mBotWrapper->getBotParam(),
                                 (mCameraChannel + "_LEFT").c_str());
    BotCamTrans* rightCam =
      bot_param_get_new_camtrans(mBotWrapper->getBotParam(),
                                 (mCameraChannel + "_RIGHT").c_str());
    populate(leftCam, leftModel);
    populate(rightCam, rightModel);
    copyTransform(Eigen::Isometry3d::Identity(), leftModel.transform);
    copyTransform(Eigen::Isometry3d::Identity(), leftModel.inv_transform);
    Eigen::Isometry3d rightToLeft;
    mBotWrapper->getTransform(mCameraChannel + "_RIGHT",
                              mCameraChannel + "_LEFT", rightToLeft);
    copyTransform(rightToLeft, rightModel.transform);
    copyTransform(rightToLeft.inverse(), rightModel.inv_transform);
    fiducial_stereo_set_camera_models(mDetector, &leftModel, &rightModel);
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }


  void onCamera(const lcm::ReceiveBuffer* iBuffer, const std::string& iChannel,
                const multisense::images_t* iMessage) {
    std::cout << "GOT IMAGE " << std::endl;
    bot_core::image_t* leftImage = NULL;
    bot_core::image_t* rightImage = NULL;
    bot_core::image_t* dispImage = NULL;

    for (int i = 0; i < iMessage->n_images; ++i) {
      switch (iMessage->image_types[i]) {
      case multisense::images_t::LEFT:
        leftImage = (bot_core::image_t*)(&iMessage->images[i]);
        break;
      case multisense::images_t::RIGHT:
        rightImage = (bot_core::image_t*)(&iMessage->images[i]);
        break;
      case multisense::images_t::DISPARITY_ZIPPED:
      case multisense::images_t::DISPARITY:
        dispImage = (bot_core::image_t*)(&iMessage->images[i]);
        break;
      default: break;
      }
    }

    cv::Mat left = leftImage->pixelformat == leftImage->PIXEL_FORMAT_MJPEG ?
      cv::imdecode(cv::Mat(leftImage->data), -1) :
      cv::Mat(leftImage->height, leftImage->width, CV_8UC1,
              leftImage->data.data());
    if (left.channels() < 3) cv::cvtColor(left, left, CV_GRAY2RGB);
    cv::Mat right = rightImage->pixelformat == rightImage->PIXEL_FORMAT_MJPEG ?
      cv::imdecode(cv::Mat(rightImage->data), -1) :
      cv::Mat(rightImage->height, rightImage->width, CV_8UC1,
              rightImage->data.data());
    if (right.channels() < 3) cv::cvtColor(right, right, CV_GRAY2RGB);

    // TODO: initialize with number of tags from config
    int numTags = 1;
    for (int tag = 0; tag < numTags; ++tag) {

      float leftScore(0), rightScore(0);
      fiducial_pose_t poseInit, poseFinal;
      // TODO: populate initial pose wrt left cam using fk and config location
      // can also initialize with previous pose (tracking)
      poseInit = fiducial_pose_ident();
      poseInit.pos.z = 1;
      poseInit.rot = fiducial_rot_from_rpy(0,2*M_PI/2,0);
      auto status =
        fiducial_stereo_process(mDetector,
                                left.data, right.data,
                                left.cols, left.rows, left.channels(),
                                poseInit, &poseFinal,
                                &leftScore, &rightScore,
                                false);
      std::cout << "PROCESSED " << status << std::endl;
      std::cout << "POSE: " << poseFinal.pos.x << " " << poseFinal.pos.y <<
        " " << poseFinal.pos.z << std::endl;
      // TODO: populate message
    }
    // TODO: fill message list and publish
  }

};


int main(const int iArgc, const char** iArgv) {
  State state;

  state.setup();
  state.start();
  
  return -1;
}
