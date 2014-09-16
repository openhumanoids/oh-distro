#include <jpl-tags/fiducial_stereo.h>
#include <opencv2/opencv.hpp>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/multisense/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/tag_detection_t.hpp>
#include <lcmtypes/drc/tag_detection_list_t.hpp>

#include <bot_core/camtrans.h>
#include <bot_param/param_util.h>

#include <zlib.h>

struct State {
  drc::BotWrapper::Ptr mBotWrapper;
  drc::LcmWrapper::Ptr mLcmWrapper;
  BotCamTrans* mCamTransLeft;
  BotCamTrans* mCamTransRight;
  fiducial_detector_t* mDetector;
  fiducial_stereo_t* mStereoDetector;
  double mStereoBaseline;
  Eigen::Matrix3d mCalibLeft;
  Eigen::Matrix3d mCalibLeftInv;

  std::string mCameraChannel;
  std::string mTagChannel;
  bool mRunStereoAlgorithm;

  State() {
    mDetector = NULL;
    mStereoDetector = NULL;
    mCamTransLeft = NULL;
    mCamTransRight = NULL;
    mCameraChannel = "CAMERA";
    mTagChannel = "JPL_TAGS";
    mRunStereoAlgorithm = true;
  }

  ~State() {
    if (mDetector != NULL) fiducial_detector_free(mDetector);
    if (mStereoDetector != NULL) fiducial_stereo_free(mStereoDetector);
  }

  void populate(BotCamTrans* iCam, fiducial_stereo_cam_model_t& oCam) {
    oCam.cols = bot_camtrans_get_width(iCam);
    oCam.rows = bot_camtrans_get_height(iCam);
    oCam.focal_length_x = bot_camtrans_get_focal_length_x(iCam);
    oCam.focal_length_y = bot_camtrans_get_focal_length_y(iCam);
    oCam.image_center_x = bot_camtrans_get_principal_x(iCam);
    oCam.image_center_y = bot_camtrans_get_principal_y(iCam);
  }

  void populate(BotCamTrans* iCam, Eigen::Matrix3d& oK) {
    oK(0,0) = bot_camtrans_get_focal_length_x(iCam);
    oK(1,1) = bot_camtrans_get_focal_length_y(iCam);
    oK(0,2) = bot_camtrans_get_principal_x(iCam);
    oK(1,2) = bot_camtrans_get_principal_y(iCam);
    oK(0,1) = bot_camtrans_get_skew(iCam);
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

    mDetector = fiducial_detector_alloc();
    fiducial_detector_init(mDetector);
    fiducial_params_t params;
    fiducial_detector_get_params(mDetector, &params);
    // TODO: can set parameters here if desired
    //params.search_size = 40;          // image search radius in pixels
    //params.min_viewing_angle = 20;    // angle of view of fiducial in degrees
    //params.dist_thresh = 0.05;        // max allowed distance in meters between initial and estimated fiducial positions
    fiducial_detector_set_params(mDetector, &params);

    mStereoDetector = fiducial_stereo_alloc();
    fiducial_stereo_init(mStereoDetector);

    // populate camera data
    fiducial_stereo_cam_model_t leftModel, rightModel;
    mCamTransLeft =
      bot_param_get_new_camtrans(mBotWrapper->getBotParam(),
                                 (mCameraChannel + "_LEFT").c_str());
    mCamTransRight =
      bot_param_get_new_camtrans(mBotWrapper->getBotParam(),
                                 (mCameraChannel + "_RIGHT").c_str());
    populate(mCamTransLeft, leftModel);
    populate(mCamTransRight, rightModel);
    copyTransform(Eigen::Isometry3d::Identity(), leftModel.transform);
    copyTransform(Eigen::Isometry3d::Identity(), leftModel.inv_transform);
    Eigen::Isometry3d rightToLeft;
    mBotWrapper->getTransform(mCameraChannel + "_RIGHT",
                              mCameraChannel + "_LEFT", rightToLeft);
    copyTransform(rightToLeft, rightModel.transform);
    copyTransform(rightToLeft.inverse(), rightModel.inv_transform);

    // set detector camera models
    fiducial_detector_set_camera_models(mDetector, &leftModel);
    fiducial_stereo_set_camera_models(mStereoDetector, &leftModel, &rightModel);

    // set internal values for stereo depth computation
    populate(mCamTransLeft, mCalibLeft);
    mCalibLeftInv = mCalibLeft.inverse();
    mStereoBaseline = rightToLeft.translation().norm();
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }


  void onCamera(const lcm::ReceiveBuffer* iBuffer, const std::string& iChannel,
                const multisense::images_t* iMessage) {
    std::cout << "GOT IMAGE " << std::endl;

    // grab camera pose
    Eigen::Isometry3d cameraToLocal, localToCamera;
    mBotWrapper->getTransform(mCameraChannel + "_LEFT", "local", cameraToLocal,
                              iMessage->utime);
    localToCamera = cameraToLocal.inverse();

    bot_core::image_t* leftImage = NULL;
    bot_core::image_t* rightImage = NULL;
    bot_core::image_t* dispImage = NULL;

    // grab image pointers
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

    cv::Mat left, right, disp;

    // decode left image
    left = leftImage->pixelformat == leftImage->PIXEL_FORMAT_MJPEG ?
      cv::imdecode(cv::Mat(leftImage->data), -1) :
      cv::Mat(leftImage->height, leftImage->width, CV_8UC1,
              leftImage->data.data());
    if (left.channels() < 3) cv::cvtColor(left, left, CV_GRAY2RGB);

    // decode right image if necessary for stereo algorithm
    if (mRunStereoAlgorithm) {
      if (rightImage != NULL) {
        right = rightImage->pixelformat == rightImage->PIXEL_FORMAT_MJPEG ?
          cv::imdecode(cv::Mat(rightImage->data), -1) :
          cv::Mat(rightImage->height, rightImage->width, CV_8UC1,
                  rightImage->data.data());
        if (right.channels() < 3) cv::cvtColor(right, right, CV_GRAY2RGB);
      }
      else {
        std::cout << "error: no right image available!" << std::endl;
        return;
      }
    }

    // otherwise decode disparity; we will compute depth from left image only
    else {
      int numPix = dispImage->width*dispImage->height;
      if ((int)dispImage->data.size() != numPix*2) {
        std::vector<uint8_t> buf(numPix*2);
        unsigned long len = buf.size();
        uncompress(buf.data(), &len, dispImage->data.data(),
                   dispImage->data.size());
        disp = cv::Mat(dispImage->height, dispImage->width,
                       CV_16UC1, buf.data());
      }
      else {
        disp = cv::Mat(dispImage->height, dispImage->width, CV_16UC1,
                       dispImage->data.data());
      }
    }

    // set up tag detections message
    drc::tag_detection_list_t msg;
    msg.utime = iMessage->utime;
    msg.num_detections = 0;

    // TODO: initialize with number of tags from config
    int numTags = 1;
    for (int i = 0; i < numTags; ++i) {

      // TODO: populate from config id
      int tagId = i;

      fiducial_pose_t poseInit, poseFinal;
      // TODO: populate initial pose wrt left cam using fk and config location
      // can also initialize with previous pose (tracking)
      poseInit = fiducial_pose_ident();
      poseInit.pos.z = 1;
      poseInit.rot = fiducial_rot_from_rpy(0,2*M_PI/2,0);

      fiducial_detector_error_t status;
      if (mRunStereoAlgorithm) {
        float leftScore(0), rightScore(0);
        status = fiducial_stereo_process(mStereoDetector,
                                         left.data, right.data,
                                         left.cols, left.rows, left.channels(),
                                         poseInit, &poseFinal,
                                         &leftScore, &rightScore,
                                         false);
      }
      else {
        float score = 0;
        status =
          fiducial_detector_match_subpixel(mDetector,
                                           left.data, left.cols, left.rows,
                                           left.channels(), poseInit, &score);
        if (status == FIDUCIAL_DETECTOR_OK) {
          float x = mDetector->fiducial_location.x;
          float y = mDetector->fiducial_location.y;
          int xInt(x), yInt(y);
          if ((xInt < 0) || (yInt < 0) ||
              (xInt >= disp.cols-1) || (yInt >= disp.rows-1)) {
            status = FIDUCIAL_DETECTOR_ERR;
          }
          else {
            // interpolate disparity
            float xFrac(x-xInt), yFrac(y-yInt);
            int d00 = disp.at<uint16_t>(xInt, yInt);
            int d10 = disp.at<uint16_t>(xInt+1, yInt);
            int d01 = disp.at<uint16_t>(xInt, yInt+1);
            int d11 = disp.at<uint16_t>(xInt+1, yInt+1);
            float d = (1-xFrac)*(1-yFrac)*d00 + xFrac*(1-yFrac)*d10 +
              (1-xFrac)*yFrac*d01 + xFrac*yFrac*d11;

            // compute depth and 3d point
            float z = mStereoBaseline*mCalibLeft(0,0) / d;
            Eigen::Vector3d pos = z*mCalibLeftInv*Eigen::Vector3d(x,y,1);
            poseFinal.pos.x = pos[0];
            poseFinal.pos.y = pos[1];
            poseFinal.pos.z = pos[2];
            // TODO: what about orientation?
          }
        }
      }

      if (status == FIDUCIAL_DETECTOR_OK) {
        // populate this tag detection message
        drc::tag_detection_t detection;
        detection.utime = msg.utime;
        detection.id = tagId;

        // put tag into local frame
        // TODO: keep in camera frame?
        Eigen::Vector3d pos(poseFinal.pos.x, poseFinal.pos.y, poseFinal.pos.z);
        pos = cameraToLocal*pos;
        for (int k = 0; k < 3; ++k) detection.pos[k] = pos[k];
        Eigen::Quaterniond q(poseFinal.rot.u, poseFinal.rot.x,
                             poseFinal.rot.y, poseFinal.rot.z);
        q = cameraToLocal.rotation()*q;
        detection.orientation[0] = q.w();
        detection.orientation[1] = q.x();
        detection.orientation[2] = q.y();
        detection.orientation[3] = q.z();

        // find pixel position of tag
        double p[] = {poseFinal.pos.x, poseFinal.pos.y, poseFinal.pos.z};
        double pix[3];
        bot_camtrans_project_point(mCamTransLeft, p, pix);
        detection.cxy[0] = pix[0];
        detection.cxy[1] = pix[1];

        // four corners are irrelevant, set them all to tag center
        for (int k = 0; k < 4; ++k) {
          for (int m = 0; m < 2; ++m) detection.p[k][m] = pix[m];
        }

        // add this detection to list
        msg.detections.push_back(detection);
        msg.num_detections = msg.detections.size();


        std::cout << "PROCESSED " << status << std::endl;
        std::cout << "POSE: " << poseFinal.pos.x << " " << poseFinal.pos.y <<
          " " << poseFinal.pos.z << std::endl;
        
      }
      else {
        std::cout << "warning: could not detect tag " << tagId << std::endl;
      }
    }
    mLcmWrapper->get()->publish(mTagChannel, &msg);
  }

};


int main(const int iArgc, const char** iArgv) {
  State state;

  state.setup();
  state.start();
  
  return -1;
}
