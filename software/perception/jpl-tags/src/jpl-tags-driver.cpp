#include <jpl-tags/fiducial_stereo.h>
#include <opencv2/opencv.hpp>

#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcm/lcm-cpp.hpp>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <model-client/model-client.hpp>

#include <lcmtypes/bot_core/robot_state_t.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <lcmtypes/drc/tag_detection_t.hpp>
#include <lcmtypes/drc/tag_detection_list_t.hpp>

#include <bot_core/camtrans.h>
#include <bot_param/param_util.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <zlib.h>

#include <chrono>
#include <mutex>

#include <ConciseArgs>


struct TagInfo {
  int64_t mId;                  // unique id
  std::string mLinkName;        // link coordinate frame name
  Eigen::Isometry3d mLinkPose;  // with respect to link
  Eigen::Isometry3d mCurPose;   // tracked pose in local frame
  bool mTracked;
  TagInfo() {
    mId = 0;
    mLinkPose = Eigen::Isometry3d::Identity();
    mCurPose = Eigen::Isometry3d::Identity();
    mTracked = false;
  }
};

typedef std::map<std::string, KDL::Frame> FrameMap;
typedef std::shared_ptr<FrameMap> FrameMapPtr;
struct FrameMapInfo {
  int64_t mTimestamp;
  FrameMapPtr mFrameMap;
};

struct State {
  drc::BotWrapper::Ptr mBotWrapper;
  drc::LcmWrapper::Ptr mLcmWrapper;
  bot_lcmgl_t* mBotLcmgl;
  BotCamTrans* mCamTransLeft;
  BotCamTrans* mCamTransRight;
  fiducial_detector_t* mDetector;
  fiducial_stereo_t* mStereoDetector;
  double mStereoBaseline;
  Eigen::Matrix3d mCalibLeft;
  Eigen::Matrix3d mCalibLeftInv;
  std::vector<TagInfo> mTags;

  int64_t mLastStateUpdateTime;
  std::list<FrameMapInfo> mLinkFrames;
  std::shared_ptr<KDL::TreeFkSolverPosFull_recursive> mFkSolver;
  std::mutex mFramesMutex;

  std::string mCameraChannel;
  std::string mTagChannel;
  bool mRunStereoAlgorithm;
  bool mDoTracking;
  bool mDebug;

  static constexpr double kMaxStateUpdateRate = 60;  // Hz
  static constexpr double kMaxStateCacheSeconds = 1;


  State() {
    mDetector = NULL;
    mStereoDetector = NULL;
    mCamTransLeft = NULL;
    mCamTransRight = NULL;
    mCameraChannel = "CAMERA";
    mTagChannel = "JPL_TAGS";
    mRunStereoAlgorithm = false;
    mDoTracking = false;
    mDebug = false;
    mLastStateUpdateTime = 0;
    mBotLcmgl = NULL;
  }

  ~State() {
    if (mDetector != NULL) fiducial_detector_free(mDetector);
    if (mStereoDetector != NULL) fiducial_stereo_free(mStereoDetector);
    if (mBotLcmgl != NULL) bot_lcmgl_destroy(mBotLcmgl);
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
    oK = Eigen::Matrix3d::Identity();
    oK(0,0) = bot_camtrans_get_focal_length_x(iCam);
    oK(1,1) = bot_camtrans_get_focal_length_y(iCam);
    oK(0,2) = bot_camtrans_get_principal_x(iCam);
    oK(1,2) = bot_camtrans_get_principal_y(iCam);
    oK(0,1) = bot_camtrans_get_skew(iCam);
  }

  Eigen::Isometry3d toEigen(const KDL::Frame& iFrame) {
    Eigen::Isometry3d xform = Eigen::Isometry3d::Identity();
    xform.translation() << iFrame.p[0], iFrame.p[1], iFrame.p[2];
    Eigen::Quaterniond q;
    iFrame.M.GetQuaternion(q.x(), q.y(), q.z(), q.w());
    xform.linear() = q.matrix();
    return xform;
  }

  fiducial_pose_t getFiducialPose(const Eigen::Isometry3d& iPose) {
    fiducial_pose_t pose = fiducial_pose_ident();
    pose.pos.x = iPose.translation()[0];
    pose.pos.y = iPose.translation()[1];
    pose.pos.z = iPose.translation()[2];
    Eigen::Quaterniond q(iPose.rotation());
    pose.rot.u = q.w();
    pose.rot.x = q.x();
    pose.rot.y = q.y();
    pose.rot.z = q.z();
    return pose;
  }


  Eigen::Matrix3d rpyToMatrix(const double iR, const double iP,
                              const double iY) {
    Eigen::Matrix3d rotation;
    rotation =
      Eigen::AngleAxisd(iY*M_PI/180, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(iP*M_PI/180, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(iR*M_PI/180, Eigen::Vector3d::UnitX());
    return rotation;
  }

  void copyTransform(const Eigen::Isometry3d& iTransform,
                     double oTransform[4][4]) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        oTransform[i][j] = iTransform(i,j);
      }
    }
  }

  void debugDrawPoint(const Eigen::Vector3f& iPoint,
                      const Eigen::Vector3f& iColor,
                      const float iPointSize) {
    bot_lcmgl_point_size(mBotLcmgl, iPointSize);
    bot_lcmgl_color3f(mBotLcmgl, iColor[0], iColor[1], iColor[2]);
    bot_lcmgl_begin(mBotLcmgl, LCMGL_POINTS);
    bot_lcmgl_vertex3f(mBotLcmgl, iPoint[0], iPoint[1], iPoint[2]);
    bot_lcmgl_end(mBotLcmgl);
  }

  void debugDrawLine(const Eigen::Vector3f& iPoint1,
                     const Eigen::Vector3f& iPoint2,
                     const Eigen::Vector3f& iColor,
                     const float iLineWidth) {
    bot_lcmgl_line_width(mBotLcmgl, iLineWidth);
    bot_lcmgl_color3f(mBotLcmgl, iColor[0], iColor[1], iColor[2]);
    bot_lcmgl_begin(mBotLcmgl, LCMGL_LINES);
    bot_lcmgl_vertex3f(mBotLcmgl, iPoint1[0], iPoint1[1], iPoint1[2]);
    bot_lcmgl_vertex3f(mBotLcmgl, iPoint2[0], iPoint2[1], iPoint2[2]);
    bot_lcmgl_end(mBotLcmgl);
  }

  void setup() {
    mBotWrapper.reset(new drc::BotWrapper());
    mLcmWrapper.reset(new drc::LcmWrapper(mBotWrapper->getLcm()));
    if (mBotLcmgl != NULL) bot_lcmgl_destroy(mBotLcmgl);
    mBotLcmgl = bot_lcmgl_init(mBotWrapper->getLcm()->getUnderlyingLCM(),
                               "jpl-tag-debug");

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

    // populate tag data from config
    mTags.clear();
    std::string keyBase = "perception.fiducial_tags";
    auto tagNames = mBotWrapper->getKeys(keyBase);
    for (int i = 0; i < (int)tagNames.size(); ++i) {
      std::string key = keyBase + "." + tagNames[i];
      TagInfo tag;
      tag.mId = mBotWrapper->getInt(key + ".id");
      tag.mLinkName = mBotWrapper->get(key + ".link");
      auto xyzrpy = mBotWrapper->getDoubles(key + ".xyzrpy");
      tag.mLinkPose.translation() << xyzrpy[0], xyzrpy[1], xyzrpy[2];
      tag.mLinkPose.linear() = rpyToMatrix(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
      mTags.push_back(tag);
    }

    // grab robot model
    ModelClient modelClient(mBotWrapper->getLcm()->getUnderlyingLCM(), 0);
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(modelClient.getURDFString(), tree)) {
      std::cout << "error: cannot parse urdf" << std::endl;
    }
    mFkSolver.reset(new KDL::TreeFkSolverPosFull_recursive(tree));

    // subscriptions
    mLcmWrapper->get()->subscribe(mCameraChannel, &State::onCamera, this);
    mLcmWrapper->get()->subscribe("EST_ROBOT_STATE", &State::onState, this);
  }

  void start() {
    mLcmWrapper->startHandleThread(true);
  }

  void onState(const lcm::ReceiveBuffer* iBuffer, const std::string& iChannel,
               const bot_core::robot_state_t* iMessage) {

    // limit update rate
    int64_t timeDelta = std::abs(iMessage->utime-mLastStateUpdateTime);
    if (timeDelta < 1e6/kMaxStateUpdateRate) return;

    // get pelvis pose
    KDL::Frame bodyToLocal = KDL::Frame::Identity();
    if (false) {  // TODO TEMP
      const auto& t = iMessage->pose.translation;
      bodyToLocal.p = KDL::Vector(t.x, t.y, t.z);
      const auto& q = iMessage->pose.rotation;
      bodyToLocal.M = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);
    }
    else {
      Eigen::Isometry3d bodyToLocalBotFrames;
      mBotWrapper->getTransform("body", "local", bodyToLocalBotFrames,
                                iMessage->utime);
      const auto& t = bodyToLocalBotFrames.translation();
      bodyToLocal.p = KDL::Vector(t[0], t[1], t[2]);
      Eigen::Quaterniond q(bodyToLocalBotFrames.rotation());
      bodyToLocal.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());
    }

    // do fk
    std::map<std::string, double> joints;
    for (int i = 0; i < (int)iMessage->num_joints; ++i) {
      joints.insert(std::make_pair(iMessage->joint_name[i],
                                   iMessage->joint_position[i]));
    }
    FrameMapPtr frameMap(new FrameMap());
    if (0 != mFkSolver->JntToCart(joints, *frameMap, true)) {
      std::cout << "error: cannot perform fk" << std::endl;
      return;
    }

    // transform to local frame
    for (auto& frame : *frameMap) frame.second = bodyToLocal*frame.second;

    // update internal frames handle
    {
      FrameMapInfo info;
      info.mTimestamp = iMessage->utime;
      info.mFrameMap = frameMap;

      std::unique_lock<std::mutex> lock(mFramesMutex);
      mLinkFrames.push_back(info);
      while ((int)mLinkFrames.size() >
             (int)(kMaxStateCacheSeconds*kMaxStateUpdateRate)) {
        mLinkFrames.pop_front();
      }
      mLastStateUpdateTime = iMessage->utime;
    }
  }


  bool decodeImages(const bot_core::images_t* iMessage,
                    cv::Mat& oLeft, cv::Mat& oRight, cv::Mat& oDisp) {

    bot_core::image_t* leftImage = NULL;
    bot_core::image_t* rightImage = NULL;
    bot_core::image_t* dispImage = NULL;

    // grab image pointers
    for (int i = 0; i < iMessage->n_images; ++i) {
      switch (iMessage->image_types[i]) {
      case bot_core::images_t::LEFT:
        leftImage = (bot_core::image_t*)(&iMessage->images[i]);
        break;
      case bot_core::images_t::RIGHT:
        rightImage = (bot_core::image_t*)(&iMessage->images[i]);
        break;
      case bot_core::images_t::DISPARITY_ZIPPED:
      case bot_core::images_t::DISPARITY:
        dispImage = (bot_core::image_t*)(&iMessage->images[i]);
        break;
      default: break;
      }
    }

    // decode left image
    if (leftImage == NULL) {
      std::cout << "error: no left image available!" << std::endl;
      return false;
    }
    oLeft = leftImage->pixelformat == leftImage->PIXEL_FORMAT_MJPEG ?
      cv::imdecode(cv::Mat(leftImage->data), -1) :
      cv::Mat(leftImage->height, leftImage->width, CV_8UC1,
              leftImage->data.data());
    if (oLeft.channels() < 3) cv::cvtColor(oLeft, oLeft, CV_GRAY2RGB);

    // decode right image if necessary for stereo algorithm
    if (mRunStereoAlgorithm) {
      if (rightImage == NULL) {
        std::cout << "error: no right image available!" << std::endl;
        return false;
      }
      oRight = rightImage->pixelformat == rightImage->PIXEL_FORMAT_MJPEG ?
        cv::imdecode(cv::Mat(rightImage->data), -1) :
        cv::Mat(rightImage->height, rightImage->width, CV_8UC1,
                rightImage->data.data());
      if (oRight.channels() < 3) cv::cvtColor(oRight, oRight, CV_GRAY2RGB);
    }

    // otherwise decode disparity; we will compute depth from left image only
    else {
      if (dispImage == NULL) {
        std::cout << "error: no disparity image available!" << std::endl;
        return false;
      }
      int numPix = dispImage->width*dispImage->height;
      if ((int)dispImage->data.size() != numPix*2) {
        std::vector<uint8_t> buf(numPix*2);
        unsigned long len = buf.size();
        uncompress(buf.data(), &len, dispImage->data.data(),
                   dispImage->data.size());
        oDisp = cv::Mat(dispImage->height, dispImage->width,
                        CV_16UC1, buf.data()).clone();
      }
      else {
        oDisp = cv::Mat(dispImage->height, dispImage->width, CV_16UC1,
                        dispImage->data.data()).clone();
      }
    }
    return true;
  }

  void onCamera(const lcm::ReceiveBuffer* iBuffer, const std::string& iChannel,
                const bot_core::images_t* iMessage) {

    auto t1 = std::chrono::high_resolution_clock::now();

    // grab camera pose
    Eigen::Isometry3d cameraToLocal, localToCamera;
    mBotWrapper->getTransform(mCameraChannel + "_LEFT", "local", cameraToLocal,
                              iMessage->utime);
    localToCamera = cameraToLocal.inverse();

    // decode images
    cv::Mat left, right, disp;
    if (!decodeImages(iMessage, left, right, disp)) return;

    // set up tag detections message
    drc::tag_detection_list_t msg;
    msg.utime = iMessage->utime;
    msg.num_detections = 0;

    for (int i = 0; i < (int)mTags.size(); ++i) {

      // initialize pose
      fiducial_pose_t poseInit, poseFinal;
      poseInit = fiducial_pose_ident();

      // use previous tracked position if available
      if (mDoTracking && mTags[i].mTracked) {
        Eigen::Isometry3d pose = localToCamera*mTags[i].mCurPose;
        poseInit = getFiducialPose(pose);
      }

      // use fk wrt left cam
      else {
        std::unique_lock<std::mutex> lock(mFramesMutex);

        // search for best link map
        int64_t bestDelta = 1000000000;
        FrameMapInfo bestInfo;
        for (const auto& info : mLinkFrames) {
          int64_t timeDelta = std::abs(info.mTimestamp - iMessage->utime);
          if (timeDelta < bestDelta) {
            bestDelta = timeDelta;
            bestInfo = info;
          }
        }
        if (bestDelta > 2*1e6/kMaxStateUpdateRate) {
          std::cout << "error: cannot find robot state at " <<
            iMessage->utime << std::endl;
          return;
        }

        // find link
        const auto& frameMap = *bestInfo.mFrameMap;
        auto link = frameMap.find(mTags[i].mLinkName);
        if (link == frameMap.end()) {
          std::cout << "error: cannot find link " << mTags[i].mLinkName <<
            std::endl;
        }

        // compute transforms
        Eigen::Isometry3d linkToLocal = toEigen(link->second);
        Eigen::Isometry3d linkToCamera = localToCamera*linkToLocal;
        Eigen::Isometry3d tagToCamera = linkToCamera*mTags[i].mLinkPose;
        poseInit = getFiducialPose(tagToCamera);

        // TODO TEMP
        if (mDebug) {
          Eigen::Isometry3d tagToLocal = linkToLocal*mTags[i].mLinkPose;
          Eigen::Vector3d p0 = tagToLocal.translation();
          debugDrawPoint(p0.cast<float>(), Eigen::Vector3f(0, 0, 1), 10);
          Eigen::Vector3d v = tagToLocal.rotation().col(2);
          Eigen::Vector3d p1 = p0 + 0.05*v;
          debugDrawLine(p0.cast<float>(), p1.cast<float>(),
                        Eigen::Vector3f(0, 0, 1), 3);
        }
      }

      mTags[i].mTracked = false;
      fiducial_detector_error_t status;

      // run stereo detector
      if (mRunStereoAlgorithm) {
        float leftScore(0), rightScore(0);
        status = fiducial_stereo_process(mStereoDetector,
                                         left.data, right.data,
                                         left.cols, left.rows, left.channels(),
                                         poseInit, &poseFinal,
                                         &leftScore, &rightScore,
                                         false);
      }

      // run mono detector and use disparity to infer 3d
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
            int d00 = disp.at<uint16_t>(yInt, xInt);
            int d10 = disp.at<uint16_t>(yInt, xInt+1);
            int d01 = disp.at<uint16_t>(yInt+1, xInt);
            int d11 = disp.at<uint16_t>(yInt+1, xInt+1);
            float d = (1-xFrac)*(1-yFrac)*d00 + xFrac*(1-yFrac)*d10 +
              (1-xFrac)*yFrac*d01 + xFrac*yFrac*d11;

            // compute depth and 3d point
            float z = 16*mStereoBaseline*mCalibLeft(0,0) / d;
            Eigen::Vector3d pos = z*mCalibLeftInv*Eigen::Vector3d(x,y,1);
            poseFinal.pos.x = pos[0];
            poseFinal.pos.y = pos[1];
            poseFinal.pos.z = pos[2];
            poseFinal.rot = poseInit.rot;
            // TODO: fill in correct orientation
          }
        }
      }

      if (status == FIDUCIAL_DETECTOR_OK) {
        // populate this tag detection message
        drc::tag_detection_t detection;
        detection.utime = msg.utime;
        detection.id = mTags[i].mId;

        // put tag into local frame
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

        // four corners are irrelevant; set them all to tag center
        for (int k = 0; k < 4; ++k) {
          for (int m = 0; m < 2; ++m) detection.p[k][m] = pix[m];
        }

        // add this detection to list
        msg.detections.push_back(detection);
        msg.num_detections = msg.detections.size();

        // tracked successfully
        mTags[i].mCurPose.linear() = q.matrix();
        mTags[i].mCurPose.translation() = pos;
        mTags[i].mTracked = true;
      }
      else {
        std::cout << "warning: could not detect tag " << mTags[i].mId <<
          std::endl;
      }
    }
    mLcmWrapper->get()->publish(mTagChannel, &msg);

    // debug draw
    if (mDebug) {
      for (const auto& detection : msg.detections) {
        debugDrawPoint(Eigen::Vector3f(detection.pos[0], detection.pos[1],
                                       detection.pos[2]),
                       Eigen::Vector3f(1, 0, 1), 10);
      }
      bot_lcmgl_switch_buffer(mBotLcmgl);
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);
    std::cout << "PROCESSED IN " << dt.count()/1e6 << " SEC" << std::endl;
  }

};


int main(const int iArgc, const char** iArgv) {
  State state;

  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(state.mCameraChannel, "c", "camera_channel",
          "channel containing stereo image data");
  opt.add(state.mTagChannel, "d", "detection_channel",
          "channel on which to publish tag detections");
  opt.add(state.mRunStereoAlgorithm, "s", "stereo",
          "whether to run stereo-based detector");
  opt.add(state.mDoTracking, "t", "tracking",
          "whether to use previous detection to initialize current detection");
  opt.add(state.mDebug, "v", "verbose",
          "whether to publish lcmgl messages and other diagnostics");
  opt.parse();

  state.setup();
  state.start();
  
  return -1;
}
