#include "RobotState.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/robot_urdf_t.hpp>
#include <lcmtypes/bot_core/robot_state_t.hpp>
//#include <kinematics/kinematics_model_gfe.h>

using namespace maps;

struct RobotState::Helper {
  std::shared_ptr<lcm::LCM> mLcm;
  lcm::Subscription* mUrdfSubscription;
  lcm::Subscription* mStateSubscription;
  bool mGotUrdf;
  bool mGotState;
//  std::shared_ptr<kinematics::Kinematics_Model_GFE> mKinModel;
  std::string mHandLinkNameLeft;
  std::string mHandLinkNameRight;

  Helper(const std::shared_ptr<lcm::LCM> iLcm) {
    mLcm = iLcm;
    mGotUrdf = mGotState = false;
    mUrdfSubscription = mLcm->subscribe("ROBOT_MODEL", &Helper::onUrdf, this);
    mStateSubscription = NULL;
    mHandLinkNameLeft = "l_hand";
    mHandLinkNameRight = "r_hand";
  }

  ~Helper() {
    mLcm->unsubscribe(mStateSubscription);
    mLcm->unsubscribe(mUrdfSubscription);
  }

  void onUrdf(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel, 
              const bot_core::robot_urdf_t* iMessage) {
    if (mGotUrdf) return;
    mGotUrdf = true;
//    mKinModel.reset(new kinematics::Kinematics_Model_GFE
//                    (iMessage->urdf_xml_string));

    mLcm->unsubscribe(mUrdfSubscription);
    mUrdfSubscription = NULL;
    mStateSubscription = mLcm->subscribe("EST_ROBOT_STATE",
                                         &Helper::onState, this);
  }

  void onState(const lcm::ReceiveBuffer* iBuf, const std::string& iChannel, 
               const bot_core::robot_state_t* iMessage) {
    if (!mGotUrdf) return;

//     mKinModel->set(*iMessage);
    return;

/*
    if (!mGotState) {
      std::map<std::string, KDL::Frame> linkFrames = mKinModel->link_frames();
      if (linkFrames.find("left_palm") != linkFrames.end()) {
        mHandLinkNameLeft = "left_palm";
      }
      else if (linkFrames.find("left_base_link") != linkFrames.end()) {
        mHandLinkNameLeft = "left_base_link";
      }
      if (linkFrames.find("right_palm") != linkFrames.end()) {
        mHandLinkNameRight = "right_palm";
      }
      else if (linkFrames.find("right_base_link") != linkFrames.end()) {
        mHandLinkNameRight = "right_base_link";
      }
      mGotState = true;
    }
*/
  }
};

RobotState::
RobotState(const std::shared_ptr<lcm::LCM>& iLcm) {
  mHelper.reset(new Helper(iLcm));
  mHelper->mLcm = iLcm;
}

RobotState::
~RobotState() {
}

bool RobotState::
getPose(const std::string& iLink, Eigen::Quaternionf& oOrientation,
        Eigen::Vector3f& oPosition) const {
  if (!mHelper->mGotState) return false;
  std::string linkName = iLink;
  if (linkName == "l_hand") linkName = mHelper->mHandLinkNameLeft;
  if (linkName == "r_hand") linkName = mHelper->mHandLinkNameRight;
//  KDL::Frame frame = mHelper->mKinModel->link(linkName);
//  oPosition << frame.p[0], frame.p[1], frame.p[2];
  double x,y,z,w;
//  frame.M.GetQuaternion(x, y, z, w);
  oOrientation.w() = w;
  oOrientation.x() = x;
  oOrientation.y() = y;
  oOrientation.z() = z;
  return true;
}
