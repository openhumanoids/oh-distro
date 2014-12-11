#include <iostream>
#include <memory>
#include <thread>

#include <drc_utils/Clock.hpp>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <affordance/AffordanceUpWrapper.h>

#include <lcmtypes/drc/gaze_command_t.hpp>
#include <lcmtypes/drc/neck_pitch_t.hpp>

struct Worker {
  drc::BotWrapper::Ptr mBotWrapper;
  bool mIsRunning;
  std::thread mThread;
  int mAffordanceId;
  std::shared_ptr<affordance::AffordanceUpWrapper> mAffordanceWrapper;

  Worker(const drc::BotWrapper::Ptr& iWrapper) {
    mBotWrapper = iWrapper;
    mBotWrapper->getLcm()->subscribe("GAZE_COMMAND", &Worker::onCommand, this);
    mAffordanceId = -1;
    boost::shared_ptr<lcm::LCM> boostLcm
      (new lcm::LCM(mBotWrapper->getLcm()->getUnderlyingLCM()));
    mAffordanceWrapper.reset(new affordance::AffordanceUpWrapper(boostLcm));
    mIsRunning = true;
    mThread = std::thread(std::ref(*this));
  }

  ~Worker() {
    mIsRunning = false;
    mThread.join();
  }

  void onCommand(const lcm::ReceiveBuffer* iBuf,
                 const std::string& iChannel,
                 const drc::gaze_command_t* iMessage) {
    std::cout << "Received gaze command for affordance " <<
      iMessage->affordance_id << std::endl;
    mAffordanceId = iMessage->affordance_id;
  }

  void operator()() {
    std::cout << "Started worker thread" << std::endl;

    const double kPi = 4*atan(1);
    const double kAngleThresh = 10*kPi/180;

    while (mIsRunning) {
      // look for affordance with given id
      std::vector<affordance::AffConstPtr> affordances;
      mAffordanceWrapper->getAllAffordances(affordances);
      affordance::AffConstPtr aff;
      for (size_t i = 0; i < affordances.size(); ++i) {
        if (affordances[i]->_uid == mAffordanceId) {
          aff = affordances[i];
          break;
        }
      }

      // if found, compute appropriate pitch angle and publish
      if (aff != NULL) {
        Eigen::Vector3d affPos(aff->_origin_xyz[0], aff->_origin_xyz[1],
                               aff->_origin_xyz[2]);
        Eigen::Isometry3d headToLocal;
        if (mBotWrapper->getTransform("head", "local", headToLocal)) {

          // compute current head pitch
          double currentPitch = asin(-headToLocal(0,2));

          // compute desired head pitch
          Eigen::Vector3d headPos = headToLocal.translation();
          Eigen::Vector3d headToAffDir = affPos - headPos;
          double yaw = atan2(headToLocal(1,0), headToLocal(0,0));
          Eigen::Matrix3d rotation;
          rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
          headToAffDir = rotation.transpose()*headToAffDir;
          headToAffDir[1] = 0;
          headToAffDir.normalize();
          double desiredPitch = asin(-headToAffDir[2]);

          // create and send message if current does not match desired
          if (fabs(desiredPitch - currentPitch) > kAngleThresh) {
            drc::neck_pitch_t msg;
            msg.utime = drc::Clock::instance()->getCurrentTime();
            msg.pitch = desiredPitch;
            mBotWrapper->getLcm()->publish("DESIRED_NECK_PITCH", &msg);
          }
        }
      }
      
      // wait in between goal publications
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
};

int main(const int iArgc, const char** iArgv) {
  drc::BotWrapper::Ptr botWrapper(new drc::BotWrapper());
  std::shared_ptr<lcm::LCM> lcm = botWrapper->getLcm();
  drc::LcmWrapper lcmWrapper(lcm);

  Worker worker(botWrapper);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  lcmWrapper.startHandleThread(true);
}
