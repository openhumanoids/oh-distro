#include "LidarAccumulator.hpp"

#include <fstream>
#include <chrono>
#include <thread>

#include <ConciseArgs>
#include <drc_utils/LcmWrapper.hpp>
#include <drc_utils/BotWrapper.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/footstep_plan_progress_t.hpp>

struct State {
  drc::LcmWrapper::Ptr mLcmWrapper;
  drc::BotWrapper::Ptr mBotWrapper;
  std::shared_ptr<maps::LidarAccumulator> mAccum;
  bool mRunContinuously;

  void setup() {
    mLcmWrapper.reset(new drc::LcmWrapper());
    mBotWrapper.reset(new drc::BotWrapper(mLcmWrapper->get()));
    mAccum.reset(new maps::LidarAccumulator());
    mAccum->setBotWrapper(mBotWrapper);
    auto lcm = mLcmWrapper->get();
    lcm->subscribe("FOOTSTEP_PLAN_PROGRESS", &State::onFootstepProgress,this);
  }

  void start() {
    if (mRunContinuously) mAccum->start();
    mLcmWrapper->startHandleThread(false);  // TODO
  }

  void onFootstepProgress(const lcm::ReceiveBuffer* iBuf,
                          const std::string& iChannel,
                          const drc::footstep_plan_progress_t* iMessage) {
    if (mRunContinuously) return;
    if (mAccum->isRunning()) return;
    if ((iMessage->num_steps > 0) &&
        (iMessage->current_step == iMessage->num_steps)) {
      std::cout << "last footstep reached; starting accumulator" << std::endl;
      mAccum->start();
    }
  }
};

int main(const int iArgc, const char** iArgv) {
  bool runContinuously = false;

  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(runContinuously, "c", "continuous", "run continuously");
  opt.parse();

  State state;
  state.mRunContinuously = runContinuously;
  state.setup();
  state.start();



  // TODO: debug
  state.mAccum->setRangeLimits(0.3,3.0);
  std::this_thread::sleep_for(std::chrono::seconds(44));
  std::vector<Eigen::Vector3f> points;
  state.mAccum->getPointCloud(1,points);
  std::cout << "GOT " << points.size() << " POINTS" << std::endl;

  std::ofstream ofs("/home/antone/temp/cloud.txt");
  for (auto pt : points) {
    ofs << pt[0] << " " << pt[1] << " " << pt[2] << std::endl;
  }
  ofs.close();

  return 1;
}
