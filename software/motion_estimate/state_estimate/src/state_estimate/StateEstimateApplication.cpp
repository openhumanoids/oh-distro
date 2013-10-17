
#include "StateEstimateApplication.h"
#include "StateEstimator.h"
#include "LCMThread.h"
#include "LCMProducers.h"
#include "IMUMessageProducer.h"


//-----------------------------------------------------------------------------
StateEstimate::StateEstimateApplication::StateEstimateApplication(const command_switches &switches)
{
  _switches = &switches;

  mMotionSimulatorSuffix = "";

  if (_switches->MATLAB_MotionSimulator) {
    mMotionSimulatorSuffix = "_MS";
  }
}

//-----------------------------------------------------------------------------
StateEstimate::StateEstimateApplication::~StateEstimateApplication()
{

}

//-----------------------------------------------------------------------------
void StateEstimate::StateEstimateApplication::handleCommandLineArguments(int argc, char* argv[])
{
  VarNotUsed(argc);
  VarNotUsed(argv);
}

//-----------------------------------------------------------------------------
int StateEstimate::StateEstimateApplication::exec()
{

  LCMThread lcmThread;

  // create message producers
  AtlasStateMessageProducer atlasStateProducer("ATLAS_STATE");
  IMUMessageProducer imuProducer("ATLAS_IMU_BATCH" + mMotionSimulatorSuffix);
  PoseMessageProducer bdiPoseProducer("POSE_BDI");
  PoseMessageProducer viconPoseProducer("ATLAS_VICON");

  // connect message producers to lcm
  atlasStateProducer.subscribe(lcmThread.lcmHandle());
  imuProducer.subscribe(lcmThread.lcmHandle());
  bdiPoseProducer.subscribe(lcmThread.lcmHandle());
  viconPoseProducer.subscribe(lcmThread.lcmHandle());

  StateEstimator estimator(
    _switches,
    lcmThread.lcmHandle(),
    atlasStateProducer.messageQueue(),
    imuProducer.messageQueue(),
    bdiPoseProducer.messageQueue(),
    viconPoseProducer.messageQueue() );

  // start comm thread
  lcmThread.start();

  // start estimator thread
  estimator.start();

  // wait for user to signal termination
  std::cout << "Press any key to exit..." << std::endl;
  getchar();

  // wait for threads to terminate
  atlasStateProducer.stop();
  estimator.stop();
  lcmThread.stop();

  return 0;
}




