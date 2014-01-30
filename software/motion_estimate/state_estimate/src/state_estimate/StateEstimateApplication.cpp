
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
  NavStateMessageProducer matlabTruthPoseProducer("TRUTH_TRAJ_MATLAB");
  INSUpdateMessageProducer INSUpdateProducer("INS_ERR_UPDATE"); //  This listens to update messages destined for the INS

  // connect message producers to lcm
  atlasStateProducer.subscribe(lcmThread.lcmHandle());
  imuProducer.subscribe(lcmThread.lcmHandle());
  bdiPoseProducer.subscribe(lcmThread.lcmHandle());
  viconPoseProducer.subscribe(lcmThread.lcmHandle());
  matlabTruthPoseProducer.subscribe(lcmThread.lcmHandle());
  INSUpdateProducer.subscribe(lcmThread.lcmHandle());

  imuProducer.setSpecialLCMPtr(lcmThread.lcmHandle());

  StateEstimator estimator(
    _switches,
    lcmThread.lcmHandle(),
    atlasStateProducer.messageQueue(),
    imuProducer.messageQueue(),
    bdiPoseProducer.messageQueue(),
    viconPoseProducer.messageQueue(),
    matlabTruthPoseProducer.messageQueue(),
    INSUpdateProducer.messageQueue());

  IMUFilter* imuFilter = imuProducer.getIMUFilter();
  imuFilter->setInertialOdometry( estimator.getInertialOdometry() );
  imuFilter->setERSMsg( estimator.getERSMsg() );
  imuFilter->setDataFusionReqMsg( estimator.getDataFusionReqMsg() );


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




