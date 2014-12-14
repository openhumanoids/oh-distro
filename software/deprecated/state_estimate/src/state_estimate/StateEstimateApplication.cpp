
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
  ERSMsgSuffix = "";
  if (_switches->ExperimentalMsgs) {
	  ERSMsgSuffix = "_EXP";
  }

  // Using Maurices latest version of leg odometry
  cl_cfg.urdf_file = "";
  cl_cfg.param_file = "";
  cl_cfg.in_log_name = "";
  cl_cfg.out_log_name = "";
  cl_cfg.read_lcmlog = false;
  cl_cfg.begin_timestamp = -1;
  cl_cfg.end_timestamp = -1;
  cl_cfg.republish_incoming = false;
  cl_cfg.processing_rate = 1;

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
  IMUMessageProducer imuProducer("ATLAS_IMU_BATCH" + mMotionSimulatorSuffix, "EST_ROBOT_STATE" + ERSMsgSuffix);
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
    cl_cfg,
    atlasStateProducer.messageQueue(),
    imuProducer.messageQueue(),
    bdiPoseProducer.messageQueue(),
    viconPoseProducer.messageQueue(),
    matlabTruthPoseProducer.messageQueue(),
    INSUpdateProducer.messageQueue());


  // Setup shared memory with the StateEstimator object
  imuProducer.getIMUFilter()->setupEstimatorSharedMemory(estimator);

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




