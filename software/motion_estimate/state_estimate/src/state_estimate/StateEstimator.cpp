#include "StateEstimator.h"

//-----------------------------------------------------------------------------
StateEstimate::StateEstimator::StateEstimator(
    boost::shared_ptr<lcm::LCM> lcmHandle,
    AtlasStateQueue& atlasStateQueue,
    IMUQueue& imuQueue,
    PoseQueue& bdiPoseQueue,
    PoseQueue& viconPoseQueue ) :

  mLCM(lcmHandle),
  mAtlasStateQueue(atlasStateQueue),
  mIMUQueue(imuQueue),
  mBDIPoseQueue(bdiPoseQueue),
  mViconQueue(viconPoseQueue)
{

  // TODO -- dehann, this should be initialized to the number of joints in the system, but just hacking to get it going for now
  jFilters.setSize(56);
  std::cout << "StateEstimator::StateEstimator -- hardcoded the number of joint Kalman Filters to 56" << std::endl;
  std::cerr << "StateEstimator::StateEstimator -- hardcoded the number of joint Kalman Filters to 56" << std::endl;
  
}

// TODO -- fix this constructor
//StateEstimate::StateEstimator::StateEstimator(
//    boost::shared_ptr<lcm::LCM> lcmHandle,
//    messageQueues& msgQueue ) :
//  
//  mLCM(lcmHandle),
//  mMSGQueues(msgQueue)
//{
//
//}

//-----------------------------------------------------------------------------
StateEstimate::StateEstimator::~StateEstimator()
{

}

//-----------------------------------------------------------------------------
void StateEstimate::StateEstimator::run()
{
  drc::atlas_state_t atlasState;
  drc::atlas_raw_imu_t imu;
  bot_core::pose_t bdiPose;
  bot_core::pose_t viconPose;

  while (!this->ShouldStop)
  {

    // wait for at least one new atlas_state message
	  // TODO -- Pat please make this pass on any event
    this->mAtlasStateQueue.waitWhileEmpty();


    int nAtlasStates = mAtlasStateQueue.size();
    for (int i = 0; i < nAtlasStates; ++i)
    {
      this->mAtlasStateQueue.dequeue(atlasState);

      // do something with new atlas state...
      
      // extract joint angles
      atlasState.joint_position[0]; //num_joints
      // filter_bank_joints()
      
      // pass to joint_velocity_handler
      
      // publish result in ERS message
      
    }

    int nIMU = mIMUQueue.size();
    printf("have %d new imu\n", nIMU);
    for (int i = 0; i < nIMU; ++i)
    {
      this->mIMUQueue.dequeue(imu);

      // do something with new imu...
      
      
    }


    const int nPoses = mBDIPoseQueue.size();
    for (int i = 0; i < nPoses; ++i)
    {
      mBDIPoseQueue.dequeue(bdiPose);

      // do something with new bdi pose...
    }

    const int nViconPoses = mViconQueue.size();
    for (int i = 0; i < nViconPoses; ++i)
    {
      mViconQueue.dequeue(viconPose);

      // do something with new vicon pose...
    }

    // add artificial delay
    //boost::this_thread::sleep(boost::posix_time::milliseconds(500));

  }
}
