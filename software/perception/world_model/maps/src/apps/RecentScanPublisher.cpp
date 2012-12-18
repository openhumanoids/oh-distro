#include <iostream>
#include <boost/thread.hpp>
#include <drc_utils/Clock.hpp>
#include <maps/PointDataBuffer.hpp>
#include <maps/SensorDataReceiver.hpp>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <ConciseArgs>
#include <bot_lcmgl_client/lcmgl.h>

using namespace std;

struct State {
  boost::shared_ptr<lcm::LCM> mLcm;
  boost::shared_ptr<PointDataBuffer> mPointDataBuffer;
  boost::shared_ptr<SensorDataReceiver> mSensorDataReceiver;
  BotParam* mBotParam;
  BotFrames* mBotFrames;
  bot_lcmgl_t* mLcmGl;

  State() {
    mPointDataBuffer.reset(new PointDataBuffer());
    mSensorDataReceiver.reset(new SensorDataReceiver());
    mLcm.reset(new lcm::LCM());
    drc::Clock::instance()->setLcm(mLcm);
    mSensorDataReceiver->setLcm(mLcm);
    mBotParam = bot_param_new_from_server(mLcm->getUnderlyingLCM(), 0);
    mSensorDataReceiver->setBotParam(mBotParam);
    mBotFrames = bot_frames_get_global(mLcm->getUnderlyingLCM(), mBotParam);
    mLcmGl = bot_lcmgl_init(mLcm->getUnderlyingLCM(), "recent-points");
  }

  ~State() {
    bot_param_destroy(mBotParam);
    bot_lcmgl_destroy(mLcmGl);
  }    
};

class DataConsumer {
public:
  DataConsumer(State* iState) {
    mState = iState;
    mPrevAngle = -1;
    mPrevTime = -1;
  }

  void operator()() {
    while(true) {
      SensorDataReceiver::PointCloudWithPose data;
      if (mState->mSensorDataReceiver->waitForData(data)) {
        PointDataBuffer::PointSet pointSet;
        pointSet.mTimestamp = data.mTimestamp;
        pointSet.mPoints = data.mPointCloud;
        pointSet.mToLocal = data.mPose;
        mState->mPointDataBuffer->add(pointSet);

        BotTrans transform;
        bot_frames_get_trans_with_utime(mState->mBotFrames, "ROTATING_SCAN", "head",
                                        data.mTimestamp, &transform);
        double currentAngle = 2*acos(transform.rot_quat[0]);
        if (currentAngle > M_PI) {
          currentAngle -= 2*M_PI;
        }
        if (mPrevAngle*currentAngle < 0) {
          std::cout << "Previous=" << (mPrevAngle*180/M_PI) <<
            ", Current=" << (currentAngle*180/M_PI) << std::endl;

          maptypes::PointCloud::Ptr cloud =
            mState->mPointDataBuffer->getAsCloud(mPrevTime, data.mTimestamp);

          bot_lcmgl_t* lcmgl = mState->mLcmGl;
          bot_lcmgl_color3f(lcmgl, 0.0f, 0.5f, 0.0f);
          for (int i = 0; i < cloud->points.size(); ++i) {
            maptypes::PointCloud::PointType point = cloud->points[i];
            bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
            bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
            bot_lcmgl_end(lcmgl);
          }
          bot_lcmgl_switch_buffer(lcmgl);
          
          std::cout << "Published " << cloud->points.size() << " points " << std::endl;
          mPrevTime = data.mTimestamp;
        }

        mPrevAngle = currentAngle;
      }
    }
  }

protected:
  State* mState;
  double mPrevAngle;
  int64_t mPrevTime;
};



int main(const int iArgc, const char** iArgv) {
  State state;

  // parse arguments
  double mapResolution = 0.02;
  string laserChannel = "ROTATING_SCAN";
  double xDim(10), yDim(10), zDim(10);
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(laserChannel, "l", "laser_channel", "channel for incoming scan data");
  opt.parse();
  state.mSensorDataReceiver->
    addChannel(laserChannel,
               SensorDataReceiver::SensorTypePlanarLidar,
               laserChannel, "local");

  // set up remaining parameters
  state.mSensorDataReceiver->setMaxBufferSize(100);

  // start running data receiver
  state.mSensorDataReceiver->start();

  // start consuming data
  DataConsumer consumer(&state);
  boost::thread thread(consumer);

  // main lcm loop
  while (0 == state.mLcm->handle());

  state.mSensorDataReceiver->stop();
  thread.join();

  return 0;
}
