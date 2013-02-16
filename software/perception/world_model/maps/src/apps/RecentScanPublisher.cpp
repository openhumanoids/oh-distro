// assume all returns facing latterally are wasted 
// -> half the scans
// assume that we cannot see back and that long ranges are useless due to resolution
// -> 70 degrees per scans
// 4 scans per degree.
// 20*70*4 = 5600 measurements per second - regardless of frequency of spinning

#include <iostream>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <drc_utils/Clock.hpp>
#include <maps/PointDataBuffer.hpp>
#include <maps/SensorDataReceiver.hpp>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <ConciseArgs>
#include <bot_lcmgl_client/lcmgl.h>


#include <pointcloud_tools/pointcloud_math.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>

using namespace std;
using namespace maps;

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
    mPrevAngle_ = -1;
    mPrevTime = -1;
    counter_ =0;

    pc_vis_ = new pointcloud_vis(mState->mLcm->getUnderlyingLCM());
    
  }

  void operator()() {
    while(true) {
      // wait for timer expiry
     // boost::asio::io_service service;
    //  boost::asio::deadline_timer timer(service);
  //    timer.expires_from_now(boost::posix_time::
//                             milliseconds(2*1000));
    //  timer.wait();

      
      maps::PointSet data;
      if (mState->mSensorDataReceiver->waitForData(data)) {
        mState->mPointDataBuffer->add(data);

        BotTrans b2l;
        bot_frames_get_trans_with_utime(mState->mBotFrames, "body", "local",
                                        data.mTimestamp, &b2l);
        
        BotTrans transform;
        bot_frames_get_trans_with_utime(mState->mBotFrames, "ROTATING_SCAN", "head",
                                        data.mTimestamp, &transform);
        double currentAngle = 2*acos(transform.rot_quat[0]);
        if (currentAngle > M_PI) {
          currentAngle -= 2*M_PI;
        }
        if (mPrevAngle_*currentAngle < 0) {
          std::cout << "Previous=" << (mPrevAngle_*180/M_PI) <<
            ", Current=" << (currentAngle*180/M_PI) << std::endl;

            mPrevTime =  data.mTimestamp - 2E6;
            cout << mPrevTime << " mPrevTime" << "\n";
            cout << data.mTimestamp << " data.mTimestamp" << "\n";
          maps::PointCloud::Ptr cloud =
            mState->mPointDataBuffer->getAsCloud(mPrevTime, data.mTimestamp);

          bot_lcmgl_t* lcmgl = mState->mLcmGl;
          
          bot_lcmgl_color3f(lcmgl, pc_vis_->colors[counter_*3], pc_vis_->colors[counter_*3+1], pc_vis_->colors[counter_*3+2]);
          bot_lcmgl_point_size(lcmgl, 1.5f);
          for (int i = 0; i < cloud->points.size(); ++i) {
            maps::PointCloud::PointType point = cloud->points[i];
            bot_lcmgl_begin(lcmgl, LCMGL_POINTS);
            bot_lcmgl_vertex3f(lcmgl, point.x, point.y, point.z);
            bot_lcmgl_end(lcmgl);
          }
          bot_lcmgl_switch_buffer(lcmgl);
          
          std::cout << "Published " << cloud->points.size() << " points " << std::endl;
          mPrevTime = data.mTimestamp;
          
          ostringstream fname;
          fname << "cloud " << b2l.trans_vec[0] << " "
              << b2l.trans_vec[1] << " "
              << b2l.trans_vec[2] << " "
              << b2l.rot_quat[0] << " "
              << b2l.rot_quat[1] << " "
              << b2l.rot_quat[2] << " "
              << b2l.rot_quat[3] << ".pcd";
          pcl::io::savePCDFileASCII (fname.str(), *cloud);
          std::cerr << data.mTimestamp << " =========Clock\n";
          std::cerr << "Saved " << cloud->points.size () << " data points to " << fname.str() << std::endl;
          counter_++;
          if(counter_*3 >= pc_vis_->colors.size() ){
           counter_=0; 
          }
         
          //pc_vis_->pose_collection_to_lcm_from_list(6001, world_to_jointsT); // all joints in world frame
          
          
        }

        mPrevAngle_ = currentAngle;
        
        
        
      }
    }
  }

protected:
  State* mState;
  double mPrevAngle_;
  int64_t mPrevTime;
  int counter_;
  pointcloud_vis* pc_vis_;
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
  state.mSensorDataReceiver->setMaxBufferSize(1000);

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
