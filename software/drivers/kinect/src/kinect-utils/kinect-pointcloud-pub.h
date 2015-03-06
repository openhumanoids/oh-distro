#ifndef __lidartilt_h__
#define __lidartilt_h__

#include <glib.h>
#include <lcmtypes/bot_core.h>
#include <bot_core/bot_core.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <queue>
#include <string>
#include <boost/thread.hpp>
#include <lcmtypes/kinect_frame_msg_t.h>
#include "kinect-utils.h"

class KinectPointCloudPub
{
 public:

  class MessageInfo {
  public:
    MessageInfo(unsigned int size, const int64_t& ts) {
      x.reserve(size);
      y.reserve(size);
      z.reserve(size);
      timestamp = ts;
    }
    void push_back(float xx, float yy, float zz) {
      x.push_back(xx); 
      y.push_back(yy);
      z.push_back(zz);
    }
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    int64_t timestamp;
  };
  typedef boost::shared_ptr<MessageInfo> MessageInfoPtr;

  KinectPointCloudPub(int argc, char **argv);
  virtual ~KinectPointCloudPub();

  void OnKinectFrame(const lcm_recv_buf_t *rbuf, const char *channel, 
		     const kinect_frame_msg_t *msg);

 private:
  void PubThread();

  std::string m_kinectFrameName;
  std::string m_pointCloudName;

  lcm_t* m_lcm;
  kinect_frame_msg_t_subscription_t* m_kinectSubscription;

  std::vector<uint8_t> m_uncompressBuffer;
  std::vector<uint16_t> m_disparity;
  KinectCalibration *m_calib;

  bool m_threadExit;
  boost::thread* m_pThread;
  boost::condition_variable m_pubReadyCondition;
  boost::mutex m_pubReadyMutex;

  std::queue<MessageInfoPtr> m_pubQueue;
};

#endif
