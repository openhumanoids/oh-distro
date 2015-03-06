#ifndef __lidartilt_h__
#define __lidartilt_h__

#include <glib.h>
#include <lcmtypes/bot_core.h>
#include <bot_core/bot_core.h>
//#include <lcmtypes/microstrain_ins_t.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <list>
#include <string>
#include <boost/circular_buffer.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/shared_ptr.hpp>
#include "openni_camera/openni_device.h"
#include "openni_camera/openni_driver.h"

//------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <inttypes.h>

#include <zlib.h>
#include <glib.h>
#include <lcm/lcm.h>
#include <pthread.h>

#if USE_JPEG_UTILS_POD
#include <jpeg-utils/jpeg.h>
#else
#include "jpeg-utils-ijg.h"
#endif

#include <lcmtypes/kinect_depth_msg_t.h>
#include <lcmtypes/kinect_image_msg_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>

#include <lcmtypes/kinect_sensor_status_t.h>

#include "timestamp.h"
#include "pixels.h"

#define dbg(...) fprintf(stderr, __VA_ARGS__)

#define alpha 0.05 //decay rate for the moving average
//----------------------------------


class KinectOpenniLCM
{
 public:

  KinectOpenniLCM(int argc, char **argv);
  virtual ~KinectOpenniLCM();

 private:
  typedef struct _rate_t {
      double target_hz;
      double current_hz;
      int64_t last_tick;
      int64_t tick_count;
  } rate_t;

  void SetupDevice(const std::string& deviceId);
  void ImageCallback (boost::shared_ptr<openni_wrapper::Image> image, void* cookie);
  void DepthCallback (boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie);
  void startSynchronization ();
  void stopSynchronization ();
  void usage(const char*);

  rate_t* rate_new(double target_hz);

  void rate_destroy(rate_t* rate);

  /**
   * returns: 1 if an image should be published.  0 if not
   */
  int rate_check(rate_t* rate);

  static void *status_thread(void *data);

 private:
  uint8_t* image_buf;
  int image_buf_size;
  
  uint16_t* depth_unpack_buf;
  int depth_unpack_buf_size;

  uint8_t* depth_compress_buf;
  int depth_compress_buf_size;  
  
  
  int jpeg_quality;
  lcm_t* m_lcm;
  int64_t m_lastImageTime;
  int64_t m_lastDepthTime;
  boost::mutex m_mutex;

  uint8_t* rgb_data;
  uint8_t* depth_data;
  bool new_data;

  double target_rate; 
  int throttle;

  pthread_t  work_thread;

  rate_t* capture_rate;
  rate_t* report_rate;  
  int8_t  requested_image_format;
  int use_zlib;

  boost::shared_ptr<openni_wrapper::OpenNIDevice> m_device;

};

#endif
