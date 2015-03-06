#include "kinect-pointcloud-pub.h"
#include <boost/math/constants/constants.hpp>
#include <boost/program_options.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/kinect_pointcloud_t.h>
#include <algorithm>
#include <zlib.h>


namespace po = boost::program_options;

void onKinectFrame(const lcm_recv_buf_t *rbuf, const char *channel, 
		   const kinect_frame_msg_t *msg, void *user_data)
{
  return ((KinectPointCloudPub*)user_data)->OnKinectFrame(rbuf, channel, msg);
}

KinectPointCloudPub::KinectPointCloudPub(int argc, char **argv) : m_threadExit(false)
{
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ( "in", 
      po::value<std::string>(&m_kinectFrameName)->default_value(std::string("KINECT_FRAME")), 
      "input kinect LCM subscribe name")
    ( "out", 
      po::value<std::string>(&m_pointCloudName)->default_value(std::string("KINECT_POINTS")), 
      "output point cloud publication")
    ;
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    exit(1);
  }

  m_lcm = bot_lcm_get_global(NULL);

  m_kinectSubscription = kinect_frame_msg_t_subscribe(m_lcm, m_kinectFrameName.c_str(), onKinectFrame, this);

  m_calib = kinect_calib_new();
  m_calib->width = 640;
  m_calib->height = 480;

  m_calib->intrinsics_depth.fx = 576.09757860;
  m_calib->intrinsics_depth.cx = 321.06398107;
  m_calib->intrinsics_depth.cy = 242.97676897;
  
  m_calib->intrinsics_rgb.fx = 528.49404721;
  m_calib->intrinsics_rgb.cx = 319.50000000;
  m_calib->intrinsics_rgb.cy = 239.50000000;
  m_calib->intrinsics_rgb.k1 = 0;
  m_calib->intrinsics_rgb.k2 = 0;

  m_calib->shift_offset = 1093.4753;
  m_calib->projector_depth_baseline = 0.07214;;
  
  double R[9] = { 0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970 };
  double T[3] = { -0.015756, -0.000923, 0.002316 };

  memcpy(m_calib->depth_to_rgb_rot, R, 9*sizeof(double));
  memcpy(m_calib->depth_to_rgb_translation, T, 3*sizeof(double));

  m_pThread = new boost::thread(&KinectPointCloudPub::PubThread, this ); 
}

void KinectPointCloudPub::PubThread()
{
  boost::unique_lock<boost::mutex> lock(m_pubReadyMutex);
  while ( !m_threadExit ) {
    while ( m_pubQueue.empty() && !m_threadExit ) {
      m_pubReadyCondition.wait(lock);
    }
    //std::cout << m_pubQueue.size() << " in queue" << std::endl;

    MessageInfoPtr front = m_pubQueue.front();
    m_pubQueue.pop();

    kinect_pointcloud_t kpc;
    kpc.timestamp = front->timestamp;
    kpc.num = front->x.size();
    kpc.x = &front->x[0];
    kpc.y = &front->y[0];
    kpc.z = &front->z[0];
    
    kinect_pointcloud_t_publish(m_lcm, m_pointCloudName.c_str(), &kpc);
  }
}

KinectPointCloudPub::~KinectPointCloudPub()
{
  if ( m_kinectSubscription ) kinect_frame_msg_t_unsubscribe(m_lcm, m_kinectSubscription);
  kinect_calib_destroy(m_calib);

  {
    boost::unique_lock<boost::mutex> lock(m_pubReadyMutex);
    m_threadExit = true;
  }
  m_pubReadyCondition.notify_all();

  m_pThread->join();
  delete m_pThread;
}

void KinectPointCloudPub::OnKinectFrame(const lcm_recv_buf_t *rbuf, const char *channel, 
					const kinect_frame_msg_t *msg)
{
  uint16_t* depth_data = (uint16_t*)msg->depth.depth_data;

  if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {

    m_uncompressBuffer.resize(msg->depth.uncompressed_size);

    unsigned long dlen = msg->depth.uncompressed_size;
    int status = uncompress(&m_uncompressBuffer[0], &dlen, msg->depth.depth_data, msg->depth.depth_data_nbytes);
    if(status != Z_OK) {
      return;
    }
    depth_data = (uint16_t*)&m_uncompressBuffer[0];

  }

  if ( msg->depth.depth_data_format != KINECT_DEPTH_MSG_T_DEPTH_11BIT ) {
    std::cout << "ERROR: unable to handle anything but 11 bit depth data" << std::endl;
    return;
  }

  double depth_to_depth_xyz[16];
  kinect_calib_get_depth_uvd_to_depth_xyz_4x4(m_calib, depth_to_depth_xyz);

  MessageInfoPtr info(new MessageInfo(msg->depth.width * msg->depth.height, msg->timestamp));

  for ( int u = 0; u < msg->depth.width; u++ ) {
    for ( int v = 0; v < msg->depth.height; v++ ) {

      if (depth_data[v*msg->depth.width + u] == 2047) continue;

      double pix[] = { u, v, depth_data[v*msg->depth.width + u], 1.0 };
      double xyz[4];
      bot_matrix_vector_multiply_4x4_4d (depth_to_depth_xyz, pix, xyz);
      //bot_matrix_print(depth_to_depth_xyz,4,4);
      info->push_back(xyz[0]/xyz[3], xyz[1]/xyz[3], xyz[2]/xyz[3]);
      //std::cout << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ", " << xyz[3] << std::endl;
    }
  }

  {
    boost::unique_lock<boost::mutex> lock(m_pubReadyMutex);
    m_pubQueue.push(info);
    //std::cout << "pushed, size = " << m_pubQueue.size() << std::endl;
  }
  m_pubReadyCondition.notify_all();
  /*
  kinect_pointcloud_t kpc;
  kpc.timestamp = msg->timestamp;
  kpc.num = 10; //x.size();
  kpc.x = &x[0];
  kpc.y = &y[0];
  kpc.z = &z[0];

  kinect_pointcloud_t_publish(m_lcm, m_pointCloudName.c_str(), &kpc);
  */
}


