#include <signal.h>
#include <cstdio>
#include <ros/ros.h>
#include "sandia_hand/hand.h"
#include "sandia_hand/palm_state.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sandia_hand_msgs/RawFingerState.h>
#include <sandia_hand_msgs/RawPalmState.h>
#include <sandia_hand_msgs/RawMoboState.h>
#include <sandia_hand_msgs/SetJointLimitPolicy.h>
#include <osrf_msgs/JointCommands.h>
#include <sandia_hand_msgs/SetFingerHome.h>
#include <sandia_hand_msgs/RelativeJointCommands.h>
#include <sandia_hand_msgs/GetParameters.h>
#include <sandia_hand_msgs/SetParameters.h>
using namespace sandia_hand;
using std::string;
using std::vector;

/////////////////////////////////////////////////////////////////////////
sandia_hand_msgs::RawFingerState g_raw_finger_state;
sandia_hand_msgs::RawMoboState   g_raw_mobo_state;
sandia_hand_msgs::RawPalmState   g_raw_palm_state;
ros::Publisher *g_raw_finger_state_pubs[Hand::NUM_FINGERS] = {NULL};
ros::Publisher *g_raw_mobo_state_pub = NULL;
ros::Publisher *g_raw_palm_state_pub = NULL;
bool g_done = false;
bool g_use_finger[4] = { true, true, true, true };
static int32_t g_last_fmcb_hall_pos[Hand::NUM_FINGERS][3]; // hack

// set the joint limits for each finger
static float upper_limits_default[4][3] = { { 1.5 ,  1.5,  1.7},
                                            { 0.05,  1.5,  1.7},
                                            { 0.05,  1.5,  1.7},
                                            { 0.3 ,  1.1,  1.0} };
static float lower_limits_default[4][3] = { {-0.05, -1.2, -1.2},
                                            {-0.05, -1.2, -1.2},
                                            {-1.5 , -1.2, -1.2},
                                            {-1.5 , -1.2, -0.8} };

static float upper_limits_none[4][3] = { { 1.57,  1.57,  1.57},
                                         { 1.57,  1.57,  1.57},
                                         { 1.57,  1.57,  1.57},
                                         { 1.57,  1.57,  1.57} };
static float lower_limits_none[4][3] = { {-1.57, -1.57, -1.57},
                                         {-1.57, -1.57, -1.57},
                                         {-1.57, -1.57, -1.57},
                                         {-1.57, -1.57, -1.57} };


/////////////////////////////////////////////////////////////////////////

void signal_handler(int signum)
{
  if (signum == SIGINT)
    g_done = true;
}

void listenToHand(Hand *hand, const float seconds)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    hand->listen(0.01);
    if ((ros::Time::now() - t_start).toSec() > seconds)
      break;
  }
}

void shutdownHand(Hand *hand)
{
  for (int i = 0; i < Hand::NUM_FINGERS; i++)
    if (g_use_finger[i])
      hand->setFingerControlMode(i, Hand::FCM_IDLE);
  listenToHand(hand, 0.001); // todo: something smarter
  hand->setCameraStreaming(false, false);
  listenToHand(hand, 0.001); // todo: something smarter
  hand->setFingerAutopollHz(0);
  listenToHand(hand, 0.001); // todo: something smarter
  hand->setMoboStateHz(0);
  listenToHand(hand, 0.001); // todo: something smarter
  hand->setAllFingerPowers(Hand::FPS_OFF);
}

int perish(const char *msg, Hand *hand)
{
  ROS_FATAL("%s", msg);
  shutdownHand(hand);
  return 1;
}

bool getParametersSrv(Hand *hand, const int finger_idx, 
                      sandia_hand_msgs::GetParameters::Request &req,
                      sandia_hand_msgs::GetParameters::Response &res)
{
  //printf("getParametersSrv, finger %d\n", finger_idx);
  if (finger_idx < 0 || finger_idx > Hand::NUM_FINGERS ||
      !g_use_finger[finger_idx])
    return false;
  const vector<sandia_hand::Param> params = 
    hand->fingers[finger_idx].mm.getParams();
  res.parameters.resize(params.size());
  for (size_t i = 0; i < params.size(); i++)
  {
    res.parameters[i].name = params[i].getName();
    if (params[i].getType() == sandia_hand::Param::PARAM_INT)
    {
      res.parameters[i].val_type = sandia_hand_msgs::Parameter::INTEGER;
      res.parameters[i].i_val = params[i].getIntVal();
    }
    else // it's a float
    {
      res.parameters[i].val_type = sandia_hand_msgs::Parameter::FLOAT;
      res.parameters[i].f_val = params[i].getFloatVal();
    }
  }
  return true;
}

bool setParametersSrv(Hand *hand, const int finger_idx,
                      sandia_hand_msgs::SetParameters::Request  &req,
                      sandia_hand_msgs::SetParameters::Response &res)
{
  printf("setParametersSrv for finger %d:\n", finger_idx);
  for (int i = 0; i < (int)req.parameters.size(); i++)
  {
    sandia_hand_msgs::Parameter *p = &req.parameters[i];
    printf("  %s: type=%d, int=%d, float=%f\n",
           p->name.c_str(), p->val_type, p->i_val, p->f_val);
    if (p->val_type == sandia_hand_msgs::Parameter::FLOAT)
      hand->fingers[finger_idx].mm.setParamFloat(p->name, p->f_val);
    else if (p->val_type == sandia_hand_msgs::Parameter::INTEGER)
      hand->fingers[finger_idx].mm.setParamInt(p->name, p->i_val);
  }
  return true;
}

bool setJointLimitPolicy(Hand *hand, const std::string &policy_name,
                         bool resume_autopoll = true)
{
  ROS_INFO("setJointLimitPolicy(%s)", policy_name.c_str());
  const float (*upper_limits)[3] = NULL;
  const float (*lower_limits)[3] = NULL;
  if (policy_name == string("default"))
  {
    upper_limits = upper_limits_default;
    lower_limits = lower_limits_default;
  }
  else if (policy_name == string("none"))
  {
    upper_limits = upper_limits_none;
    lower_limits = lower_limits_none;
  }
  else
  {
    ROS_ERROR("unknown joint limit policy: %s", policy_name.c_str());
    return false;
  }
  if (!hand->setFingerAutopollHz(0))
  {
    ROS_ERROR("couldn't stop finger autopoll");
    return false;
  }
  listenToHand(hand, 0.1);
  for (int i = 0; i < Hand::NUM_FINGERS; i++)
  {
    if (!g_use_finger[i])
      continue;
    /*
    ROS_INFO("finger %d lower limits: %.3f %.3f %.3f",
             i, lower_limits[i][0], lower_limits[i][1], lower_limits[i][2]);
    ROS_INFO("finger %d upper limits: %.3f %.3f %.3f",
             i, upper_limits[i][0], upper_limits[i][1], upper_limits[i][2]);
    */
    if (!hand->fingers[i].mm.setJointLimits(lower_limits[i], upper_limits[i]))
    {
      ROS_ERROR("couldn't set joint limits on finger %d", i);
      return false;
    }
  }
  if (resume_autopoll && !hand->setFingerAutopollHz(100))
  {
    ROS_ERROR("couldn't resume finger autopoll");
    return false;
  }
  return true;
}

bool setHomeSrv(Hand *hand, 
                sandia_hand_msgs::SetFingerHome::Request &req,
                sandia_hand_msgs::SetFingerHome::Response &res)
{
  ROS_INFO("set home service call for finger %d", req.finger_idx);
  if (req.finger_idx > 3 || !g_use_finger[req.finger_idx])
    return false;
  hand->setFingerAutopollHz(0); // we need a clear channel to the finger
  listenToHand(hand, 0.1); // wait for scheduler to realize we're stopped
  hand->setFingerControlMode(req.finger_idx, Hand::FCM_IDLE); // no controller
  listenToHand(hand, 0.1);
  if (!hand->fingers[req.finger_idx].mm.setHallOffsets(
                                     g_last_fmcb_hall_pos[req.finger_idx]))
    ROS_ERROR("unable to set finger %d hall offsets", req.finger_idx);
  hand->setFingerJointPos(req.finger_idx, 0, 0, 0);
  hand->setFingerControlMode(req.finger_idx, Hand::FCM_JOINT_POS); // resume
  hand->setFingerAutopollHz(100);
  return true;
}

bool setJointLimitPolicySrv(Hand *hand,
                         sandia_hand_msgs::SetJointLimitPolicy::Request &req,
                         sandia_hand_msgs::SetJointLimitPolicy::Response &res)
{
  return setJointLimitPolicy(hand, req.policy);
}


void fingerJointCommandsCallback(Hand *hand, const uint8_t finger_idx,
                                 const osrf_msgs::JointCommandsConstPtr &msg)
{
  //ROS_INFO("finger %d joint command %.3f %.3f %.3f",
  //         finger_idx, msg->position[0], msg->position[1], msg->position[2]);
  if (msg->position.size() < 3)
  {
    ROS_WARN("ignoring joint commands message with insufficient length");
    return; // woah there partner
  }
  if (!g_use_finger[finger_idx])
    return;
  hand->setFingerJointPos(finger_idx,
                      msg->position[0], msg->position[1], msg->position[2]);
}

void relativeJointCommandsCallback(Hand *hand,
                const sandia_hand_msgs::RelativeJointCommands::ConstPtr &msg)
{
  if (msg->position.size() < 12)
  {
    ROS_WARN("ignoring joint commands message with insufficient length");
    return; // woah there partner
  }
  uint8_t max_effort_dummy[12]; // todo: if max_effort is populated, take it.
  float relative_joint_angles[12];
  for (int i = 0; i < 12; i++)
  {
    max_effort_dummy[i] = 50;
    relative_joint_angles[i] = msg->position[i];
  }
  hand->setAllRelativeFingerJointPos(relative_joint_angles, max_effort_dummy);
}

void jointCommandsCallback(Hand *hand, 
                           const osrf_msgs::JointCommandsConstPtr &msg)
{
  /*
  ROS_INFO("joint command %.3f %.3f %.3f",
           msg->position[0], msg->position[1], msg->position[2]);
  hand->setFingerJointPos(0, 
         msg->position[0], msg->position[1], msg->position[2]);
  */
  if (msg->position.size() < 12)
  {
    ROS_WARN("ignoring joint commands message with insufficient length");
    return; // woah there partner
  }
  uint8_t max_effort_dummy[12]; // todo: if max_effort is populated, take it.
  float joint_angles[12];
  for (int i = 0; i < 12; i++)
  {
    max_effort_dummy[i] = 50;
    joint_angles[i] = msg->position[i];
  }
  hand->setAllFingerJointPos(joint_angles, max_effort_dummy);
}

// todo: have firmware and driver pull from same .h file, instead of pure haxx
typedef struct
{
  uint32_t pp_tactile_time;
  uint32_t dp_tactile_time;
  uint32_t fmcb_time;
  uint16_t pp_tactile[6];
  uint16_t dp_tactile[12];
  int16_t  pp_imu[6];
  int16_t  dp_imu[6];
  int16_t  fmcb_imu[6];
  uint16_t pp_temp[4];
  uint16_t dp_temp[4];
  uint16_t fmcb_temp[3];
  uint16_t fmcb_voltage;
  uint16_t fmcb_pb_current;
  uint32_t pp_strain;
  int32_t  fmcb_hall_tgt[3];
  int32_t  fmcb_hall_pos[3];
  int16_t  fmcb_effort[3];
} finger_state_t;

void rxFingerState(const uint8_t finger_idx, 
                    const uint8_t *payload, const uint16_t payload_len)
{
  //printf("rxFingerState %d:   %d bytes\n", finger_idx, payload_len);
  if (payload_len < sizeof(finger_state_t) || finger_idx >= Hand::NUM_FINGERS)
    return; // buh bye
  const finger_state_t *p = (const finger_state_t *)payload;
  sandia_hand_msgs::RawFingerState *rfs = &g_raw_finger_state; // save typing
  rfs->fmcb_time = p->fmcb_time;
  rfs->pp_time = p->pp_tactile_time;
  rfs->dp_time = p->dp_tactile_time;
  for (int i = 0; i < 6; i++)
    rfs->pp_tactile[i] = p->pp_tactile[i];
  for (int i = 0; i < 12; i++)
    rfs->dp_tactile[i] = p->dp_tactile[i];
  rfs->pp_strain = p->pp_strain;
  for (int i = 0; i < 3; i++)
  {
    rfs->mm_accel[i] = p->fmcb_imu[i];
    rfs->mm_mag[i]   = p->fmcb_imu[i+3];
    rfs->pp_accel[i] = p->pp_imu[i];
    rfs->pp_mag[i]   = p->pp_imu[i+3];
    rfs->dp_accel[i] = p->dp_imu[i];
    rfs->dp_mag[i]   = p->dp_imu[i+3];
  }
  for (int i = 0; i < 4; i++)
  {
    rfs->pp_temp[i] = p->pp_temp[i];
    rfs->dp_temp[i] = p->dp_temp[i];
  }
  for (int i = 0; i < 3; i++)
    rfs->fmcb_temp[i] = p->fmcb_temp[i];
  rfs->fmcb_voltage = p->fmcb_voltage;
  rfs->fmcb_pb_current = p->fmcb_pb_current;
  for (int i = 0; i < 3; i++)
  {
    // ugh. electronics and firmware has joint indices flipped. fix someday.
    rfs->hall_tgt[i] = p->fmcb_hall_tgt[2-i];
    rfs->hall_pos[i] = p->fmcb_hall_pos[2-i];
    rfs->fmcb_effort[i] = p->fmcb_effort[2-i];
    g_last_fmcb_hall_pos[finger_idx][i] = p->fmcb_hall_pos[2-i]; // gross
  }
  if (g_raw_finger_state_pubs[finger_idx])
    g_raw_finger_state_pubs[finger_idx]->publish(g_raw_finger_state);
}

void rxPalmState(const uint8_t *data, const uint16_t data_len)
{
  if (!g_raw_palm_state_pub)
    return;
  const palm_state_t *p = (palm_state_t *)data;
  g_raw_palm_state.palm_time = p->palm_time;
  for (int i = 0; i < 3; i++)
  {
    g_raw_palm_state.palm_accel[i] = p->palm_accel[i];
    g_raw_palm_state.palm_gyro[i] = p->palm_accel[i];
    g_raw_palm_state.palm_mag[i] = p->palm_accel[i];
  }
  for (int i = 0; i < 7; i++)
    g_raw_palm_state.palm_temps[i] = p->palm_temps[i];
  for (int i = 0; i < 32; i++)
    g_raw_palm_state.palm_tactile[i] = p->palm_tactile[i];
  g_raw_palm_state_pub->publish(g_raw_palm_state);
}

void rxMoboState(const uint8_t *data, const uint16_t data_len)
{
  if (!g_raw_mobo_state_pub)
    return;
  const mobo_state_t *p = (mobo_state_t *)data;
  g_raw_mobo_state.mobo_time = p->mobo_time_ms;
  for (int i = 0; i < 4; i++)
    g_raw_mobo_state.finger_currents[i] = p->finger_currents[i];
  for (int i = 0; i < 3; i++)
  {
    g_raw_mobo_state.logic_currents[i] = p->logic_currents[i];
    g_raw_mobo_state.mobo_temp[i] = p->mobo_raw_temperatures[i];
  }
  g_raw_mobo_state.mobo_max_effort = p->mobo_max_effort;
  g_raw_mobo_state_pub->publish(g_raw_mobo_state);
}

static const unsigned NUM_CAMS = 2;
boost::shared_ptr<camera_info_manager::CameraInfoManager> g_cinfo[NUM_CAMS];
image_transport::CameraPublisher *g_image_pub[NUM_CAMS] = {0};
sensor_msgs::Image g_img_msg[NUM_CAMS];

void image_cb(const uint8_t cam_idx, const uint32_t frame_count, 
              const uint8_t *img_data)
{
  if (cam_idx > 1)
    return; // woah
  fillImage(g_img_msg[cam_idx], "mono8",
            Hand::IMG_HEIGHT, Hand::IMG_WIDTH, Hand::IMG_WIDTH, img_data);
  if (cam_idx == 0)
    g_img_msg[cam_idx].header.stamp = ros::Time::now();
  else
    g_img_msg[cam_idx].header.stamp = g_img_msg[0].header.stamp;
  g_img_msg[cam_idx].encoding = "mono8";
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(
                                        g_cinfo[cam_idx]->getCameraInfo()));
  ci->header.stamp = g_img_msg[0].header.stamp;
  ci->header.frame_id = "stereo_cam";
  if (g_image_pub[cam_idx])
    g_image_pub[cam_idx]->publish(g_img_msg[cam_idx], *ci);
}

int main(int argc, char **argv)
{
  // todo: this has become an abomination. factor out large blocks of this
  // initialization and either put it in a HandROS class or into a function
  // library or something.
  ros::init(argc, argv, "sandia_hand_node");
  ros::NodeHandle nh, nh_right("right"), nh_left("left"), nh_private("~");
  Hand hand;
  string hand_ip_str;
  nh_private.param<string>("ip", hand_ip_str, "10.66.171.23");
  int base_port;
  nh_private.param<int>("port", base_port, 12321);

  ROS_INFO("connecting to hand at %s using host ports %d-%d...", 
           hand_ip_str.c_str(), base_port, base_port+3);
  if (!hand.init(hand_ip_str.c_str(), base_port))
  {
    ROS_FATAL("couldn't init hand");
    return 1;
  }

  if (hand.getSide() == Hand::LEFT)
  {
    // geometry is mirrored, so mirror the allowable configuration space.
    for (int i = 0; i < Hand::NUM_FINGERS; i++)
    {
      float f = upper_limits_default[i][0];
      upper_limits_default[i][0] = -lower_limits_default[i][0];
      lower_limits_default[i][0] = -f;
    }
  }

  for (int i = 0; i < Hand::NUM_FINGERS; i++)
    for (int j = 0; j < 3; j++)
      g_last_fmcb_hall_pos[i][j] = 0; // gross

  bool use_fingers, use_cameras; // sometimes we just want fingers/cameras
  bool use_phalanges; // sometimes we don't need/want phalanges
  nh_private.param<bool>("use_fingers", use_fingers, true);
  nh_private.param<bool>("use_cameras", use_cameras, true);
  nh_private.param<bool>("use_phalanges", use_phalanges, false);
  nh_private.param<bool>("use_finger_0", g_use_finger[0], true);
  nh_private.param<bool>("use_finger_1", g_use_finger[1], true);
  nh_private.param<bool>("use_finger_2", g_use_finger[2], true);
  nh_private.param<bool>("use_finger_3", g_use_finger[3], true);
  if (!use_fingers)
    ROS_INFO("not using fingers");
  if (!use_phalanges)
    ROS_INFO("not using phalanges");

  // be sure we can ping it
  if (!hand.pingMoboMCU())
  {
    ROS_FATAL("couldn't ping hand motherboard.");
    return 1;
  }
  ROS_INFO("successfully pinged hand motherboard.");
  // before we get any further, make sure any current streams are turned off
  hand.setCameraStreaming(false, false);
  listenToHand(&hand, 0.001); // todo: something smarter
  hand.setFingerAutopollHz(0);
  listenToHand(&hand, 0.001); // todo: something smarter
  hand.setMoboStateHz(0);

  signal(SIGINT, signal_handler);
  g_cinfo[0] = boost::shared_ptr<camera_info_manager::CameraInfoManager>
                        (new camera_info_manager::CameraInfoManager(nh_left));
  g_cinfo[1] = boost::shared_ptr<camera_info_manager::CameraInfoManager>
                        (new camera_info_manager::CameraInfoManager(nh_right));
  g_cinfo[0]->setCameraName("left");
  g_cinfo[1]->setCameraName("right");
  g_cinfo[0]->loadCameraInfo("package://sandia_hand_driver/camera_calibration/stereo/left.ini");
  g_cinfo[1]->loadCameraInfo("package://sandia_hand_driver/camera_calibration/stereo/right.ini");
  image_transport::ImageTransport it(nh);
  image_transport::CameraPublisher image_pub[2];
  image_pub[0] = it.advertiseCamera("left/image_raw", 1);
  image_pub[1] = it.advertiseCamera("right/image_raw", 1);
  g_image_pub[0] = &image_pub[0];
  g_image_pub[1] = &image_pub[1];
  hand.setImageCallback(&image_cb); 
  // abomination
  ros::Publisher raw_finger_state_pubs[Hand::NUM_FINGERS];
  if (use_fingers)
    for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
      if (g_use_finger[finger_idx])
        if (!hand.fingers[finger_idx].mm.ping())
          hand.setFingerPower(finger_idx, Hand::FPS_LOW);
  listenToHand(&hand, 1.0);
  if (use_fingers)
    for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
      if (g_use_finger[finger_idx])
        if (!hand.fingers[finger_idx].mm.blBoot())
          ROS_WARN("couldn't boot finger %d. is it already booted?",finger_idx);
  listenToHand(&hand, 0.5);  // let the finger applications start
  if (use_fingers)
  {
    for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
    {
      if (!g_use_finger[finger_idx])
        continue;
      if (!hand.fingers[finger_idx].mm.ping())
      {
        ROS_FATAL("couldn't ping finger %d", finger_idx);
        shutdownHand(&hand);
        return 1;
      }
      hand.setFingerPower(finger_idx, Hand::FPS_FULL);
      listenToHand(&hand, 0.1); // wait a bit to settle after charging fmcb caps
      ROS_INFO("finger %d motor module up and running.", finger_idx);
    }
    // abomination
    if (use_phalanges)
    {
      for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
      {
        if (!g_use_finger[finger_idx])
          continue;
        if (!hand.fingers[finger_idx].pp.ping())
          hand.fingers[finger_idx].mm.setPhalangeBusPower(true);
      }
      listenToHand(&hand, 1.0);
      for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
      {
        if (!g_use_finger[finger_idx])
          continue;
        if (!hand.fingers[finger_idx].pp.blBoot())
          ROS_WARN("couldn't boot finger %d proximal phalange", finger_idx);
        if (!hand.fingers[finger_idx].dp.blBoot())
          ROS_WARN("couldn't boot finger %d distal phalange", finger_idx);
      }
      listenToHand(&hand, 0.5);
      for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
      {
        if (!g_use_finger[finger_idx])
          continue;
        if (!hand.fingers[finger_idx].pp.ping())
        {
          if (!hand.fingers[finger_idx].pp.ping())
            return perish("couldn't ping proximal phalange", &hand);
          if (!hand.fingers[finger_idx].dp.ping())
            return perish("couldn't ping distal phalange", &hand);
        }
        ROS_INFO("finger %d phalanges booted", finger_idx);
      }
    }
    if (!setJointLimitPolicy(&hand, "default", false))
      return perish("couldn't set finger joint limits", &hand);

    for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
    {
      if (!g_use_finger[finger_idx])
        continue;
      char topic_name[100];
      snprintf(topic_name, sizeof(topic_name), 
               "finger_%d/raw_state",finger_idx);
      printf("advertising topic [%s]\n", topic_name);
      raw_finger_state_pubs[finger_idx] = 
        nh.advertise<sandia_hand_msgs::RawFingerState>(topic_name, 1);
      g_raw_finger_state_pubs[finger_idx] = &raw_finger_state_pubs[finger_idx];
      // todo: some sort of auto-home sequence. for now, the fingers assume they 
      // were powered up in (0,0,0)
      listenToHand(&hand, 0.001);
      hand.fingers[finger_idx].mm.registerRxHandler(
                                MotorModule::PKT_FINGER_STATUS,
                                boost::bind(rxFingerState, finger_idx, _1, _2));
      if (use_phalanges)
      {
        if (!hand.fingers[finger_idx].mm.setPhalangeAutopoll(true))
          return perish("couldn't start phalange autopoll", &hand);
        ROS_INFO("finger %d is autopolling its phalanges", finger_idx);
      }
    }
  }
  ros::Publisher raw_mobo_state_pub = 
    nh.advertise<sandia_hand_msgs::RawMoboState>("mobo/raw_state", 1);
  g_raw_mobo_state_pub = &raw_mobo_state_pub;
 
  // if we get here, all fingers are up and running. let's start everything now
  listenToHand(&hand, 0.001);
  hand.registerRxHandler(CMD_ID_MOBO_STATUS, rxMoboState);

  // todo: ping the palm to make sure it's alive
  ros::Publisher raw_palm_state_pub =
    nh.advertise<sandia_hand_msgs::RawPalmState>("palm/raw_state", 1);
  g_raw_palm_state_pub = &raw_palm_state_pub;
  hand.palm.registerRxHandler(Palm::PKT_PALM_STATE,
                              boost::bind(rxPalmState, _1, _2));
  // abomination
  if (use_fingers)
    for (int finger_idx = 0; finger_idx < Hand::NUM_FINGERS; finger_idx++)
      if (g_use_finger[finger_idx])
        hand.setFingerControlMode(finger_idx, Hand::FCM_JOINT_POS);

  ros::Subscriber joint_commands_sub = 
    nh.subscribe<osrf_msgs::JointCommands>("joint_commands", 1, 
        boost::bind(jointCommandsCallback, &hand, _1));

  ros::Subscriber relative_joint_commands_sub = 
    nh.subscribe<sandia_hand_msgs::RelativeJointCommands>
       ("relative_joint_commands", 1, 
        boost::bind(relativeJointCommandsCallback, &hand, _1));

  ros::Subscriber finger_joint_commands_subs[Hand::NUM_FINGERS];
  ros::ServiceServer get_param_srvs[Hand::NUM_FINGERS];
  ros::ServiceServer set_param_srvs[Hand::NUM_FINGERS];
  if (use_fingers)
    for (int i = 0; i < Hand::NUM_FINGERS; i++)
    {
      if (!g_use_finger[i])
        continue;
      char topic[100];
      snprintf(topic, sizeof(topic), "finger_%d/joint_commands", i);
      finger_joint_commands_subs[i] = 
        nh.subscribe<osrf_msgs::JointCommands>(topic, 1,
          boost::bind(fingerJointCommandsCallback, &hand, i, _1));
      snprintf(topic, sizeof(topic), "finger_%d/get_parameters", i);
      get_param_srvs[i] =
        nh.advertiseService<sandia_hand_msgs::GetParameters::Request,
                            sandia_hand_msgs::GetParameters::Response>
          (topic, boost::bind(getParametersSrv, &hand, i, _1, _2));
      snprintf(topic, sizeof(topic), "finger_%d/set_parameters", i);
      set_param_srvs[i] =
        nh.advertiseService<sandia_hand_msgs::SetParameters::Request,
                            sandia_hand_msgs::SetParameters::Response>
          (topic, boost::bind(setParametersSrv, &hand, i, _1, _2));
    }

  ros::ServiceServer joint_limit_srv = 
    nh.advertiseService<sandia_hand_msgs::SetJointLimitPolicy::Request,
                        sandia_hand_msgs::SetJointLimitPolicy::Response>
      ("set_joint_limit_policy", boost::bind(setJointLimitPolicySrv, 
       &hand, _1, _2));

  ros::ServiceServer home_srv = 
    nh.advertiseService<sandia_hand_msgs::SetFingerHome::Request, 
                        sandia_hand_msgs::SetFingerHome::Response>
      ("set_finger_home", boost::bind(setHomeSrv, &hand, _1, _2));

  listenToHand(&hand, 0.001);
  if (use_fingers)
  {
    if (!hand.setFingerAutopollHz(100))
      return perish("couldn't set finger autopoll rate for hand", &hand);
    ROS_INFO("hand is autopolling its fingers");
  }
  hand.setMoboCurrentLimit(5);

  hand.setMoboStateHz(100);
  if (use_cameras)
    hand.setCameraStreaming(true, true);
  ros::spinOnce();
  ros::Time t_prev_spin = ros::Time::now();
  for (int i = 0; !g_done; i++)
  {
    hand.listen(0.01);
    if ((ros::Time::now() - t_prev_spin).toSec() > 0.01)
    {
      if (!nh.ok())
        break;
      ros::spinOnce();
      t_prev_spin = ros::Time::now();
    }
  }
  shutdownHand(&hand);
  return 0;
}

