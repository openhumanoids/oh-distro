#include <signal.h>
#include <cstdio>
#include <vector>
#include <ros/ros.h>
#include <sandia_hand_msgs/RawPalmState.h>
#include "sandia_hand/loose_palm.h"
#include <sandia_hand_msgs/GetParameters.h>
#include <sandia_hand_msgs/SetParameters.h>
#include "sandia_hand/palm_state.h"
using namespace sandia_hand;
using std::string;
using std::vector;

/////////////////////////////////////////////////////////////////////////
sandia_hand_msgs::RawPalmState g_raw_palm_state;
ros::Publisher *g_raw_palm_state_pub = NULL;
bool g_done = false;
/////////////////////////////////////////////////////////////////////////

void signal_handler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    g_done = true;
}

void listenToPalm(LoosePalm *palm, const float seconds)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    palm->listen(0.01);
    if ((ros::Time::now() - t_start).toSec() > seconds)
      break;
  }
}

int perish(const char *msg)
{
  ROS_FATAL("%s", msg);
  return 1;
}

bool getParametersSrv(LoosePalm *palm,
                      sandia_hand_msgs::GetParameters::Request &req,
                      sandia_hand_msgs::GetParameters::Response &res)
{
  ROS_INFO("get parameters");
  const vector<sandia_hand::Param> params = palm->getParams();
  res.parameters.resize(params.size());
  for (size_t i = 0; i < params.size(); i++)
  {
    res.parameters[i].name = params[i].getName();
    if (params[i].getType() == Param::PARAM_INT)
    {
      res.parameters[i].val_type = sandia_hand_msgs::Parameter::INTEGER;
      res.parameters[i].i_val = params[i].getIntVal();
    }
    else
    {
      res.parameters[i].val_type = sandia_hand_msgs::Parameter::FLOAT;
      res.parameters[i].f_val = params[i].getFloatVal();
    }
  }
  return true;
}

bool setParametersSrv(LoosePalm *palm,
                      sandia_hand_msgs::SetParameters::Request &req,
                      sandia_hand_msgs::SetParameters::Response &res)
{
  ROS_INFO("get parameters");
  bool all_ok = true;
  for (size_t i = 0; i < req.parameters.size(); i++)
  {
    const sandia_hand_msgs::Parameter *p = &req.parameters[i]; // save typing
    if (p->val_type == sandia_hand_msgs::Parameter::INTEGER) //Param::PARAM_INT)
      all_ok &= palm->setParamInt(p->name, (int32_t)p->i_val);
    else
      all_ok &= palm->setParamFloat(p->name, (float)p->f_val);
  }
  return all_ok;
}

void rxPalmState(const uint8_t *payload, const uint16_t payload_len)
{
  //printf("rxPalmState: %d bytes\n", payload_len);
  if (payload_len < sizeof(palm_state_t))
    return; // buh bye
  const palm_state_t *p = (const palm_state_t *)payload;
  sandia_hand_msgs::RawPalmState *rps = &g_raw_palm_state; // save typing
  rps->palm_time = p->palm_time;
  for (int i = 0; i < PALM_STATE_NUM_TAXELS; i++)
    rps->palm_tactile[i] = p->palm_tactile[i];
  for (int i = 0; i < 3; i++)
  {
    rps->palm_accel[i] = p->palm_accel[i];
    rps->palm_mag[i]   = p->palm_mag[i];
    rps->palm_gyro[i]  = p->palm_gyro[i];
  }
  for (int i = 0; i < PALM_STATE_NUM_TEMPS; i++)
    rps->palm_temps[i] = p->palm_temps[i];
  if (g_raw_palm_state_pub)
    g_raw_palm_state_pub->publish(g_raw_palm_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sandia_hand_loose_palm_node");
  ros::NodeHandle nh, nh_private("~");
  LoosePalm palm;
  std::string serial_device;
  nh_private.param<string>("serial_device", serial_device, "/dev/ttyUSB0");
  if (!palm.init(serial_device.c_str()))
  {
    ROS_FATAL("couldn't init palm serial port");
    return 1;
  }
  if (!palm.blBoot())
    ROS_WARN("couldn't boot palm. is it already booted?");
  else
    ROS_INFO("booted palm.");
  if (!palm.ping()) // be sure we can ping it
  {
    ROS_FATAL("couldn't ping palm");
    return 1;
  }
  ROS_INFO("successfully pinged finger motor module.");
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  listenToPalm(&palm, 0.5);
  ros::Publisher raw_palm_state_pub = 
      nh.advertise<sandia_hand_msgs::RawPalmState>("raw_palm_state", 1);
  g_raw_palm_state_pub = &raw_palm_state_pub;
  palm.registerRxHandler(Palm::PKT_PALM_STATE, rxPalmState);
  ros::ServiceServer param_dump_srv =
    nh.advertiseService<sandia_hand_msgs::GetParameters::Request,
                        sandia_hand_msgs::GetParameters::Response>
      ("get_parameters", boost::bind(getParametersSrv, &palm, _1, _2));
  ros::ServiceServer param_set_srv =
    nh.advertiseService<sandia_hand_msgs::SetParameters::Request,
                        sandia_hand_msgs::SetParameters::Response>
      ("set_parameters", boost::bind(setParametersSrv, &palm, _1, _2));
  ros::spinOnce();
  ros::Time t_prev_spin = ros::Time::now();
  while (!g_done)
  {
    palm.listen(0.01);
    if ((ros::Time::now() - t_prev_spin).toSec() > 0.01)
    {
      if (!nh.ok())
        break;
      ros::spinOnce();
      t_prev_spin = ros::Time::now();
      palm.pollState();
    }
  }
  return 0;
}

