#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>
#include <lcmtypes/mav/rpy_t.hpp>
#include <lcmtypes/mav/pose_t.hpp>
#include <lcmtypes/mav/ins_t.hpp>
#include <bot_core/bot_core.h>
#include <Eigen/Dense>

using namespace std;

class RPYTranslator {
public:
  lcm::LCM lcm;
  string rpy_chan;

  RPYTranslator() :
      lcm()
  {
  }

  void publishRPY(const double quat[4], int64_t utime)
  {
    fprintf(stderr, ".");
    mav::rpy_t rpy_msg;
    rpy_msg.utime = utime;
    bot_quat_to_roll_pitch_yaw(quat, rpy_msg.rpy);

    Eigen::Map<Eigen::Vector3d> rpy_vec = Eigen::Map<Eigen::Vector3d>(rpy_msg.rpy);
    rpy_vec = bot_to_degrees(rpy_vec);
    lcm.publish(rpy_chan, &rpy_msg);
  }

  void poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const mav::pose_t* msg)
  {
    publishRPY(msg->orientation, msg->utime);
  }

  void insHandler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const mav::ins_t* msg)
  {
    publishRPY(msg->quat, msg->utime);
  }
};

int main(int argc, char** argv)
{

  RPYTranslator rpy_trans = RPYTranslator();

  string poseChannel = "STATE_ESTIMATOR_POSE";
  string insChannel = "";
  string rpySuffix = "_RPY";
  ConciseArgs opt(argc, argv);
  opt.add(poseChannel, "p", "pose_channel", "pose channel to set rpy");
  opt.add(insChannel, "i", "ins_channel", "ins channel to set rpy");
  opt.add(rpySuffix, "r", "rpy_suffix", "rpy suffix");
  opt.parse();

  if (opt.wasParsed("ins_channel")) {
    fprintf(stderr, "subscribing to %s\n", insChannel.c_str());
    rpy_trans.rpy_chan = insChannel + rpySuffix;
    rpy_trans.lcm.subscribe(insChannel, &RPYTranslator::insHandler, &rpy_trans);
  }
  else {
    fprintf(stderr, "subscribing to %s\n", poseChannel.c_str());
    rpy_trans.rpy_chan = poseChannel + rpySuffix;
    rpy_trans.lcm.subscribe(poseChannel, &RPYTranslator::poseHandler, &rpy_trans);
  }

  while (1)
    rpy_trans.lcm.handle();

  return 0;
}
