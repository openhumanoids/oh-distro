#include <stdio.h>
#include <bot_core/bot_core.h>
#include <lcmtypes/isam_slam.hpp>
#include <lcmtypes/bot_core.hpp>
#include <lcm_utils/lcm_utils.hpp>
#include <ConciseArgs>
#include <vector>
#include <string>

using namespace std;

int main(int argc, char** argv)
{
  string logfileName;
  string trajLogFileName;
  string outFileName;
  ConciseArgs opt(argc, argv, "logFname trajLogFname outFname");

  double xyz[3] = { 0 };
  double rpy[3] = { 0 };

  opt.add(xyz[0], "x", "trans_x");
  opt.add(xyz[1], "y", "trans_y");
  opt.add(xyz[2], "z", "trans_z");
  opt.add(rpy[0], "r", "roll", "");
  opt.add(rpy[1], "p", "pitch", "");
  opt.add(rpy[2], "w", "yaw", "(yaw)");

  opt.parse(logfileName, trajLogFileName, outFileName);

  for (int i = 0; i < 3; i++)
    rpy[i] = bot_to_radians(rpy[i]);

  double quat[4];
  bot_roll_pitch_yaw_to_quat(rpy, quat);

  BotTrans poseModifierTrans;
  bot_trans_set_from_quat_trans(&poseModifierTrans, quat, xyz);

  printf("splicing poses from trajectory in %s\n", trajLogFileName.c_str());
  printf("into log %s\n", logfileName.c_str());
  printf("and writing it to %s\n", outFileName.c_str());
  printf("Transforming poses by:\n");
  bot_trans_print_trans(&poseModifierTrans);
  printf("\n");

  vector<isam_slam::trajectory_t> trajMsgs = lcm_utils::loadMsgsFromLog<isam_slam::trajectory_t>(trajLogFileName,
      "SLAM_COMPLETE_TRAJECTORY");

  // Open the log file.
  lcm::LogFile log(logfileName, "r");
  if (!log.good()) {
    perror("LogFile");
    std::cerr << "couldn't open log file: " << logfileName << std::endl;
    return 1;
  }

  isam_slam::trajectory_t & traj = trajMsgs.back();
  //  lcm::LCM out_log("file://" + outFileName + "?mode=w");
  lcm::LogFile out_log(outFileName, "w");

  int pose_ind = 0;
  while (1) {
    // Read a log event.
    const lcm::LogEvent *event = log.readNextEvent();
    if (!event)
      break;
    if (event->channel == "POSE")
      continue;
    else if (event->channel == "LASER") {
      bot_core::planar_lidar_t lmsg;
      lmsg.decode(event->data, 0, event->datalen);
      while (pose_ind < traj.poses.size() && traj.poses[pose_ind].utime <= lmsg.utime) {
        //throw the pose in right before the laser message
        bot_core::pose_t pmsg;
        pmsg.utime = traj.poses[pose_ind].utime;

        BotTrans transformedPose;
        bot_trans_set_from_quat_trans(&transformedPose, traj.poses[pose_ind].orientation, traj.poses[pose_ind].pos);

        //apply the passed in transform
        bot_trans_apply_trans(&transformedPose, &poseModifierTrans);

        for (int j = 0; j < 3; j++) {
          pmsg.pos[j] = transformedPose.trans_vec[j];
          pmsg.orientation[j] = transformedPose.rot_quat[j];
          pmsg.vel[j] = 0;
          pmsg.rotation_rate[j] = 0;
        }
        pmsg.orientation[3] = transformedPose.rot_quat[3]; //copy 4th

        lcm::LogEvent pevent;
        pevent.timestamp = event->timestamp - 1;
        pevent.channel = "POSE";
        pevent.datalen = pmsg.getEncodedSize();
        pevent.data = malloc(pevent.datalen);
        pmsg.encode(pevent.data, 0, pevent.datalen);
        //        out_log.publish("POSE", &pmsg);
        out_log.writeEvent(&pevent);
        free(pevent.data);
        pose_ind++;
      }
      if (pose_ind == traj.poses.size())
        break;
    }
//    out_log.publish(event->channel, event->data, event->datalen);

    lcm::LogEvent wevent = *event; //copy event structure so that event num can be written to
    out_log.writeEvent(&wevent);

  }

  return 0;
}
