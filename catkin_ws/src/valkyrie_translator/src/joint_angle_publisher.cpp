#include <cstdlib>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/robot_command_t.hpp"
#include <boost/shared_ptr.hpp>

int main(int argc, char** argv)
{
  if (argc == 1){
    printf("Usage: \n\t Pairs of <jointname> <jointforce>\n");
    printf("\t (Eg: rightKneePitch 0.1 leftKneePitch 0.1\n");
    return 0;
  }
  if ((argc-1)%2 != 0){
    printf("Must supply properly paired arguments!\n");
    return 0;
  }

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  // assemble and spit out message
  drc::robot_command_t lcm_pose_msg;
  lcm_pose_msg.num_joints = (argc-1)/2;

  lcm_pose_msg.joint_commands.resize(lcm_pose_msg.num_joints);

  for (unsigned int i = 0; i < lcm_pose_msg.num_joints; i++){
    lcm_pose_msg.joint_commands[i].joint_name = std::string(argv[1+i*2]);
    lcm_pose_msg.joint_commands[i].position = 0.0;
    lcm_pose_msg.joint_commands[i].velocity = 0.0;
    lcm_pose_msg.joint_commands[i].effort = atof(argv[2+i*2]);

    lcm_pose_msg.joint_commands[i].k_q_p = 0.0;
    lcm_pose_msg.joint_commands[i].k_q_i = 0.0;
    lcm_pose_msg.joint_commands[i].k_qd_p = 0.0;
    lcm_pose_msg.joint_commands[i].k_f_p = 0.0;
    lcm_pose_msg.joint_commands[i].ff_qd = 0.0;
    lcm_pose_msg.joint_commands[i].ff_qd_d = 0.0;
    lcm_pose_msg.joint_commands[i].ff_f_d = 1.0;
    lcm_pose_msg.joint_commands[i].ff_const = 0.0;

    printf("%d: %s, %f\n", i, lcm_pose_msg.joint_commands[i].joint_name.c_str(), lcm_pose_msg.joint_commands[i].effort);
  }
  lcm->publish("NASA_COMMAND", &lcm_pose_msg);
  


  return 0;
}
