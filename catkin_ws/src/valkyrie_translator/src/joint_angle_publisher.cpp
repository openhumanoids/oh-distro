#include <cstdlib>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/joint_angles_t.hpp"
#include <boost/shared_ptr.hpp>

int main(int argc, char** argv)
{
  if (argc == 1){
    printf("Usage: \n\t Pairs of <jointname> <jointval>\n");
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
  drc::joint_angles_t lcm_pose_msg;
  lcm_pose_msg.num_joints = (argc-1)/2;
  lcm_pose_msg.joint_name.assign(lcm_pose_msg.num_joints, "");
  lcm_pose_msg.joint_position.assign(lcm_pose_msg.num_joints, 0.);
  for (unsigned int i = 0; i < lcm_pose_msg.num_joints; i++){
    lcm_pose_msg.joint_name[i] = std::string(argv[1+i*2]);
    lcm_pose_msg.joint_position[i] = atof(argv[2+i*2]);
    printf("%d: %s, %f\n", i, lcm_pose_msg.joint_name[i].c_str(), lcm_pose_msg.joint_position[i]);
  }
  lcm->publish("VAL_TRANSLATOR_JOINT_COMMAND", &lcm_pose_msg);
  


  return 0;
}
