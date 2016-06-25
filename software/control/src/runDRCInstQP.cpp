#include "threadedController.cpp"
#include "drake/systems/controllers/controlUtil.h"
#include "drake/systems/controllers/InstantaneousQPController.h"

#include <ConciseArgs>

int main(int argc, char** argv) {
  const char* drc_path = std::getenv("DRC_BASE");
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }
  std::string urdf = std::string(drc_path) + "/software/models/atlas_v5/model_minimal_contact.urdf";
  std::string urdf_mods;
  std::string command_channel = "ATLAS_COMMAND"; 
  std::string behavior_channel = "ATLAS_BEHAVIOR_COMMAND";
  std::string control_config_filename = std::string(drc_path) + "/software/drake/drake/examples/Atlas/config/control_config_sim.yaml";
  bool fixedBase = false;
  bool publishControllerState = false;
  int max_infocount = -1;

  ConciseArgs parser(argc, argv);
  parser.add(urdf, "u", "urdf", "Robot URDF");
  parser.add(urdf_mods, "um", "urdf-mods", "URDF modifications file");
  parser.add(control_config_filename, "c", "control-config-filename", "Control config filename");
  parser.add(command_channel, "lc", "lcm-command-channelname", "Command channel (LCM)");
  parser.add(behavior_channel, "lb", "lcm-behavior-channelname", "Behavior channel (LCM)");
  parser.add(max_infocount, "i", "max-infocount", "Max infocount before controller safing");
  parser.add(fixedBase, "fb", "fixedBase", "set to true if you want to launch controller with fixedBase model");
  parser.add(publishControllerState, "pub", "publishControllerState", "set to true if you want to publish the controller state");
  parser.parse();


  DrakeJoint::FloatingBaseType floatingBaseType;
  if (fixedBase){
    std::cout << "using a FIXED base robot in QP controller" << std::endl;
    floatingBaseType = DrakeJoint::FIXED;
  } else{
    floatingBaseType = DrakeJoint::ROLLPITCHYAW;
    std::cout << "using a FLOATING base robot in QP controller" << std::endl;
  }

  std::unique_ptr<RigidBodyTree> robot =
      std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, floatingBaseType));
  if (parser.wasParsed("urdf-mods")){
    applyURDFModifications(robot, urdf_mods);
  }

  InstantaneousQPController instqp(std::move(robot), control_config_filename);

  std::cout << "InstQP instantiated: " << std::endl;
  std::cout << "\t urdf: " << urdf << std::endl;
  std::cout << "\t urdf_mods: " << urdf_mods << std::endl;
  std::cout << "\t control_config " << control_config_filename << std::endl;
  std::cout << "\t Command channel: " << command_channel << std::endl;
  std::cout << "\t Behavior channel: " << behavior_channel << std::endl;
  std::cout << "\t Max Infocount: " << max_infocount << std::endl;
  
  std::shared_ptr<ThreadedControllerOptions> ctrl_opts (new ThreadedControllerOptions());
  ctrl_opts->atlas_command_channel = command_channel;
  ctrl_opts->robot_behavior_channel = behavior_channel;
  ctrl_opts->max_infocount = max_infocount;
  ctrl_opts->publishControllerState = publishControllerState;
  ctrl_opts->fixedBase = fixedBase;
  if (ctrl_opts->atlas_command_channel.size() == 0) {
    throw std::runtime_error("Atlas command channel cannot be empty");
  }
  if (ctrl_opts->robot_behavior_channel.size() == 0) {
    throw std::runtime_error("Robot behavior channel cannot be empty");
  }

  controllerLoop(&instqp, ctrl_opts);
  return -1;
}
