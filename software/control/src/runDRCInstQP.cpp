#include <mex.h>
#include "threadedController.cpp"
#include "drake/systems/controllers/controlUtil.h"
#include "drake/systems/controllers/InstantaneousQPController.h"

int main(int argc, char** argv) {
  const char* drc_path = std::getenv("DRC_BASE");
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }
  std::string urdf;
  std::string urdf_mods;
  std::string command_channel;
  std::string behavior_channel;
  std::string control_config_filename;
  int max_infocount;
  if (argc > 1 && 0==strcmp(argv[1], "--help")){
    printf("Usage:\n");
    printf("\tdrc-inst-qp <urdf=model_min_contact> <urdf-modifications=none> <control_config_filename> <command_channel=<ATLAS_COMMAND> <robot_behavior_channel=ATLAS_BEHAVIOR_COMMAND> <max_infocount = -1>\n");
    exit(0);
  }
  if (argc < 2){
    urdf = std::string(drc_path) + "/software/models/atlas_v5/model_minimal_contact.urdf";
  } else {
    urdf = std::string(argv[1]);
  }
  if (argc < 3){
    urdf_mods = std::string("none");
  } else {
    urdf_mods = std::string(argv[2]);
  }
  if (argc < 4){
    control_config_filename = std::string(drc_path) + "/software/drake/drake/examples/Atlas/config/control_config_sim.yaml";
  } else {
    control_config_filename = std::string(argv[3]);
  }
  if (argc < 5){
    command_channel = "ATLAS_COMMAND";
  } else {
    command_channel = std::string(argv[4]);
  }
  if (argc < 6){
    behavior_channel = "ATLAS_BEHAVIOR_COMMAND";
  } else {
    behavior_channel = std::string(argv[5]);
  }
  if (argc < 7){
    max_infocount = -1;
  } else {
    max_infocount = atoi(argv[6]);
  }

  std::unique_ptr<RigidBodyTree> robot =
      std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf));
  if (urdf_mods != "none"){
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
  if (ctrl_opts->atlas_command_channel.size() == 0) {
    throw std::runtime_error("Atlas command channel cannot be empty");
  }
  if (ctrl_opts->robot_behavior_channel.size() == 0) {
    throw std::runtime_error("Robot behavior channel cannot be empty");
  }

  controllerLoop(&instqp, ctrl_opts);
  return -1;
}