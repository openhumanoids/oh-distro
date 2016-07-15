#include "plan_eval.h"
#include <string>

int main(int argc, const char *argv[]) {
  if (argc != 3 && argc != 1) {
    std::cerr << "usage: sf_plan_eval <urdf_path> <config.yaml>\n";
    return -1;
  }
  
  std::string urdf, config;
  if (argc == 3) {
    urdf = std::string(argv[1]);
    config = std::string(argv[2]);
  } 
  else {
    config = Drake::getDrakePath() + std::string("/examples/Atlas/config/control_config_sim.yaml");
    urdf = Drake::getDrakePath() + std::string("/examples/Atlas/urdf/atlas_minimal_contact.urdf");
  }
  PlanEval pe(urdf, config);
  pe.Start();
  while (true)
    ;
}
