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
    config = Drake::getDrakePath() + std::string("/../../config/atlas_sim_mit/plan_eval_config_atlas.yaml");
    urdf = Drake::getDrakePath() + std::string("/examples/Atlas/urdf/atlas_minimal_contact.urdf");
  }
  std::cout << "Using urdf: " << urdf << std::endl;
  std::cout << "Using config: " << config << std::endl;

  PlanEval pe(urdf, config);
  pe.Start();
  while (true)
    ;
}
