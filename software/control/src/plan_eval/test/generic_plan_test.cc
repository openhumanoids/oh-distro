//
// Created by manuelli on 2/8/17.
//

// this was installed by a library hopefully
#include "../walking_plan.h"
#include "../manip_plan.h"
#include "drake/Path.h"
#include "../utils/simple_timer.h"

#include <string>

int main(int argc, const char *argv[]) {
  using namespace plan_eval;
  using namespace plan_eval::utils;

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

//  ManipPlan manip_plan = ManipPlan(urdf, config);
//  WalkingPlan walking_plan = WalkingPlan(urdf, config)
  SimpleTimer simple_timer = SimpleTimer();
  simple_timer.Start();
  std::shared_ptr<plan_eval::GenericPlan> new_plan_ptr(new plan_eval::WalkingPlan(urdf, config));
  std::chrono::milliseconds elapsed_time = simple_timer.Elapsed();
  std::cout << "elapsed time in ms = " << elapsed_time.count() << std::endl;
  new_plan_ptr->SimpleTest();

  plan_eval::WalkingPlan walking_plan(urdf, config);
}
