//
// Created by manuelli on 2/10/17.
//

#ifndef CONTROL_FOOTSTEP_PLAN_H
#define CONTROL_FOOTSTEP_PLAN_H

#include "drc/footstep_t.hpp"
#include "drake/systems/robotInterfaces/Side.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <memory>


// stores footstep plan that will be used in the walking planner.
namespace plan_eval {

  class Footstep {
  public:
    Footstep(){};
    Footstep(const drc::footstep_t &msg);
    Side getSide() const;
    const Eigen::Isometry3d &getPose() const;
  private:
    Side side_;
    Eigen::Isometry3d pose_;
  };


  class FootstepPlan {
  public:

    std::vector <std::shared_ptr<Footstep>> footsteps_;
    int next_footstep_idx_;
    int num_footsteps_;
    std::vector <drc::footstep_t> footstep_msgs_;

    FootstepPlan(){};
    FootstepPlan(std::vector <drc::footstep_t> footstep_msgs);
    void incrementCounter();
    std::shared_ptr <Footstep> getNextFootstep();
    std::shared_ptr<const Footstep> getNextFootstep() const;
    const drc::footstep_t & getNextFootstepMsg() const;
    bool hasNextFootstep() const;
    Side sideOfNextFootstep() const;
    void printDebugInfo() const;
  private:

  };
}// plan_eval


#endif //CONTROL_FOOTSTEP_PLAN_H
