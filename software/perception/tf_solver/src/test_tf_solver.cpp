#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <lcmtypes/bot_core.hpp>

#include "tf_solver/tf_solver.hpp"

using namespace std;

int main(int argc, char ** argv)
{

  boost::shared_ptr<lcm::LCM> _lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  boost::shared_ptr<forward_kinematics::TfSolver> localTfSolver = boost::shared_ptr<forward_kinematics::TfSolver>(new forward_kinematics::TfSolver(_lcm));

  bot_core::rigid_transform_t tf;

  while(0 == _lcm->handle()){
    if(localTfSolver->getLinkTf ("rotating_laser_link",tf)){
      _lcm->publish("BODY_TO_ROTATING_SCAN",&tf);
    }
    // These don't change currently:
    //if(localTfSolver->getLinkTf ("camera_link",tf)){
    //  cout << "cam" <<endl;
    //  _lcm->publish("BODY_TO_CAMERA",&tf);
    //}
    //if(localTfSolver->getLinkTf ("base_laser_link",tf)){
    //  cout << "base laser" <<endl;
    //  _lcm->publish("BODY_TO_BASE_SCAN",&tf);
    //}
  };
  return 0;
}
