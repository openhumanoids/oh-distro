#ifndef OTDF_RENDERER_ROBOT_STATE_LISTENER_H
#define OTDF_RENDERER_ROBOT_STATE_LISTENER_H

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include <bot_vis/bot_vis.h>

namespace otdf_renderer 
{
  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class RobotStateListener
  {
    //--------fields
  private:
    std::string _robot_name;
    boost::shared_ptr<lcm::LCM> _lcm;    

    //get rid of this
    BotViewer *_viewer;

    //----------------constructor/destructor
  public:
    RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer);
    ~RobotStateListener();
    
    KDL::Frame T_body_world; // current body origin in world frame
    
    //-------------message callback
  private:
    void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);

}; //class RobotStateListener

} //end namespace 


#endif //OTDF_RENDERER_ROBOT_STATE_LISTENER_H
