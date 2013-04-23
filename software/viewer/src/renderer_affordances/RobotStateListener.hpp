#ifndef RENDERER_AFFORDANCES_ROBOTSTATELISTENER_HPP
#define RENDERER_AFFORDANCES_ROBOTSTATELISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include <bot_vis/bot_vis.h>
#include "renderer_affordances.hpp" // has definition of RendererAffordances struc

namespace renderer_affordances
{
  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class RobotStateListener
  {
    //--------fields
  private:
    std::string _robot_name;
    boost::shared_ptr<lcm::LCM> _lcm;    
    RendererAffordances* _parent_renderer; 
    
    //BotViewer *_viewer;

    //----------------constructor/destructor
  public:
   // RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer);
		RobotStateListener(RendererAffordances* parent_renderer);
    ~RobotStateListener();
    
    KDL::Frame T_body_world; // current body origin in world frame
    long last_state_msg_timestamp;
    drc::robot_state_t last_robotstate_msg;
    bool _robot_state_received;

    //-------------message callback
  private:
    void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);

    }; //class RobotStateListener

} //end namespace 


#endif //RENDERER_AFFORDANCES_ROBOTSTATELISTENER_HPP
