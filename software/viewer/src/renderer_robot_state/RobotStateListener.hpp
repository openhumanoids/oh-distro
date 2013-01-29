#ifndef RENDERER_ROBOTSTATE_ROBOTSTATELISTENER_HPP
#define RENDERER_ROBOTSTATE_ROBOTSTATELISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include "lcmtypes/bot_core.hpp"
#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>
#include <path_util/path_util.h>

#include <Eigen/Dense>
#include <collision/collision_detector.h>
#include <collision/collision_object_box.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif


namespace renderer_robot_state
{

  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class RobotStateListener
  {
    //--------fields
  private:
    std::string _robot_name;
    std::string _urdf_xml_string; 

    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    boost::shared_ptr<lcm::LCM> _lcm;   

    //get rid of this
    BotViewer *_viewer;
    
    bool _urdf_parsed;
    
  public:  
    bool _urdf_subscription_on;

   
    //----------------constructor/destructor
  public:
    RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer);
    ~RobotStateListener();

    boost::shared_ptr<collision::Collision_Detector> _collision_detector; 
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_robot;
    
    
    //-------------message callback
  private:
    void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);
    void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg);   

  }; //class RobotStateListener

} //end namespace renderer_robot_state


#endif //RENDERER_ROBOTSTATE_ROBOTSTATELISTENER_HPP
