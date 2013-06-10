#ifndef RENDERER_AFFORDANCES_ROBOTSTATELISTENER_HPP
#define RENDERER_AFFORDANCES_ROBOTSTATELISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
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
    std::string _urdf_xml_string;
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    boost::shared_ptr<lcm::LCM> _lcm;    
    RendererAffordances* _parent_renderer; 
	int64_t _last_state_msg_system_timestamp;
	int object_update_counter;
    
    //BotViewer *_viewer;

    //----------------constructor/destructor
  public:
   // RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer);
		RobotStateListener(RendererAffordances* parent_renderer);
    ~RobotStateListener();
    
    KDL::Frame T_body_world; // current body origin in world frame

    drc::robot_state_t _last_robotstate_msg;
    bool _robot_state_received;
    bool _urdf_parsed;
    bool _urdf_subscription_on;
    // Using GlKinematicBody as cache for fk queries.
    // TODO: Feature dev after VRC  (Sisir, 5th June)
    // We should have a bare bones KinematicBody class when no drawing(parsing meshes etc) is required.
    boost::shared_ptr<visualization_utils::GlKinematicBody> _gl_robot; 


	// updates the position of objects if there are double melded (associated sticky hand is melded to true hand and object itself also melded)
	// object tracking via FK
	void updateGraspedObjectPosesViaFK(void);

    //-------------message callback
  private:
    void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);
    void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg); 

    }; //class RobotStateListener

} //end namespace 


#endif //RENDERER_AFFORDANCES_ROBOTSTATELISTENER_HPP
