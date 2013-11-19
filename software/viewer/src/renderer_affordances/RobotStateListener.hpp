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


  // MIT's version of this struct used by BDI:
  struct PelvisServoParams
  {
    float pelvis_height;
    float pelvis_yaw;
    float pelvis_pitch;
    float pelvis_roll;
    float com_v0;
    float com_v1;  
    
    PelvisServoParams() :
            pelvis_height(0.85f),
            pelvis_yaw(0.0f),
            pelvis_pitch(0.0f),
            pelvis_roll(0.0f),
            com_v0(0.0f),
            com_v1(0.0f)
    {}  
  } ; //AtlasBehaviorPelvisServoParams

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
    
    PelvisServoParams currentPelvisState; // the current parameters reported by the robot
    bool currentPelvisState_received; // have received a message with these params?

    drc::robot_state_t _last_robotstate_msg;
    bool _robot_state_received;
    bool _urdf_parsed;
    bool _urdf_subscription_on;
    // Using GlKinematicBody as cache for fk queries.
    // TODO: should have a bare bones KinematicBody class when no drawing(parsing meshes etc) is required.
    boost::shared_ptr<visualization_utils::GlKinematicBody> _gl_robot; 
    boost::shared_ptr<visualization_utils::GlKinematicBody> _gl_robot_tmp; 

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
    void handleAtlasStatus(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::atlas_status_t *msg)
    {
      currentPelvisState.pelvis_height = msg->manipulate_feedback.internal_desired.pelvis_height;     
      currentPelvisState.pelvis_yaw = msg->manipulate_feedback.internal_desired.pelvis_yaw;
      currentPelvisState.pelvis_pitch = msg->manipulate_feedback.internal_desired.pelvis_pitch;
      currentPelvisState.pelvis_roll = msg->manipulate_feedback.internal_desired.pelvis_roll;
      currentPelvisState.com_v0 = msg->manipulate_feedback.internal_desired.com_v0;
      currentPelvisState.com_v1 = msg->manipulate_feedback.internal_desired.com_v1;
      currentPelvisState_received = true;
    };			    

    }; //class RobotStateListener

} //end namespace 


#endif //RENDERER_AFFORDANCES_ROBOTSTATELISTENER_HPP
