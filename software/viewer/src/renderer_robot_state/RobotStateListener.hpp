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
#include <visualization_utils/file_access_utils.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/affordance_utils/affordance_seed_utils.hpp>
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/robot_urdf_t.hpp"
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
  public:  
    std::string _robot_name;
  private:  
    std::string _urdf_xml_string; 

    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    boost::shared_ptr<lcm::LCM> _lcm;   

    //get rid of this
    BotViewer *_viewer;
    
    bool _urdf_parsed;
  
    std::vector<std::string> _jointdof_filter_list;
  public:  
    bool _urdf_subscription_on;
    int64_t _last_state_msg_sim_timestamp; 
    int64_t _last_state_msg_system_timestamp; 
    bool _end_pose_received;
    drc::robot_state_t _received_endpose;

    std::map<std::string,Eigen::Vector3f> ee_forces_map;
    std::map<std::string,Eigen::Vector3f> ee_torques_map;
   
    //----------------constructor/destructor
  public:
    RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer, int operation_mode);
    ~RobotStateListener();

    boost::shared_ptr<collision::Collision_Detector> _collision_detector; 
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_robot;
    
    
    //-------------message callback
  private:
    void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);
    void handleCandidateRobotEndPoseMsg (const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);
    void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg); 
    void updateForceAndTorqueCache(const drc::force_torque_t &msg);
    
    
   public:
   
   //-------------------------------
  /*void prepareDesiredStateForEndPoseStorage(KDL::Frame &T_world_aff, 
                                   std::vector<std::string> &stateframe_ids,
                                   std::vector< std::vector<double> > &stateframe_values)
  {
    visualization_utils::prepareEndPoseForStorage(T_world_aff,_received_endpose,stateframe_ids,stateframe_values);
 
  }; // end method*/
  
  
  void setDesiredStateFromStorage(KDL::Frame &T_world_aff, 
                                   std::vector<std::string> &stateframe_ids,
                                   std::vector< std::vector<double> > &stateframe_values)
  {
  
    drc::robot_state_t msg;     
    visualization_utils::decodeEndPoseFromStorage(T_world_aff,stateframe_ids,stateframe_values,msg);
    handleCandidateRobotEndPoseMsg(NULL," ",&msg);
    //std::string channel = "STORED_ROBOT_ENDPOSE";
    //_lcm->publish(channel, &msg);  // Msg used to update plan cache in KEYFRAME ADJUSTMENT ENGINE. 
 
  };
    


  }; //class RobotStateListener

} //end namespace renderer_robot_state


#endif //RENDERER_ROBOTSTATE_ROBOTSTATELISTENER_HPP
