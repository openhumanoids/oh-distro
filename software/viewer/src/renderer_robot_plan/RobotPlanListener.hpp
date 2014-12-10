#ifndef RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP
#define RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include "lcmtypes/drc/aff_indexed_robot_plan_t.hpp"
#include "lcmtypes/drc/deprecated_footstep_plan_t.hpp"
#include "lcmtypes/drc/robot_plan_t.hpp"
#include "lcmtypes/drc/robot_plan_w_keyframes_t.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/robot_urdf_t.hpp"
#include "lcmtypes/drc/system_status_t.hpp"
#include "lcmtypes/drc/controller_status_t.hpp"
#include <bot_vis/bot_vis.h>
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include <visualization_utils/file_access_utils.hpp>
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/stickyhand_utils/sticky_hand_utils.hpp>
#include <visualization_utils/affordance_utils/affordance_seed_utils.hpp>
#include <sys/time.h>
#include <time.h>

namespace renderer_robot_plan 
{

  class RobotPlanListener
  {
    //--------fields
    
   public:
    std::string _robot_name;
   
   private:      
    std::string _urdf_xml_string;   
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    boost::shared_ptr<lcm::LCM> _lcm;    
    BotViewer *_viewer;
    bool _urdf_parsed;
    bool _urdf_subscription_on;
    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_robot;
    int _in_motion_keyframe_index;

  public:
    bool _is_manip_plan;
    bool _is_manip_map;
    bool _current_plan_committed; // is set to true when a plan is committed but is still retained in the viewer for visualization.
    int64_t _last_plan_msg_timestamp; 
    RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer, int operation_mode);
    ~RobotPlanListener();
    
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_robot_list;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_robot_keyframe_list;
    std::vector< int64_t >  _keyframe_timestamps;
    std::vector< int >  _breakpoint_indices;
    int _active_breakpoint;
    int _num_breakpoints;
    int _retractable_cycle_counter;
    
    drc::robot_state_t _last_robotstate_msg;
    drc::robot_plan_t _received_plan;
    
    drc::aff_indexed_robot_plan_t _received_map;
    
    int64_t _controller_utime;
    int8_t _controller_status;
  
   
    
  private:
   void handleManipPlanMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_plan_w_keyframes_t* msg);
		      
   void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg);
				 
   void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);	
}; //class RobotPlanListener
  

} //namespace renderer_robot_plan


#endif //RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP
