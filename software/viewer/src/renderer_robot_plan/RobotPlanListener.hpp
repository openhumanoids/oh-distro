#ifndef RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP
#define RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include <bot_vis/bot_vis.h>
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include <visualization_utils/file_access_utils.hpp>

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
    
    //get rid of this
    BotViewer *_viewer;

    
    bool _urdf_parsed;
    bool _urdf_subscription_on;
    

    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_robot;
    //boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _base_gl_robot;
    //----------------constructor/destructor
    
    int _in_motion_keyframe_index;
  public:
    bool _is_manip_plan;
    int64_t _last_plan_msg_timestamp; 
    RobotPlanListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer);
    ~RobotPlanListener();
    
   // boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    //std::vector< boost::shared_ptr<visualization_utils::GlKinematicBody> >  _gl_robot_list;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_robot_list;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_robot_keyframe_list;
    std::vector< int64_t >  _keyframe_timestamps;
    
    //The following are local in-motion copies that appear on doubleclk of a keyframe and can be moved around via markers
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_left_hand;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> _gl_right_hand;
    //-------------message callback
    
    drc::robot_plan_t revieved_plan_;
    
    void commit_robot_plan(int64_t utime,std::string &channel);

    
    void set_in_motion_hands_state(int index)
    {
 
      KDL::Frame T_base_palm,T_world_palm_l,T_world_palm_r,T_world_base_l,T_world_base_r;
     
      T_base_palm = KDL::Frame::Identity();
      T_base_palm.M = KDL::Rotation::RPY(0,-M_PI/2,0);
      _gl_robot_keyframe_list[index]->get_link_frame("left_palm",T_world_palm_l);
      _gl_robot_keyframe_list[index]->get_link_frame("right_palm",T_world_palm_r);

      T_world_base_l = T_world_palm_l*(T_base_palm.Inverse());
      T_world_base_r = T_world_palm_r*(T_base_palm.Inverse());

      std::map<std::string, double> jointpos_l;
      jointpos_l=_gl_left_hand->_current_jointpos;
      _gl_left_hand->set_state(T_world_base_l,jointpos_l);
      _gl_left_hand->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);

      std::map<std::string, double> jointpos_r;
      jointpos_r = _gl_right_hand->_current_jointpos;
      _gl_right_hand->set_state(T_world_base_r,jointpos_r);    
      _gl_right_hand->set_bodypose_adjustment_type((int)visualization_utils::InteractableGlKinematicBody::THREE_D);
      
      _in_motion_keyframe_index = index;
    
    };
    
    bool is_in_motion(int index) {  
        return (index==_in_motion_keyframe_index);
    };
    
    
    int64_t get_keyframe_timestamp(int index) {
        return _keyframe_timestamps[index];
    };
    
  private:
    void handleRobotPlanMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_plan_t* msg);
		void handleManipPlanMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_plan_w_keyframes_t* msg);
   void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg);    
			    
	 bool load_hand_urdfs(std::string &_left_hand_urdf_xml_string,std::string &_right_hand_urdf_xml_string);


}; //class RobotPlanListener
  

} //namespace renderer_robot_plan


#endif //RENDERER_ROBOTPLAN_ROBOTPLANLISTENER_HPP
