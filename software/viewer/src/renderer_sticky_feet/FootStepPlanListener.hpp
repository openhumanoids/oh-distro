#ifndef RENDERER_STICKYFEET_FOOTSTEPPLANLISTENER_HPP
#define RENDERER_STICKYFEET_FOOTSTEPPLANLISTENER_HPP

#include <iostream>
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
#include <visualization_utils/eigen_kdl_conversions.hpp>
#include <visualization_utils/file_access_utils.hpp>

//#include "renderer_sticky_feet.hpp"

using namespace std;
using namespace boost;
using namespace visualization_utils;

namespace renderer_sticky_feet
{

  class FootStepPlanListener
  {
    //--------fields
  private:
    //RendererStickyFeet* _parent_renderer;
    BotViewer *_viewer;
    
    boost::shared_ptr<lcm::LCM> _lcm;    

    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_stickyfoot_left;
    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_stickyfoot_right;
    
    drc::ee_goal_sequence_t revieved_plan_;
     

    std::string _left_urdf_xml_string;
    std::string _right_urdf_xml_string;


   //----------------constructor/destructor   
  public:
     std::string _robot_name;          
     std::string _left_foot_name; // foot ee names
     std::string _right_foot_name;
     Eigen::Vector3f _left_foot_offset;
     Eigen::Vector3f _right_foot_offset;
     int64_t _last_plan_msg_timestamp;
     bool _last_plan_approved;  
   
     KDL::Frame _T_bodyframe_meshframe_left;
     KDL::Frame _T_bodyframe_meshframe_right;
     KDL::Frame _T_bodyframe_groundframe_left; 
     KDL::Frame _T_bodyframe_groundframe_right;
   
    //FootStepPlanListener(RendererStickyFeet *parent_renderer);
    FootStepPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer);
    ~FootStepPlanListener();
    
   // boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_planned_stickyfeet_list; 
    std::vector< int64_t >  _gl_planned_stickyfeet_timestamps; 
        
        
    
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody>   _gl_on_motion_copy;
    int on_motion_footstep_index; // make markers for moving footsteps persistent across multiple plans
    int64_t on_motion_footstep_utime;
    
    
    enum _foot_type
    {
       LEFT=0, RIGHT
    } foot_type; 
    
    std::vector< int >  _planned_stickyfeet_info_list; 
    

    void commit_footstep_plan(int64_t utime,string &channel);
    
        
    void create_sticky_foot_local_copy(int index){
      _gl_on_motion_copy.reset();
      _gl_on_motion_copy = shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(*_gl_planned_stickyfeet_list[index],_gl_planned_stickyfeet_list[index]->_unique_name));
      map<string,double> jointpos_in;
      jointpos_in = _gl_planned_stickyfeet_list[index]->_current_jointpos;    
      _gl_on_motion_copy->set_state(_gl_planned_stickyfeet_list[index]->_T_world_body,jointpos_in);
      _gl_on_motion_copy->set_bodypose_adjustment_type((int)InteractableGlKinematicBody::TWO_D);
      on_motion_footstep_index=index;
      on_motion_footstep_utime=_gl_planned_stickyfeet_timestamps[index];
    };
    
     //-------------message callback
    private:
    void handleFootStepPlanMsg(const lcm::ReceiveBuffer* rbuf,
		        const std::string& chan, 
		        const drc::ee_goal_sequence_t* msg);

    bool load_foot_urdfs();


   
    
  }; //class FootStepPlanListener

} //namespace renderer_sticky_feet


#endif //RENDERER_STICKYFEET_FOOTSTEPPLANLISTENER_HPP
