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
  typedef struct _StickyFeetInfoStruct
  {
    int foot_type; // left or right
    bool is_fixed;
    bool is_in_contact;
    double step_speed;
    double step_height;
  } StickyFeetInfoStruct;
  
  class FootStepPlanListener
  {
    //--------fields
  private:
    //RendererStickyFeet* _parent_renderer;
    BotViewer *_viewer;
    
    boost::shared_ptr<lcm::LCM> _lcm;    

    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_stickyfoot_left;
    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_stickyfoot_right;
    
    drc::footstep_plan_t revieved_plan_;
     

    std::string _left_urdf_xml_string;
    std::string _right_urdf_xml_string;



   //----------------constructor/destructor   
  public:
     std::string _robot_name;          
     std::string _left_foot_name; // foot ee names
     std::string _right_foot_name;
     // Eigen::Vector3f _left_foot_offset;
     // Eigen::Vector3f _right_foot_offset;
     int64_t _last_plan_msg_timestamp;
     bool _last_plan_approved;  
     bool _waiting_for_new_plan;
      
     // KDL::Frame _T_bodyframe_meshframe_left;
     // KDL::Frame _T_bodyframe_meshframe_right;
      KDL::Frame _T_bodyframe_groundframe_left; 
      KDL::Frame _T_bodyframe_groundframe_right;
   
    //FootStepPlanListener(RendererStickyFeet *parent_renderer);
    FootStepPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer);
    ~FootStepPlanListener();
    


    // the cache
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_planned_stickyfeet_list; 
    // std::vector< int64_t >  _gl_planned_stickyfeet_timestamps; 
    // std::vector< double >  _gl_planned_stickyfeet_speeds; 
    std::vector< int32_t >  _gl_planned_stickyfeet_ids;
    enum _foot_type
    {
       LEFT=0, RIGHT
    } foot_type; 
    
    std::vector< StickyFeetInfoStruct >  _planned_stickyfeet_info_list;
    boost::shared_ptr<visualization_utils::InteractableGlKinematicBody>   _gl_in_motion_copy;

       

    // methods
    void commit_footstep_plan(int64_t utime,string &channel);
    void create_sticky_foot_local_copy(int index){
      _gl_in_motion_copy.reset();
      _gl_in_motion_copy = boost::shared_ptr<InteractableGlKinematicBody>(new InteractableGlKinematicBody(*_gl_planned_stickyfeet_list[index],_gl_planned_stickyfeet_list[index]->_unique_name));
      map<string,double> jointpos_in;
      jointpos_in = _gl_planned_stickyfeet_list[index]->_current_jointpos;    
      _gl_in_motion_copy->set_state(_gl_planned_stickyfeet_list[index]->_T_world_body,jointpos_in);
      _gl_in_motion_copy->set_bodypose_adjustment_type((int)InteractableGlKinematicBody::THREE_D);
      in_motion_footstep_id=_gl_planned_stickyfeet_ids[index];
    };
    
    bool is_motion_copy(int index){    
      if(_gl_in_motion_copy)//exists
        return (_gl_planned_stickyfeet_ids[index]==in_motion_footstep_id);  
      else
        return false;
    };
    
    int get_motion_copy_index()
    {  
      std::vector<int32_t>::const_iterator found;
      found = std::find (_gl_planned_stickyfeet_ids.begin(), _gl_planned_stickyfeet_ids.end(), in_motion_footstep_id); 
      if(found != _gl_planned_stickyfeet_ids.end()) 
      {  
          int index = found - _gl_planned_stickyfeet_ids.begin();
          return index;
      }  
      return  -1.0; 
    };
    
     //-------------message callback
    private:
    
    void handleFootStepPlanMsg(const lcm::ReceiveBuffer* rbuf,
		        const std::string& chan, 
		        const drc::footstep_plan_t* msg);

    bool load_foot_urdfs();
    int in_motion_footstep_id; // make markers for moving footsteps persistent across multiple plans

   
    
  }; //class FootStepPlanListener

} //namespace renderer_sticky_feet


#endif //RENDERER_STICKYFEET_FOOTSTEPPLANLISTENER_HPP
