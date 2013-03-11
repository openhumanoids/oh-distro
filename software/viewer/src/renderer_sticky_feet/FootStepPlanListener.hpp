#ifndef RENDERER_STICKYFEET_FOOTSTEPPLANLISTENER_HPP
#define RENDERER_STICKYFEET_FOOTSTEPPLANLISTENER_HPP

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

namespace renderer_sticky_feet 
{

  class FootStepPlanListener
  {
    //--------fields
  private:

    boost::shared_ptr<lcm::LCM> _lcm;    
    
    //get rid of this
    BotViewer *_viewer;


    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_stickyfoot_left;
    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_stickyfoot_right;
    
    drc::ee_goal_sequence_t revieved_plan_;
     

    std::string _left_urdf_xml_string;
    std::string _right_urdf_xml_string;
    Eigen::Vector3f _left_foot_offset;
    Eigen::Vector3f _right_foot_offset;

   //----------------constructor/destructor   
  public:
     std::string _robot_name;          
     std::string _left_foot_name; // foot ee names
     std::string _right_foot_name;
     int64_t _last_plan_msg_timestamp; 
   
    FootStepPlanListener(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer);
    ~FootStepPlanListener();
    
   // boost::shared_ptr<collision::Collision_Detector> _collision_detector;
    std::vector< boost::shared_ptr<visualization_utils::InteractableGlKinematicBody> >  _gl_planned_stickyfeet_list; 
    
    enum _foot_type
    {
       LEFT=0, RIGHT
    } foot_type; 
    
    std::vector< int >  _planned_stickyfeet_info_list; 
    

     //-------------message callback
  private:
    void handleFootStepPlanMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::ee_goal_sequence_t* msg);

    bool load_foot_urdfs();
   
    
}; //class FootStepPlanListener

} //namespace renderer_sticky_feet


#endif //RENDERER_STICKYFEET_FOOTSTEPPLANLISTENER_HPP
