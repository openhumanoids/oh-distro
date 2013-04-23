#ifndef RENDERER_AFFORDANCES_CANDIDATEGRASPLISTENER_HPP
#define RENDERER_AFFORDANCES_CANDIDATEGRASPLISTENER_HPP

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
#include "renderer_affordances.hpp" // has definition of RendererAffordances struc

namespace renderer_affordances
{

  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class CandidateGraspSeedListener
  {
    //--------fields
  private:
    std::string _urdf_xml_string; // of the hand/hands

    boost::shared_ptr<lcm::LCM> _lcm; 
    RendererAffordances* _parent_renderer; // maintains the list of objects.  

   
    //----------------constructor/destructor
  public:
    CandidateGraspSeedListener(RendererAffordances* affordance_renderer);
    ~CandidateGraspSeedListener();

    //boost::shared_ptr<collision::Collision_Detector> _collision_detector; 

    

    //-------------message callback
  private:
    void handleDesiredGraspStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::desired_grasp_state_t* msg);
			      
		//-------------utils
  private:	      
		void add_or_update_sticky_hand(int uid, std::string& unique_hand_name, KDL::Frame &T_world_hand, drc::joint_angles_t &posture_msg);		
		bool load_hand_urdf(int grasp_type);
		
		std::string _object_name;
		std::string _geometry_name;
		int _grasp_type;
		int _optimization_status;// RUNNING=0, SUCCESS=1, FAILURE=2;
		std::vector< boost::shared_ptr<visualization_utils::GlKinematicBody> >  _gl_hand_list;
		std::map<int,int> _handtype_id_map;
  }; //class CandidateGraspSeedListener


} //end namespace


#endif //RENDERER_AFFORDANCES_CANDIDATEGRASPLISTENER_HPP
