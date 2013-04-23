#ifndef RENDERER_AFFORDANCES_CANDIDATEFOOTSTEPSEEDMANAGER_HPP
#define RENDERER_AFFORDANCES_CANDIDATEFOOTSTEPSEEDMANAGER_HPP

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

  class CandidateFootStepSeedManager
  {
    //--------fields
  private:
    std::string _urdf_xml_string; // of the hand/hands

    boost::shared_ptr<lcm::LCM> _lcm; 
    RendererAffordances* _parent_renderer; // maintains the list of objects.  

   
    //----------------constructor/destructor
  public:
    CandidateFootStepSeedManager(RendererAffordances* affordance_renderer);
    ~CandidateFootStepSeedManager();
    
    void add_or_update_sticky_foot(int uid,int foot_type, string& object_name, string& geometry_name,
                                   KDL::Frame &T_world_foot, vector<string> &joint_names,
                                   vector<double> &joint_positions);
                                   
    KDL::Frame _T_groundframe_bodyframe_left; 
    KDL::Frame _T_groundframe_bodyframe_right; 

  private:	      

    bool load_foot_urdfs();
    bool _foot_urdfs_found;
    std::string _left_foot_name; // foot ee names
    std::string _right_foot_name;
    std::string _left_urdf_xml_string;
    std::string _right_urdf_xml_string;
		boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_foot_left;
    boost::shared_ptr<visualization_utils::GlKinematicBody> _base_gl_foot_right;


  }; //class CandidateFootStepSeedManager


} //end namespace


#endif //RENDERER_AFFORDANCES_CANDIDATEGRASPLISTENER_HPP
