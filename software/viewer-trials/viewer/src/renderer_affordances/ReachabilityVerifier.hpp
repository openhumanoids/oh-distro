#ifndef RENDERER_AFFORDANCES_REACHABILITYVERIFIER_HPP
#define RENDERER_AFFORDANCES_REACHABILITYVERIFIER_HPP

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

#include <state/state_gfe.h>
#include <kinematics/kinematics_model_gfe.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include "renderer_affordances.hpp" // has definition of RendererAffordances struc

namespace renderer_affordances
{

  class ReachabilityVerifier 
  {
    //--------fields
  private:
    RendererAffordances* _parent_renderer; // maintains the list of objects.  
    kinematics::Kinematics_Model_GFE _kinematics_model_gfe;
   
    //----------------constructor/destructor
  public:
    ReachabilityVerifier(RendererAffordances* affordance_renderer);
    ~ReachabilityVerifier();
    //hand_type SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT=3, IROBOT_RIGHT=4, IROBOT_BOTH=5;
    bool has_IK_solution_from_pelvis_to_hand(drc::robot_state_t &statemsg, int hand_type, KDL::Frame &T_world_hand);
    //foot_type LEFT=0, RIGHT=1
    bool has_IK_solution_from_pelvis_to_foot(drc::robot_state_t &statemsg, int foot_type, KDL::Frame &T_world_foot);
  private:	      


  }; //class ReachabilityVerifier


} //end namespace


#endif //RENDERER_AFFORDANCES_CANDIDATEGRASPLISTENER_HPP
