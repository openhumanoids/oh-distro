#ifndef RENDERER_AFFORDANCES_INITGRASPOPTPUBLISHER_HPP
#define RENDERER_AFFORDANCES_INITGRASPOPTPUBLISHER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>
#include <kdl/frames.hpp>
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.h>

#include "renderer_affordances.hpp" // has definition of RendererAffordances struc

namespace renderer_affordances
{

  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class InitGraspOptPublisher
  {
    //--------fields
  private:
    std::string _urdf_xml_string; // of the hand/hands

    boost::shared_ptr<lcm::LCM> _lcm; 
    RendererAffordances* _parent_renderer; // maintains the list of objects.  

    //get rid of this
   // BotViewer *_viewer;
   
    //----------------constructor/destructor
  public:
    //InitGraspOptPublisher(boost::shared_ptr<lcm::LCM> &lcm, BotViewer *viewer);
    InitGraspOptPublisher(RendererAffordances* affordance_renderer);
    ~InitGraspOptPublisher();
    void publishGraspOptControlMsg(const std::string& channel, const KDL::Frame &T_geom_lhandpose,  const KDL::Frame &T_geom_rhandpose,const int grasp_type,const int contact_mask,const int drake_control, const int uid);	

			      
		//-------------utils
  private:	
		
		std::string _object_name;
		std::string _geometry_name;
		int _grasp_type;
		
  }; //class InitGraspOptPublisher


} //end namespace

#endif //RENDERER_AFFORDANCES_INITGRASPOPTPUBLISHER_HPP
