#ifndef RENDERER_AFFORDANCES_AFFORDANCECOLLECTIONLISTENER_HPP
#define RENDERER_AFFORDANCES_AFFORDANCECOLLECTIONLISTENER_HPP

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>
#include <lcmtypes/bot_core.hpp>
#include <bot_vis/bot_vis.h>
#include "renderer_affordances.hpp" // has definition of RendererAffordances struc
#include <visualization_utils/GlKinematicBody.hpp>
#include <visualization_utils/InteractableGlKinematicBody.hpp>

namespace renderer_affordances
{
  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class AffordanceCollectionListener
  {
    //--------fields
  private:
    std::string _robot_name;
    boost::shared_ptr<lcm::LCM> _lcm;    
    RendererAffordances* _parent_affordance_renderer; // maintains the list of objects.


    //----------------constructor/destructor
  public:
//     AffordanceCollectionListener(boost::shared_ptr<lcm::LCM> &lcm,
// 		       RendererAffordances* affordance_renderer);
    AffordanceCollectionListener(RendererAffordances* affordance_renderer);
    ~AffordanceCollectionListener();
    
  private:
   //-------------message callbacks
    
   void handleAffordanceCollectionMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::affordance_collection_t* msg);
   void handleAffordanceMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::affordance_t* msg);    
			    
	//-------------utils   
	//std::string get_filename(int32_t otdf_id); 
	void add_new_otdf_object_instance (std::string &filename, const drc::affordance_t &aff);
	void update_object_instance (const drc::affordance_t &aff);

}; //class AffordanceCollectionListener

} //namespace affordance_renderer


#endif //RENDERER_AFFORDANCES_AFFORDANCECOLLECTIONLISTENER_HPP
