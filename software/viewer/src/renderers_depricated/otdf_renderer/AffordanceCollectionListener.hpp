#ifndef OTDF_RENDERER_AFFORDANCE_COLLECTION_LISTENER_HPP
#define OTDF_RENDERER_AFFORDANCE_COLLECTION_LISTENER_HPP

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
#include "renderer_otdf.hpp" // has definition of RendererOtdf struc

namespace otdf_renderer
{
  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class AffordanceCollectionListener
  {
    //--------fields
  private:
    std::string _robot_name;
    boost::shared_ptr<lcm::LCM> _lcm;    
    RendererOtdf* _parent_otdf_renderer; // maintains the list of objects.


    //----------------constructor/destructor
  public:
//     AffordanceCollectionListener(boost::shared_ptr<lcm::LCM> &lcm,
// 		       RendererOtdf* otdf_renderer);
    AffordanceCollectionListener(RendererOtdf* otdf_renderer);
    ~AffordanceCollectionListener();
    
    
   
  private:
   //-------------message callbacks
    
    void handleAffordanceCollectionMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::affordance_collection_t* msg);
    void handleAffordanceMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::affordance_t* msg);    
			    
	//-------------utils   
	std::string get_filename(int32_t otdf_id); 
	void add_new_otdf_object_instance (std::string &filename, const drc::affordance_t &aff);
	void update_object_instance (const drc::affordance_t &aff);
	void run_fk_and_gen_link_shapes_and_tfs (OtdfInstanceStruc &instance_struc);

}; //class AffordanceCollectionListener

} //namespace otdf_renderer


#endif //OTDF_RENDERER_AFFORDANCE_COLLECTION_LISTENER_HPP
