#ifndef constraint_app_HPP_
#define constraint_app_HPP_

#include <lcm/lcm.h>

#include <otdf_parser/otdf_parser.h>

#include <lcmtypes/drc_lcmtypes.h>
#include <lcmtypes/bot_core.h>
#include <path_util/path_util.h>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <forward_kinematics/treefksolverposfull_recursive.hpp>
#include <otdf_parser/otdf_parser.h>
#include <otdf_parser/otdf_urdf_converter.h>

class constraint_app{
  
  public:

  constraint_app(lcm_t* publish_lcm);
        
  private:
    lcm_t* publish_lcm_;

    static void affordance_collection_handler_aux(const lcm_recv_buf_t* rbuf,
						  const char* channel,
						  const drc_affordance_collection_t* msg,
						  void* user_data) {
      ((constraint_app *) user_data)->affordance_collection_handler(msg);
    }
    void affordance_collection_handler(const drc_affordance_collection_t *msg);

  static void affordance_track_collection_handler_aux(const lcm_recv_buf_t* rbuf,
						      const char* channel,
						      const drc_affordance_track_collection_t* msg,
						      void* user_data) {
    ((constraint_app *) user_data)->affordance_track_collection_handler(msg);
  }
  void affordance_track_collection_handler(const drc_affordance_track_collection_t *msg);

  void print_kdl_tree(const KDL::Tree& tree, 
		      const KDL::TreeElement& segment,
		      unsigned int depth);
};    

#endif
