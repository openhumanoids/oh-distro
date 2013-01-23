#include <map>
#include <boost/shared_ptr.hpp>

#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"

#include <lcm/lcm.h>
#include "lcmtypes/drc_lcmtypes.h"



class ModelClient 
{
  public:
    ModelClient (lcm_t* lcm_, int keep_updated);
    ModelClient(lcm_t* lcm_, std::string model_channel_, int keep_updated_);
    ~ModelClient() {}
    
    void doModelClient();

    static void drc_robot_urdf_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel, 
                                            const drc_robot_urdf_t* msg, void* user_data){
      ((ModelClient *) user_data)->drc_robot_urdf_handler(channel, msg);
    }
    void drc_robot_urdf_handler(const char* channel, const drc_robot_urdf_t *msg);        
    
    std::string getURDFString(){ return urdf_xml_string_; }
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> getSolver(){ return fksolver_; }

  private:
    lcm_t* lcm_;
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    bool urdf_parsed_;
    std::string model_channel_;
    std::string robot_name_;
    std::string urdf_xml_string_;
    std::vector<std::string> joint_names_;    
    std::map<std::string, boost::shared_ptr<urdf::Link> > links_map_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;

    int keep_updated_;

};
