#ifndef JOINT_ANGLES_LISTENER_H
#define JOINT_ANGLES_LISTENER_H

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"

#include <bot_vis/bot_vis.h>

namespace fk
{
  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class RobotStateListener
  {
    //--------fields
  private:
    std::string _robot_name;
    std::string _urdf_xml_string; 
    std::vector<std::string> _joint_names_;
    std::map<std::string, boost::shared_ptr<urdf::Link> > _links_map;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;
    
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    bool _urdf_parsed;
    boost::shared_ptr<lcm::LCM> _lcm;    
    
    std::vector<boost::shared_ptr<urdf::Geometry> > _link_shapes;
    std::vector<drc::link_transform_t> _link_tfs;

    //get rid of this
    BotViewer *_viewer;

    //----------------constructor/destructor
  public:
    RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer);
    ~RobotStateListener();
    
    
    //-------------message callback
  private:
//  void handleJointAnglesMsg(const lcm::ReceiveBuffer* rbuf,
//			       const std::string& chan, 
//			      const drc::joint_angles_t* msg);    
    void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);
    void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg);

    //-----observers
  public:
    void getState(std::vector<boost::shared_ptr<urdf::Geometry> > &_link_shapes,
		  std::vector<drc::link_transform_t> &_link_tfs);

    //---------------debugging
    static void printTransforms(const std::vector<boost::shared_ptr<urdf::Geometry> > &_link_shapes,
				const std::vector<drc::link_transform_t> &_link_tfs);

}; //class JointAnglesHandler

} //namespace fk


#endif //JOINT_ANGLES_LISTENER_H
