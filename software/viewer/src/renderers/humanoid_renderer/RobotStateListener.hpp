#ifndef JOINT_ANGLES_LISTENER_H
#define JOINT_ANGLES_LISTENER_H

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include <bot_vis/bot_vis.h>
#include <bot_core/bot_core.h>
#include <path_util/path_util.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif


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
   
    boost::shared_ptr<lcm::LCM> _lcm;   

    std::vector<std::string > _link_names;
    std::vector<boost::shared_ptr<urdf::Geometry> > _link_shapes;
    std::vector<drc::link_transform_t> _link_tfs;
     

    //get rid of this
    BotViewer *_viewer;
    
    bool _urdf_parsed;
    bool _urdf_subscription_on;

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
    void getState(std::vector<boost::shared_ptr<urdf::Geometry> > &_link_shapes, std::vector<drc::link_transform_t> &_link_tfs, std::vector<std::string> & _link_names);

    //---------------debugging
    static void printTransforms(const std::vector<boost::shared_ptr<urdf::Geometry> > &_link_shapes,
				const std::vector<drc::link_transform_t> &_link_tfs);

    std::map<std::string, GLuint > _mesh_map;  
  private:
 std::string evalMeshFilePath(std::string file_path_expression);
 std::string exec(std::string cmd);
 std::string evalROSMeshFilePath(std::string file_path_expression);

}; //class RobotStateListener

} //namespace fk


#endif //JOINT_ANGLES_LISTENER_H
