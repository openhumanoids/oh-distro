#ifndef RENDERER_EEGOAL_ROBOTSTATELISTENER_H
#define RENDERER_EEGOAL_ROBOTSTATELISTENER_H

#include <boost/function.hpp>
#include <map>

#include "urdf/model.h"
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/bot_core.hpp"
#include <bot_vis/bot_vis.h>

namespace renderer_ee_goal
{
  /**Class for keeping track of robot link state / joint angles.
   The constructor subscribes to MEAS_JOINT_ANGLES and registers a callback*/
  class RobotStateListener
  {
    //--------fields
  public:
    std::string _robot_name;
  private:
    
    std::string _urdf_xml_string; 
    std::map<std::string, boost::shared_ptr<urdf::Link> > _links_map;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> _fksolver;
    
    lcm::Subscription *_urdf_subscription; //valid as long as _urdf_parsed == false
    boost::shared_ptr<lcm::LCM> _lcm;   


    //get rid of this
    BotViewer *_viewer;

    bool _urdf_parsed;
    bool _urdf_subscription_on;

    std::map<std::string, KDL::Frame > cartpos_out;
    int64_t cartpos_out_utime;

    //----------------constructor/destructor
  public:
    RobotStateListener(boost::shared_ptr<lcm::LCM> &lcm,
		       BotViewer *viewer);
    ~RobotStateListener();
    
    KDL::Frame T_body_world; // current body origin in world frame
    bool getLinkTf(std::string link_name, KDL::Frame &T_body_link); 
    
    //-------------message callback
  private:
    void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);
    // TODO:
    void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg); 



    void TransformLCMToKDLFrame(const drc::transform_t &t, KDL::Frame &k)
    {
      k.p[0] = t.translation.x;
      k.p[1] = t.translation.y;
      k.p[2] = t.translation.z;
      KDL::Rotation M;
      M =  KDL::Rotation::Quaternion( t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);
      k.M = M;
    };

}; //class RobotStateListener

} //end namespace 


#endif //RENDERER_EEGOAL_ROBOTSTATELISTENER_H
