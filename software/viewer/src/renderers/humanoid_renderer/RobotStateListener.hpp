#ifndef JOINT_ANGLES_LISTENER_H
#define JOINT_ANGLES_LISTENER_H

#include <boost/function.hpp>

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
    lcm::LCM _lcm;    
    
    //----------------constructor/destructor
  public:
    RobotStateListener(lcm::LCM &lcm);
    ~RobotStateListener();
    
    
    //-------------message callback
  private:
    void  handleJointAnglesMsg(const lcm::ReceiveBuffer* rbuf,
			       const std::string& chan, 
			       const drc::joint_angles_t* msg);
    void handleRobotUrdfMsg(const lcm::ReceiveBuffer* rbuf, const std::string& channel, 
			    const  drc::robot_urdf_t* msg);
}; //class JointAnglesHandler

} //namespace fk


#endif //JOINT_ANGLES_LISTENER_H
