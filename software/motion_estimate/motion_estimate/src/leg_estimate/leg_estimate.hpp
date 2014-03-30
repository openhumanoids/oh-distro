#ifndef LEG_ESTIMATE_HPP_
#define LEG_ESTIMATE_HPP_

#include <fstream>      // std::ofstream

#include <lcm/lcm-cpp.hpp>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/assign/std/vector.hpp>

#include <map>



#include "urdf/model.h"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include <model-client/model-client.hpp>

#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <pointcloud_tools/pointcloud_math.hpp>
#include <pointcloud_tools/pointcloud_lcm.hpp>
#include <pointcloud_tools/pointcloud_vis.hpp>

#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/drc/pose_transform_t.hpp"

#include <estimate_tools/Filter.hpp>
#include <foot_contact/FootContact.h>
#include <foot_contact_alt/FootContactAlt.h>
#include <leg_estimate/foot_contact_classify.hpp>
#include <leg_estimate/common_conversions.hpp>




///////////////////////////////////////////////////////////////
class leg_estimate{
  public:
    leg_estimate(boost::shared_ptr<lcm::LCM> &lcm_publish_,
                 BotParam* botparam_, boost::shared_ptr<ModelClient> &model_);
    
    ~leg_estimate(){
    }
    
    void setPoseBDI(Eigen::Isometry3d bdi_to_body_in ){ bdi_to_body_ = bdi_to_body_in; }
    void setPoseBody(Eigen::Isometry3d world_to_body_in ){ world_to_body_ = world_to_body_in; }
    
    void setFootSensing(FootSensing lfoot_sensing_in, FootSensing rfoot_sensing_in ){ 
      lfoot_sensing_ = lfoot_sensing_in;
      rfoot_sensing_ = rfoot_sensing_in;
    }
    
    // Update the running leg odometry solution
    // returns: odometry_status - a foot contact classification
    // which can be used to determine a suitable covariance
    // 0 -> 1 float
    // 0 is very accurate   
    // 1 very inaccuracy    - foot breaks
    // -1 unuseable/invalid - foot strikes
    float updateOdometry(std::vector<std::string> joint_name, std::vector<float> joint_position, int64_t utime);


    void getDeltaLegOdometry(Eigen::Isometry3d &delta_odom_to_body, int64_t &current_utime, int64_t &previous_utime){
      delta_odom_to_body = delta_odom_to_body_;
      current_utime = current_utime_;
      previous_utime = previous_utime_;
    }
    
    Eigen::Isometry3d getRunningEstimate(){ return odom_to_body_; }
    
    void setLegOdometryMode(std::string leg_odometry_mode_in ){ leg_odometry_mode_ = leg_odometry_mode_in; }
    void setInitializationMode(std::string initialization_mode_in ){ initialization_mode_ = initialization_mode_in; }

  private:
    /// Utilites
    boost::shared_ptr<lcm::LCM> lcm_subscribe_, lcm_publish_;
    BotParam* botparam_;
    boost::shared_ptr<ModelClient> model_;
    boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
    pointcloud_vis* pc_vis_;
    // joint position filters, optionally used
    LowPassFilter lpfilter_[28];  
    

    /// Parameters
    int verbose_;
    std::string leg_odometry_mode_;
    // How the position will be initialized
    std::string initialization_mode_;
    // Which link is assumed to be stationary - typically [lr]_foot or [lr]_talus
    std::string l_standing_link_;
    std::string r_standing_link_;
    // use a heavy low pass filter on the input joints
    bool filter_joint_positions_;
    // detect and filter kinematics when contact occurs
    bool filter_contact_events_;
    // Publish Debug Data e.g. kinematic velocities and foot contacts
    bool publish_diagnostics_;    
    
    /// Foot Contact Classifiers
    // most recent measurements for the feet forces (typically synchronised with joints measurements
    FootSensing lfoot_sensing_, rfoot_sensing_; // unfiltered... check
    TwoLegs::FootContact* foot_contact_logic_; // dehann, conservative
    TwoLegs::FootContactAlt* foot_contact_logic_alt_; // mfallon
    // Classify Contact Events (seperate from the above)
    foot_contact_classify* foot_contact_classify_;
    
    // original method from Dehann uses a very conservative Schmitt trigger
    contact_status_id footTransition();
    // a more aggressive trigger with different logic
    contact_status_id footTransitionAlt();
    // output from the FootContact class(es), not directly related to primary_foot_, but close
    int standing_foot_; 
    
    /// Integration Methods:
    bool initializePose(Eigen::Isometry3d body_to_foot);
    bool prepInitialization(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, contact_status_id contact_status);
    // Pure Leg Odometry, no IMU
    // return: true on initialization, else false
    bool leg_odometry_basic(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, contact_status_id contact_status);
    // At the moment a foot transition occurs: slave the pelvis pitch and roll and then fix foot using fk.
    // Dont move or rotate foot after that.
    // return: true on initialization, else false    
    bool leg_odometry_gravity_slaved_once(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, contact_status_id contact_status);
    // Foot position, as with above. For subsequent ticks, foot quaternion is updated using the pelvis quaternion
    // The pelvis position is then backed out using this new foot positon and fk.
    // return: true on initialization, else false    
    bool leg_odometry_gravity_slaved_always(Eigen::Isometry3d body_to_l_foot,Eigen::Isometry3d body_to_r_foot, contact_status_id contact_status);
    
    /// State Variables
    // Current time from current input msg 
    int64_t current_utime_;
    int64_t previous_utime_;
    
    // Current position of pevis in nominal odometry frame
    Eigen::Isometry3d odom_to_body_;
    Eigen::Isometry3d previous_odom_to_body_;
    // The incremental motion of the pelvis: transform between previous_odom_to_body_ and odom_to_body_
    Eigen::Isometry3d delta_odom_to_body_;
    bool leg_odo_init_; // has the leg odometry been initialized. (set to false when an anomoly is detected)
    footid_alt primary_foot_; // the foot assumed to be fixed for the leg odometry. TODO: unify the foot ids
    Eigen::Isometry3d odom_to_fixed_primary_foot_; // Position in the odom frame in which the fixed foot is kept
    Eigen::Isometry3d odom_to_secondary_foot_; // Ditto for moving foot (entirely defined by kinematics)

    // Pelvis Position Estimate produced by BDI. 
    // Used to calculate position delta by defining pelvis and foot orientation
    // TODO: use estimated state instead
    Eigen::Isometry3d bdi_to_body_;
    
    // Pelvis Position produced by mav-estimator
    // TODO: this should be the same as the RBIS state, currently using LCM to provide this
    Eigen::Isometry3d world_to_body_;
    Eigen::Isometry3d world_to_fixed_primary_foot_; // as for odom, except in world frame
    Eigen::Isometry3d world_to_secondary_foot_; // as for odom, except in world frame
    
    Eigen::Isometry3d previous_body_to_l_foot_; // previous FK positions. Only used in one of the integration methods
    Eigen::Isometry3d previous_body_to_r_foot_;
    
    
};    

#endif
