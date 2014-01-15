#ifndef TWOLEGODOMETRY_H_
#define TWOLEGODOMETRY_H_

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"
#include "lcmtypes/drc_lcmtypes.hpp"

#include "Footsteps.h"
#include "TwoLegsEstimate_types.h"
#include <leg-odometry/SignalTap.hpp>
#include <leg-odometry/Filter.hpp>
#include <leg-odometry/HeavyLowPassFilter.hpp>
#include <leg-odometry/FootContact.h>

namespace TwoLegs {

class TwoLegOdometry {
  private:
    Footsteps footsteps;
    int stepcount;
    
    //Eigen::Quaterniond imu_orientation_estimate;
    Eigen::Quaterniond local_frame_orientation;
    Eigen::Vector3d local_frame_rates;
    Eigen::Vector3d local_velocities;
    Eigen::Vector3d local_accelerations;
    Eigen::Isometry3d previous_isometry;
    int64_t previous_isometry_time;
    
    DataFileLogger accel_spike_isolation_log;
    
    // isolate and ignore velocity estimate spikes
    BipolarSchmittTrigger* _vel_spike_isolation[3];
    NumericalDiff accel;
    NumericalDiff pelvis_vel_diff;
    DistributedDiff d_pelvis_vel_diff;
    
    HeavyFiltering::HeavyLowPassFilter lpfilter[3];
    LowPassFilter pos_lpfilter[3];
    
    double temp_max_testing[3];
    
    Eigen::Vector3d accrued_sliding;
    Eigen::Isometry3d slidecorrection;
    
    // Convert the LCM message containing robot pose information to that which is requried by the leg odometry calculations
    void parseRobotInput();
    
    // used to predict where the secondary foot is in the world.
    // now this seems excessive and should maybe be a more general transform update thing used for legs and head and pelvis
    state getSecondaryFootState();
    
    // take new data and update local states to what the robot is doing - as is used by the motion_model estimate
    // TODO - unused function, to be depreciated
    // This function is currently not used
    void updateInternalStates();
    
    void ResetInitialConditions(const Eigen::Isometry3d &left_, const Eigen::Isometry3d &init_states);
    
    Eigen::Isometry3d getPelvisFromStep();
    Eigen::Isometry3d getSecondaryFootToPelvis();

    // This has been moved to QuaternionLib
    //Eigen::Quaterniond mult(Eigen::Quaterniond lhs, Eigen::Quaterniond rhs);
    
    Eigen::Isometry3d AccumulateFootPosition(const Eigen::Isometry3d &from, const int foot_id);
    
    // To be depreciated
    Eigen::Quaterniond MergePitchRollYaw(const Eigen::Quaterniond &lhs, const Eigen::Quaterniond &rhs);

    Eigen::Isometry3d getLastStep_w_IMUq();

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TwoLegOdometry(bool _log_data_files, bool dont_init_hack, const float atlasWeight);
    ~TwoLegOdometry();
    
    // TODO -- Move this to private members, with pointer return member functions()
    Eigen::Isometry3d pelvis_to_left;
    Eigen::Isometry3d left_to_pelvis;
    Eigen::Isometry3d pelvis_to_right;
    Eigen::Isometry3d right_to_pelvis;
    Eigen::Isometry3d local_to_pelvis;
    
    
    FootContact* foot_contact;
    
    Eigen::Isometry3d getPrimaryFootToPelvis();

    //void CalculateBodyStates_Testing(int counter);
    
    // 
    bool UpdateStates(int64_t utime, const Eigen::Isometry3d &left, const Eigen::Isometry3d &right, const float &left_force, const float &right_force);
    
    
    bool FootLogic(long utime, float leftz, float rightz);
    
    void setLegTransforms(const Eigen::Isometry3d &left, const Eigen::Isometry3d &right);
    void setOrientationTransform(const Eigen::Quaterniond &ahrs_orientation, const Eigen::Vector3d &body_rates);

    void setPelvisPosition(Eigen::Isometry3d transform);
    void ResetWithLeftFootStates(const Eigen::Isometry3d &left_, const Eigen::Isometry3d &right_, const Eigen::Isometry3d &init_states);
    
    Eigen::Isometry3d getPrimaryInLocal();
    Eigen::Isometry3d getSecondaryInLocal();
    
    Eigen::Isometry3d getLeftInLocal();
    Eigen::Isometry3d getRightInLocal();
    
    Eigen::Vector3d getLocalFrameRates();
    Eigen::Quaterniond getLocalOrientation();
    
    Eigen::Isometry3d getPelvisState();
    Eigen::Vector3d getPelvisVelocityStates();
    int getStepCount();
    
    void terminate();
    
    Eigen::Isometry3d add(const Eigen::Isometry3d& lhs, const Eigen::Isometry3d& rhs);
    
    void calculateUpdateVelocityStates(int64_t current_time, const Eigen::Isometry3d &current_pelvis);
    void calculateUpdateVelocityStates(int64_t current_time, const Eigen::Isometry3d &current_pelvis, const bool &usedirectdiff, const bool &applyfiltering);
    void overwritePelvisVelocity(const Eigen::Vector3d &set_velocity);

    void AccruedPelvisPosition(const Eigen::Vector3d &delta);
    void setAccruedOffset(const Eigen::Vector3d &offset);
    void AccruedPrimaryFootOffset(const Eigen::Vector3d &delta);

    // TODO -- remove this, only for testing
    Eigen::Vector3d truth_E;
    void setTruthE(const Eigen::Vector3d &tE);
};

}

#endif /*TWOLEGODOMETRY_H_*/