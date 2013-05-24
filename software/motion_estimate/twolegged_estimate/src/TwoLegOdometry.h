#ifndef TWOLEGODOMETRY_H_
#define TWOLEGODOMETRY_H_


#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"

#include "lcmtypes/drc_lcmtypes.hpp"

#include "Footsteps.h"
#include "TwoLegsEstimate_types.h"
#include "SignalTap.hpp"
#include "Filter.hpp"
#include "HeavyLowPassFilter.hpp"
#include "LegOdo_Parameters.hpp"


namespace TwoLegs {

class TwoLegOdometry {
	private:
		Footsteps footsteps;
		int standing_foot;
		//bool both_feet_in_contact;
		bool foottransitionintermediateflag;
		bool standingintermediate;
		float expectedweight;
		footforces leftforces;
		footforces rightforces;
		int64_t lcmutime;
		int64_t deltautime;
		int64_t transition_timespan;
		int64_t standing_timer;
		int64_t standing_delay;
		int stepcount;
		
		Eigen::Quaterniond imu_orientation_estimate;
		Eigen::Quaterniond local_frame_orientation;
		Eigen::Vector3d local_frame_rates;
		Eigen::Vector3d local_velocities;
		Eigen::Vector3d local_accelerations;
		Eigen::Isometry3d previous_isometry;
		int64_t previous_isometry_time;
		
		DataFileLogger datafile;
		DataFileLogger footcontactfile;
		DataFileLogger accel_spike_isolation_log;
		
		SchmittTrigger* _left_contact_state;
		SchmittTrigger* _right_contact_state;
		
		// isolate and ignore velocity estimate spikes
		BipolarSchmittTrigger* _vel_spike_isolation[3];
		NumericalDiff accel;
		NumericalDiff pelvis_vel_diff;
		DistributedDiff d_pelvis_vel_diff;
		
		HeavyFiltering::HeavyLowPassFilter lpfilter[3];
		//Filter* _filter[3];
		
		LowPassFilter pos_lpfilter[3];
		//Filter* _pos_filter[3];
		
		double temp_max_testing[3];
		
		

		// TODO - these were made public for debugging, but should be brought back to private members once we have confidence in the various frame transformations
		/*
	    drc::transform_t pelvis_to_left;
	    drc::transform_t left_to_pelvis;
	    drc::transform_t pelvis_to_right;
	    drc::transform_t right_to_pelvis;
	    drc::transform_t local_to_pelvis;
	    */
	    
		// Convert the LCM message containing robot pose information to that which is requried by the leg odometry calculations
		void parseRobotInput();
		
		// used to predict where the secondary foot is in the world.
		state getSecondaryFootState();
		
		// take new data and update local states to what the robot is doing - as is used by the motion_model estimate
		void updateInternalStates();
		
		// Used internally to change the active foot for motion estimation
		void setStandingFoot(int foot);
		
		
		float getPrimaryFootZforce();
		float getSecondaryFootZforce();
		void ResetInitialConditions(const Eigen::Isometry3d &left_, const Eigen::Isometry3d &init_states);
		
		Eigen::Isometry3d getPelvisFromStep();
		Eigen::Isometry3d getSecondaryFootToPelvis();
		Eigen::Isometry3d getPrimaryFootToPelvis();
		
		// This has been moved to QuaternionLib
		//Eigen::Quaterniond mult(Eigen::Quaterniond lhs, Eigen::Quaterniond rhs);
		
		Eigen::Isometry3d AccumulateFootPosition(const Eigen::Isometry3d &from, const int foot_id);
		
		Eigen::Quaterniond MergePitchRollYaw(const Eigen::Quaterniond &lhs, const Eigen::Quaterniond &rhs);
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
		
		// TODO -- Move this to private members, with pointer return member functions()
		Eigen::Isometry3d pelvis_to_left;
	    Eigen::Isometry3d left_to_pelvis;
	    Eigen::Isometry3d pelvis_to_right;
	    Eigen::Isometry3d right_to_pelvis;
	    Eigen::Isometry3d local_to_pelvis;
		
		TwoLegOdometry(bool _log_data_files, bool dont_init_hack);
		~TwoLegOdometry();
		
		// Testing function not dependent on LCM messages
		// not implemented yet
		void CalculateBodyStates_Testing(int counter);
		
		// 
		bool UpdateStates(int64_t utime, const Eigen::Isometry3d &left, const Eigen::Isometry3d &right, const float &left_force, const float &right_force);
		
		// Return which foot is which - these are based on the RIGHTFOOT and LEFTFOOT defines in TwoLegsEstiamte_types.h
		int secondary_foot();
		int primary_foot();
		
		bool FootLogic(long utime, float leftz, float rightz);
		footstep DetectFootTransistion(int64_t utime, float leftz, float rightz);
		
		void setLegTransforms(const Eigen::Isometry3d &left, const Eigen::Isometry3d &right);
		void setOrientationTransform(const Eigen::Quaterniond &ahrs_orientation, const Eigen::Vector3d &body_rates);

		void setPelvisPosition(Eigen::Isometry3d transform);
		void ResetWithLeftFootStates(const Eigen::Isometry3d &left_, const Eigen::Isometry3d &right_, const Eigen::Isometry3d &init_states);
		
		Eigen::Isometry3d getSecondaryInLocal();
		Eigen::Isometry3d getPrimaryInLocal();
		
		Eigen::Isometry3d getLeftInLocal();
		Eigen::Isometry3d getRightInLocal();
		
		Eigen::Vector3d const getLocalFrameRates();
		
		Eigen::Isometry3d getPelvisState();
		Eigen::Vector3d getPelvisVelocityStates();
		int getStepCount();
		int getActiveFoot();
		float leftContactStatus();
		float rightContactStatus();
		
		void updateSingleFootContactStates(long utime, const double left_force, const double right_force);
		
		void terminate();
		
		static Eigen::Isometry3d add(const Eigen::Isometry3d& lhs, const Eigen::Isometry3d& rhs);
		
		void calculateUpdateVelocityStates(int64_t current_time, const Eigen::Isometry3d &current_pelvis);
		void overwritePelvisVelocity(const Eigen::Vector3d &set_velocity);


		// TODO -- remove this, only for testing
		Eigen::Vector3d truth_E;
		void setTruthE(const Eigen::Vector3d &tE);
};


}

#endif /*TWOLEGODOMETRY_H_*/
