/*
 * This class is the interface between the LCM communications system and the computations relating to estimation of the robots pelvis position,
 * based on leg motion and joint encoder angles of the vehicle. This software depends on the LCM events to operate and compute the new states. 
 * The computed states are then retransmitted on LCM for whoever wants to use the pelvis and head positions and orientations estimates.
 * Navigation of the pelvis and head is done relative to a reset position, from where estimates are relative to that reset position and orientation 
 * reference frame.
 * 
 * d fourie
 * 3/24/2013
*/


#ifndef LEGODOMETRY_LCM_HANDLER_HPP_
#define LEGODOMETRY_LCM_HANDLER_HPP_

#include <stdio.h>
#include <iostream>
#include <string>
#include <map>

//#include <lcm/lcm-cpp.hpp>
//#include <boost/shared_ptr.hpp>
//#include <model-client/model-client.hpp>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/assign/std/vector.hpp>

//#include <lcmtypes/bot_core.hpp>
#include <lcm/lcm-cpp.hpp>
//#include <boost/shared_ptr.hpp>
#include <model-client/model-client.hpp>
#include <lcmtypes/bot_core.hpp>

#include <lcm/lcm.h>
#include "lcmtypes/drc_lcmtypes.hpp"
#include "TwoLegOdometry.h"

// So that we can draw pretty pictures and figure out what the transforms are doing
#include "visualization/viewer.hpp"
#include "visualization/pointcloud.hpp"
#include <isam/isam.h>
//#include "bot-core.h"


#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

//#include "urdf/model.h"

#define DISPLAY_FOOTSTEP_POSES

// At present this places a large computational burden on the system -- new poses are added at full rate but is not cleared, not important to fix for me at this point, so take note
//#define DRAW_DEBUG_LEGTRANSFORM_POSES

//#define PUBLISH_AT_TRUE_POSITION

class LegOdometry_Handler {
private:
	TwoLegs::TwoLegOdometry *_leg_odo;
	boost::shared_ptr<lcm::LCM> lcm_;
    BotParam* _botparam;
    BotFrames* _botframes;
	
	
	int ratecounter;
	
	lcm_t * lcm_viewer; // using this one separately for displaying leg odometry results in the collections viewer
	bool _do_estimation;
	bool _draw_footsteps;
	bool _log_data_files;
	bool _lcm_add_ext;
	
	// LCM stuff
	const char* robot_pose_channel;
	bool _finish;
	
	boost::shared_ptr<ModelClient> model_;
	KDL::Tree tree;
	boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
	std::vector<drc::link_transform_t> _link_tfs;
	
	// Drawing stuff
	ObjectCollection* _obj;
	LinkCollection* _link;
	
	bool stillbusy;
	int poseplotcounter;
	int collectionindex;
	bool firstpass;
	std::string _channel_extension;
	
	NumericalDiff local_to_head_vel_diff;
	NumericalDiff local_to_head_acc_diff;
	NumericalDiff local_to_head_rate_diff;
	
	DataFileLogger state_estimate_error_log;
	
	// TODO -- This must be removed and the actual valued from the LCM message should be used directly
	enum { UNKNOWN, DIFF_SCHMITT_WITH_DELAY };
	
	// Connect to the correct LCM messages and start to propagate that data into the odometry object for state estimation
	void setupLCM();
	
	//void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
	void robot_state_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);	
	void torso_imu_handler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::imu_t* msg);
	
	void getTransforms(const drc::robot_state_t * msg, Eigen::Isometry3d &left, Eigen::Isometry3d &right);
	void setupSolver();
	
	//void drc_transform_inverse(const drc::transform_t &in, drc::transform_t &out);

	Viewer* _viewer;
	
	void addIsometryPose(int objnumber, const Eigen::Isometry3d &target);

	void addFootstepPose_draw();
	
	void drawSumPose();

	void drawLeftFootPose();
	void drawRightFootPose();	

	void PublishFootContactEst(int64_t utime);
	void PublishEstimatedStates(const drc::robot_state_t * msg);
	
	void ParseFootForces(const drc::robot_state_t* msg, double &left_force, double &right_force);
	// This function may move to a new class in the future. This is just the starting location for testing of the functionality
	void DetermineLegContactStates(long utime, float left_z, float right_z);
	
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	LegOdometry_Handler(boost::shared_ptr<lcm::LCM> &lcm_, bool _do_estimation, bool _draw_footsteps, bool _log_data_files, bool _add_lcm_ext);
	~LegOdometry_Handler();
	
	void finish() { _finish = true; }
	
	// Run the estimator, assuming the LCM connections have been set up correctly
	void run(bool testingmode);
	
	void terminate();
};


#endif /*LEGODOMETRY_LCM_HANDLER_H_*/
