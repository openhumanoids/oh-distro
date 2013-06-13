

#include <iostream>
#include <cmath>

#include "TwoLegOdometry.h"

#include "TwoLegsEstimate_types.h"

#include "QuaternionLib.h"
#include <sstream>


using namespace TwoLegs;
using namespace std;

TwoLegOdometry::TwoLegOdometry(bool _log_data_files, bool dont_init_hack)
{
	cout << "A new TwoLegOdometry object was created" << endl;
	
	// This is just here initially for initial testing and setup of the code structure
	// TODO
	// Assuming the robot is standing still.
	//Eigen::Isometry3d first;
	footsteps.reset();
	standing_foot = -1;

	//imu_orientation_estimate.setIdentity();
	local_frame_orientation.setIdentity();
	
	foottransitionintermediateflag = true;
	standingintermediate = true;
	
	// TODO - expected weight must be updated from live vehicle data and not be hard coded like this
	expectedweight = 900.f;
	
	local_velocities.setZero();
	accel.setSize(3);
	pelvis_vel_diff.setSize(3);
	d_pelvis_vel_diff.setSize(3);
	if (!dont_init_hack) {
		std::cout << "Attempting to read parameters from file.\n";
		d_pelvis_vel_diff.ParameterFileInit();
	}

	leftforces.x = 0.f;
	leftforces.y = 0.f;
	leftforces.z = 0.f;
	
	rightforces.x = 0.f;
	rightforces.y = 0.f;
	rightforces.z = 0.f;
	
	slidecorrection.setIdentity();

	lcmutime = 0;
	deltautime = 0;
	transition_timespan = 0;
	standing_timer = 0;
	standing_delay = 0;
	
	stepcount = 0;
	
	//for (int i=0;i<3;i++) {_filter[i] = &lpfilter[i];}
	//for (int i=0;i<3;i++) {_pos_filter[i] = &pos_lpfilter[i];}
	
	_left_contact_state = new SchmittTrigger(LOW_FOOT_CONTACT_THRESH, HIGH_FOOT_CONTACT_THRESH, FOOT_CONTACT_DELAY);
	_right_contact_state = new SchmittTrigger(LOW_FOOT_CONTACT_THRESH, HIGH_FOOT_CONTACT_THRESH, FOOT_CONTACT_DELAY);
	
	// the idea at this point is that if the acceleration component of velocity is above the limits for 3 ms in a row the state will assume that it is infact the correct veloticy estimate
	_vel_spike_isolation[0] = new BipolarSchmittTrigger(3, 5, VEL_SPIKE_ISOLATION_DELAY); 
	_vel_spike_isolation[1] = new BipolarSchmittTrigger(3, 5, VEL_SPIKE_ISOLATION_DELAY);
	_vel_spike_isolation[2] = new BipolarSchmittTrigger(3, 5, VEL_SPIKE_ISOLATION_DELAY);
	
	datafile.Open(_log_data_files,"datalog.csv");
	footcontactfile.Open(_log_data_files,"footcontactlog.csv");
	accel_spike_isolation_log.Open(_log_data_files,"accel_spikes.csv");
	
	for (int i=0;i<3;i++) {temp_max_testing[i] = 0.;}
	
	accrued_sliding.setZero();
}

TwoLegOdometry::~TwoLegOdometry() {
	std::cout << "Terminating a TwoLegOdometry object and its allocated memory\n";
	
	terminate();
	
	delete _left_contact_state;
	delete _right_contact_state;
	delete _vel_spike_isolation[0];
	delete _vel_spike_isolation[1];
	delete _vel_spike_isolation[2];
		
}

void TwoLegOdometry::parseRobotInput() {
	
  cout << "TwoLegOdometry::parseLCMInput() called, NOT implemented" << endl;
  
  return;
}

/*
 * Updates the internal states of the odometry object and under the correct circumstances transistions the standing foot as the robot is walking

void TwoLegOdometry::CalculateBodyStates_Testing(int counter) {
	cout << "_leg_odo::CalculateBodyStates() NOT implemented yet" << endl;
	
	// calculate the new pelvis and head states
	// create some data and pass it to this function
	
	/*data = ?
	
	//CalculateBodyStates(/*data);
	
	return;
}*/

int TwoLegOdometry::secondary_foot() {
	if (standing_foot == LEFTFOOT)
		return RIGHTFOOT;
	if (standing_foot == RIGHTFOOT)
		return LEFTFOOT;
	std::cout << "TwoLegOdometry::secondary_foot(): THIS SHOULD NOT HAPPEND THE FOOT NUMBERS ARE INCONSISTENT\n";
	return -99;
}

bool TwoLegOdometry::UpdateStates(int64_t utime, const Eigen::Isometry3d &left, const Eigen::Isometry3d &right, const float &left_force, const float &right_force) {
	
	bool foot_transition = false;
	Eigen::Isometry3d old_pelvis;
	old_pelvis = getPelvisFromStep();

	
	setLegTransforms(left, right);
	foot_transition = FootLogic(utime, left_force, right_force);
	
	// the local_to_pelvis transform will include the new footstep location
	Eigen::Isometry3d pelvis;
	pelvis = getPelvisFromStep();
	setPelvisPosition(pelvis);
	
	/*
	if (foot_transition) {
		// old - new should be zero
		std::cout << "check7: should be zero " << (old_pelvis.translation() - local_to_pelvis.translation()).transpose() << std::endl;
	}*/

	return foot_transition;
}



// now this seems excessive and should maybe be a more general transform update thing used for legs and head and pelvis
state TwoLegOdometry::getSecondaryFootState() {
	state secondaryfoot_state;
	
	std::cerr << "TwoLegOdometry::getSecondaryFootState -- IS NOT READY TO BE USED\n";

	//secondaryfoot_state = ??
	
	return secondaryfoot_state;
}

void TwoLegOdometry::setStandingFoot(int foot) {
	standing_foot = foot;
}

// TODO - unused function, to be depreciated
void TwoLegOdometry::updateInternalStates(/*data*/) {
	cout << "void TwoLegOdometry::updateInternalStates(); nothing updated - NOT implemented" << endl;
	
	// This function is currently not used
	
	return;
}

int TwoLegOdometry::primary_foot() {
	return standing_foot;
}

footstep TwoLegOdometry::DetectFootTransistion(int64_t utime, float leftz, float rightz) {
	
	deltautime =  utime - lcmutime;
	lcmutime = utime;
	leftforces.z = leftz;
	rightforces.z = rightz;
	
	footstep newstep;
	newstep.foot = -1;
	
	stringstream ss (stringstream::in | stringstream::out);
	stringstream cnct_est (stringstream::in | stringstream::out);

	
	if (getSecondaryFootZforce() - SCHMITT_LEVEL*expectedweight > getPrimaryFootZforce()) {
	  transition_timespan += deltautime;
	}
	else
	{
	  transition_timespan = 0.;
	  foottransitionintermediateflag = true;
	  
	  // in the intermediate zone
	  // potentially standing
	}
	  
	if (transition_timespan > TRANSITION_TIMEOUT && foottransitionintermediateflag) 
	{
		Eigen::Isometry3d transform;
		
		
		Eigen::Isometry3d temp;
		temp = getSecondaryInLocal();
#ifdef VERBOSE_DEBUG
		std::cout << "Adding footstep with values: " << temp.translation().transpose() << std::endl;
#endif

	  foottransitionintermediateflag = false;
	  
	  std::cout << "NEW STEP ON " << ((secondary_foot()==LEFTFOOT) ? "LEFT" : "RIGHT") << " stepcount: " << stepcount << " at x= " << getSecondaryInLocal().translation().x() << std::endl;

	  stepcount++;
	  newstep.foot = secondary_foot();
	  //newstep.footprintlocation = getSecondaryInLocal();
	  newstep.footprintlocation = AccumulateFootPosition(getPrimaryInLocal(), primary_foot());
	  // if this fails, we need to ensure that botht eh libbot conversions are consistent
	  //std::cout << "check4: should be zeros " << (C2e(newstep.footprintlocation.linear()) - q2e_new(local_frame_orientation) ).transpose() << std::endl;
	  
	  // TODO - investigate this large delay requirement and tie it to a proper requirements, rather than a fudge factor
  	  standing_delay = 5*STANDING_TRANSITION_TIMEOUT;
	  
	} else {

		
			double loadsplit;
			
			loadsplit = abs(leftz*leftz - rightz*rightz) < (LOADSPLIT_LEVEL*expectedweight*LOADSPLIT_LEVEL*expectedweight);
			
			// separation requirement no longer needed, kept for future reference
			//separation = abs(pelvis_to_left.translation().x()) + abs(pelvis_to_right.translation().x());
			//&& separation < MIN_STANDING_FEET_X_SEP
			
			// Second layer of logic testing to isolate the robot standing condition
			if (loadsplit && leftz > MIN_STANDING_FORCE && rightz > MIN_STANDING_FORCE) {
				
				if (standing_delay > 0) {
					standing_delay -= deltautime;
					//std::cout << "reducing standing delay to: " << standing_delay << std::endl;
				} else {
					standing_delay = 0;
				}
				
				standing_timer += deltautime;
			}
			else
			{
				if (standing_timer>STANDING_TRANSITION_TIMEOUT)
				{
					standing_timer = STANDING_TRANSITION_TIMEOUT;
					standingintermediate = true;
				}
				standing_timer -= deltautime;
				if (standing_timer<0) {
					standingintermediate = false;
					standing_timer = 0;
					
					//both_feet_in_contact = false;
				}
				else
				{
					;
				}
			}
			
			if ((standing_timer > STANDING_TRANSITION_TIMEOUT && standing_delay<=0) || standingintermediate) {
				//std::cout << "Standing for: " << standing_timer <<  "\n";
				//both_feet_in_contact = true;
			}
			else
			{
				;
			}
			
	}
	
//#if defined( LOG_DATA_FILES )
	// log blocking is handled by the object itself
	ss << leftforces.z << ", " << rightforces.z << ", ";
	
	ss << standing_timer << ", " << standing_delay << ", ";
	ss << ((standingintermediate) ? "1" : "0") << ", ";
	//ss << ((both_feet_in_contact) ? "1" : "0");
	
	ss << std::endl;
	string datastr = ss.str();
	datafile << datastr;
	
	cnct_est << leftforces.z << ", " << rightforces.z << ", ";
	
	cnct_est << ( leftContactStatus() > 0.5 ? "1" : "0") << ", ";
	cnct_est << (rightContactStatus() > 0.5 ? "1" : "0");
	cnct_est << "\n";
	
	footcontactfile << cnct_est.str();
//#endif
	
	
	return newstep;
}

bool TwoLegOdometry::FootLogic(long utime, float leftz, float rightz) {
  footstep newstep;

  newstep = DetectFootTransistion(utime, leftz, rightz);
  
  //std::cout << "Foot at this point is: " << newstep.foot << std::endl;

  if (newstep.foot == LEFTFOOT || newstep.foot == RIGHTFOOT) {
	  std::cout << "PELVIS AT STEP: " << 57.29*(truth_E - C2e(newstep.footprintlocation.rotation())).transpose() << std::endl;

	  std::cout << "FootLogic adding Footstep " << (newstep.foot == LEFTFOOT ? "LEFT" : "RIGHT") << std::endl;
	  standing_foot = newstep.foot;

	  Eigen::Vector3d alias;
	  alias = newstep.footprintlocation.translation() + accrued_sliding;
	  newstep.footprintlocation.translation() = alias;
	  //std::cout << "TwoLegOdometry::FootLogic -- is adding sliding accrued offset of: " << accrued_sliding.transpose() << std::endl;
	  accrued_sliding.setZero();

	  // Reset the offset if we are applying the correction to the footstep location
	  slidecorrection.setIdentity();

	  // This will show left transforms have non-zero rotations
	  //std::cout << "check3: should be zeros " << (q2e_new(local_frame_orientation) - C2e(newstep.footprintlocation.linear())).transpose() << std::endl;

	  //stripping out what should be the IMU based rotation for the new footstep, and is assumed to be added back when you query standing foot location -- compliments of torso_imu_handler
	  newstep.footprintlocation.linear() = Eigen::Matrix3d::Identity();
	  footsteps.newFootstep(newstep);
	  return true;
  }
  
  return false;
}

void TwoLegOdometry::AccruedPrimaryFootOffset(const Eigen::Vector3d &delta) {
	Eigen::Vector3d alias;

	alias = slidecorrection.translation() + delta;
	slidecorrection.translation() = alias;

	//std::cout << "slide translation offset by: " << delta.transpose() << " to: " << slidecorrection.translation() << std::endl;
}

float TwoLegOdometry::getPrimaryFootZforce() {
  if (standing_foot == LEFTFOOT)
    return leftforces.z;
  return rightforces.z;
}

float TwoLegOdometry::getSecondaryFootZforce() {
  if (standing_foot == LEFTFOOT)
	return rightforces.z;
  return leftforces.z;
}

void TwoLegOdometry::setLegTransforms(const Eigen::Isometry3d &left, const Eigen::Isometry3d &right) {
	pelvis_to_left = left;
	pelvis_to_right = right;
	left_to_pelvis = left.inverse();
	right_to_pelvis = right.inverse();
}

void TwoLegOdometry::setOrientationTransform(const Eigen::Quaterniond &ahrs_orientation, const Eigen::Vector3d &body_rates) {
	
	local_frame_orientation = ahrs_orientation;
	local_frame_rates = q2C(local_frame_orientation) * body_rates;

	if (false) {
		Eigen::Matrix3d C;
		C = q2C(ahrs_orientation);
		//imu_orientation_estimate = ahrs_orientation;
		Eigen::Quaterniond yaw_q;
		yaw_q = e2q(C2e(getPelvisFromStep().rotation()));
		// Merging is no longer required
		//local_frame_orientation = MergePitchRollYaw(imu_orientation_estimate,yaw_q);
	}

	
}

Eigen::Vector3d TwoLegOdometry::getLocalFrameRates() {
	return local_frame_rates;
}

Eigen::Isometry3d TwoLegOdometry::getSecondaryFootToPelvis() {
	//std::cout << "Taking primary as: " << (footsteps.lastFoot()==LEFTFOOT ? "LEFT" : "RIGHT") << std::endl;
	if (footsteps.lastFoot() == LEFTFOOT)
		return right_to_pelvis;
	if (footsteps.lastFoot() == RIGHTFOOT)
		return left_to_pelvis;

	// TODO -- This shou;ld be an exception
	std::cerr << "TwoLegOdometry::getSecondaryFootToPelvis() THIS SHOULD NEVER HAPPEN, FEET OUT OF SYNC\n";
	return Eigen::Isometry3d();
}

Eigen::Isometry3d TwoLegOdometry::getPrimaryFootToPelvis() {
	if (footsteps.lastFoot() == LEFTFOOT) {
		//std::cout << "check2: should be zeros " << C2e(left_to_pelvis.linear()).transpose() << std::endl;
		return left_to_pelvis;
	}
	if (footsteps.lastFoot() == RIGHTFOOT) {
		//std::cout << "check2: should be zeros " << C2e(right_to_pelvis.linear()).transpose() << std::endl;
		return right_to_pelvis;
	}

	std::cerr << "TwoLegOdometry::getPrimaryFootToPelvis() THIS SHOULD NEVER HAPPEN, FEET OUT OF SYNC -- foot here: " << footsteps.lastFoot() << "\n";
	return Eigen::Isometry3d();
}

Eigen::Quaterniond TwoLegOdometry::MergePitchRollYaw(const Eigen::Quaterniond &q_RollPitch, const Eigen::Quaterniond &q_Yaw) {
	Eigen::Vector3d E_rp;
	Eigen::Vector3d E_y;
	Eigen::Vector3d output_E;
		
	// TODO -- Remove the dependence on gimbal lock, by not using the Euler angle representation when merging the attitude angle estimates from the different computations

	E_rp = q2e_new(q_RollPitch);
	E_y  = q2e_new(q_Yaw);
	

	Eigen::Quaterniond return_q;
	
	// Only use the yaw angle from the leg kinematics
	E_y(0) = 0.;
	E_y(1) = 0.;

	E_rp(2) = 0.;
	
	
	if (false) {
		// Merge option
		output_E = (E_rp + E_y);
	} else {
		// use only the IMU angles
		output_E = q2e_new(q_RollPitch);
	}

	return_q = e2q(output_E);
	return return_q;
}




Eigen::Vector3d TwoLegOdometry::getPelvisVelocityStates() {
	
	return local_velocities;
}

Eigen::Quaterniond TwoLegOdometry::getLocalOrientation() {
	return local_frame_orientation;
}


// The intended user member call to get the pelvis state. The orientation from torso IMU
// Translation is from the accumulated leg kinematics
Eigen::Isometry3d TwoLegOdometry::getPelvisState() {
	return local_to_pelvis;
}
Eigen::Isometry3d TwoLegOdometry::getPelvisFromStep() {

	Eigen::Isometry3d returnval;
	returnval.setIdentity();

#ifdef MATTS_HELP
	

	Eigen::Isometry3d lhs;// this is just to test
	lhs = slidecorrection * footsteps.getLastStep(); // TODO

	returnval.translation() = add(lhs,getPrimaryFootToPelvis()).translation();
	returnval.linear() = q2C(local_frame_orientation);

#else

	returnval = getLastStep_w_IMUq() * getPrimaryFootToPelvis();

#endif


	return returnval;
}

Eigen::Isometry3d TwoLegOdometry::getLastStep_w_IMUq() {
	Eigen::Isometry3d standingfoot;
	standingfoot.setIdentity();

	// dehann and Matt, do not replace this getLastStep
	standingfoot = footsteps.getLastStep(); // TODO -- replace get last steps

	//std::cout << "check1: should be zeros " << C2e(standingfoot.linear()).transpose() << std::endl;

	standingfoot.linear() = q2C(local_frame_orientation);

	return standingfoot;
}


Eigen::Isometry3d TwoLegOdometry::AccumulateFootPosition(const Eigen::Isometry3d &from, const int foot_id) {
	Eigen::Isometry3d returnval;
	returnval.translation() << -999999999., -999999999.,-999999999.;
	
	switch (foot_id) {
	case LEFTFOOT:
		returnval = from * left_to_pelvis * pelvis_to_right;

		break;
	case RIGHTFOOT:
		//returnval = add(add(from,right_to_pelvis),pelvis_to_left);
		returnval = from * right_to_pelvis * pelvis_to_left;

		break;
	default:

		std::cerr << "THIS SHOULD NEVER HAPPEN - TwoLegOdometry::AccumulateFootPosition()" << std::endl;
		break;
	}
	
	return returnval;
}

Eigen::Isometry3d TwoLegOdometry::getSecondaryInLocal() {
	
	Eigen::Isometry3d returnval;
	returnval = AccumulateFootPosition(getPrimaryInLocal(),primary_foot());

	return returnval;
}

Eigen::Isometry3d TwoLegOdometry::getPrimaryInLocal() {

#ifdef MATTS_HELP
	return footsteps.getLastStep();
#else

	// This should have translation with IMU orientation overwritten
	return getLastStep_w_IMUq();
#endif
}

Eigen::Isometry3d TwoLegOdometry::getLeftInLocal() {

	return add(getPelvisFromStep(), pelvis_to_left);
}


Eigen::Isometry3d TwoLegOdometry::getRightInLocal() {

	return add(getPelvisFromStep(), pelvis_to_right);
}

void TwoLegOdometry::setPelvisPosition(Eigen::Isometry3d transform) {

  local_to_pelvis = transform;
}


Eigen::Isometry3d TwoLegOdometry::add(const Eigen::Isometry3d& lhs, const Eigen::Isometry3d& rhs) {

  Eigen::Isometry3d add;
  add = lhs*rhs;

  return add;
}

void TwoLegOdometry::ResetInitialConditions(const Eigen::Isometry3d &left_, const Eigen::Isometry3d &init_states) {
	
	// The left foot is used to initialize height of the pelvis.
	
	stepcount = 0;
	local_to_pelvis = init_states;
	footsteps.reset();
}

void TwoLegOdometry::ResetWithLeftFootStates(const Eigen::Isometry3d &left_, const Eigen::Isometry3d &right_, const Eigen::Isometry3d &init_states) {
	
	ResetInitialConditions(left_, init_states);
	footsteps.addFootstep(add(local_to_pelvis,left_),LEFTFOOT);
	standing_foot = LEFTFOOT; // Not sure that double states should be used, this should probably change TODO
}

int TwoLegOdometry::getStepCount() {
	return stepcount;
}

int TwoLegOdometry::getActiveFoot() {
	return standing_foot;
}

void TwoLegOdometry::updateSingleFootContactStates(long utime, const double left_force, const double right_force) {
	_left_contact_state->UpdateState(utime, left_force);
	_right_contact_state->UpdateState(utime, right_force);
}


float TwoLegOdometry::leftContactStatus() {
	return _left_contact_state->getState();
}

float TwoLegOdometry::rightContactStatus() {
	return _right_contact_state->getState();
}

void TwoLegOdometry::terminate() {
	std::cout << "Terminating and cleaning out TwoLegOdometry object\n";
	
	datafile.Close();
	footcontactfile.Close();
	accel_spike_isolation_log.Close();
}


void TwoLegOdometry::calculateUpdateVelocityStates(int64_t current_time, const Eigen::Isometry3d &current_pelvis) {
	calculateUpdateVelocityStates(current_time, current_pelvis, false,false);
}

void TwoLegOdometry::calculateUpdateVelocityStates(int64_t current_time, const Eigen::Isometry3d &current_pelvis, const bool &usedirectdiff, const bool &applyfiltering) {
	//std::cout << "Not implemented yet\n";
	stringstream accel_data_ss (stringstream::in | stringstream::out);
		
	Eigen::Vector3d current_position;
	//Eigen::Vector3d velocity_estimate;
	
	current_position = current_pelvis.translation();
	
	Eigen::Vector3d prev_velocities;
	prev_velocities = local_velocities;
	Eigen::Vector3d unfiltered_vel;

	if (usedirectdiff) {
		unfiltered_vel = pelvis_vel_diff.diff((unsigned long long)current_time, current_position);
	} else {
		// use a distributed differential
		unfiltered_vel = d_pelvis_vel_diff.diff((unsigned long long)current_time, current_position);
	}
	
	// with or without filtering
	if (applyfiltering) {
		for (int i=0;i<3;i++) {
			local_velocities(i) = lpfilter[i].processSample(unfiltered_vel(i));
		}
	} else {
		// no filtering on the joints
		overwritePelvisVelocity(unfiltered_vel);
	}
	
	local_accelerations = accel.diff(current_time, local_velocities);



	/*
	 * This is older spike isolation code -- but turns out this is lossy. The correct way to do this is loosely slave IMU double integral to LegOdo position
		if (false) {
			// this was used to isolate velocity spikes, while there was a bug in the foot to pelvis transforms --

			for (int i=0;i<3;i++) {
				_vel_spike_isolation[i]->UpdateState(current_time, local_accelerations(i));

				accel_data_ss << !_vel_spike_isolation[i]->getState()  << ", ";

				if (local_accelerations(i) < -3.5 || local_accelerations(i) > 3.5)
				{
					if (!_vel_spike_isolation[i]->getState()) {
						// accel values have not remained high, and can therefore be ignored
						local_velocities(i) = prev_velocities(i);
					}

				}
			}
		}
		*/

}

void TwoLegOdometry::overwritePelvisVelocity(const Eigen::Vector3d &set_velocity) {
	local_velocities = set_velocity;
}

void TwoLegOdometry::setTruthE(const Eigen::Vector3d &tE) {
	truth_E = tE;
}

void TwoLegOdometry::AccruedPelvisPosition(const Eigen::Vector3d &delta) {

	//std::cout << "TwoLegOdometry::AcruedPelvisPosition -- adjusting the pelvis translation state by: " << delta.transpose() << std::endl;

	Eigen::Vector3d alias,alias1;

	alias = local_to_pelvis.translation() + delta;
	local_to_pelvis.translation() = alias;
}

void TwoLegOdometry::setAccruedOffset(const Eigen::Vector3d &offset) {
	accrued_sliding = offset;
}
