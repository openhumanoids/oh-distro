#include <iostream>
#include <cmath>
#include <sstream>

#include <leg-odometry/TwoLegOdometry.h>
#include <leg-odometry/TwoLegsEstimate_types.h>
#include <leg-odometry/QuaternionLib.h>

using namespace TwoLegs;
using namespace std;

TwoLegOdometry::TwoLegOdometry(bool _log_data_files, bool dont_init_hack, const float atlasWeight){
  cout << "A new TwoLegOdometry object was created" << endl;
  foot_contact = new FootContact(_log_data_files, atlasWeight);

  // This is just here initially for initial testing and setup of the code structure
  // TODO
  // Assuming the robot is standing still.
  //Eigen::Isometry3d first;
  footsteps.reset();

  previous_isometry_time = 0;

  //imu_orientation_estimate.setIdentity();
  local_frame_orientation.setIdentity();

  local_velocities.setZero();
  accel.setSize(3);
  pelvis_vel_diff.setSize(3);
  d_pelvis_vel_diff.setSize(3);
  if (!dont_init_hack) {
    std::cout << "Attempting to read parameters from file.\n";
    d_pelvis_vel_diff.ParameterFileInit();
  }

  slidecorrection.setIdentity();
  stepcount = 0;

  //for (int i=0;i<3;i++) {_filter[i] = &lpfilter[i];}
  //for (int i=0;i<3;i++) {_pos_filter[i] = &pos_lpfilter[i];}

  // the idea at this point is that if the acceleration component of velocity is above the limits for 3 ms in a row the state will assume that it is infact the correct veloticy estimate
  _vel_spike_isolation[0] = new BipolarSchmittTrigger(3, 5, VEL_SPIKE_ISOLATION_DELAY); 
  _vel_spike_isolation[1] = new BipolarSchmittTrigger(3, 5, VEL_SPIKE_ISOLATION_DELAY);
  _vel_spike_isolation[2] = new BipolarSchmittTrigger(3, 5, VEL_SPIKE_ISOLATION_DELAY);

  accel_spike_isolation_log.Open(_log_data_files,"accel_spikes.csv");

  for (int i=0;i<3;i++) {
    temp_max_testing[i] = 0.;
  }

  accrued_sliding.setZero();
}

TwoLegOdometry::~TwoLegOdometry() {
  std::cout << "Terminating a TwoLegOdometry object and its allocated memory\n";
  terminate();
  delete _vel_spike_isolation[0];
  delete _vel_spike_isolation[1];
  delete _vel_spike_isolation[2];		
}

void TwoLegOdometry::parseRobotInput() {
  cout << "TwoLegOdometry::parseLCMInput() called, NOT implemented" << endl;
  return;
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

state TwoLegOdometry::getSecondaryFootState() {
  state secondaryfoot_state;
  std::cerr << "TwoLegOdometry::getSecondaryFootState -- IS NOT READY TO BE USED\n";
  //secondaryfoot_state = ??
  return secondaryfoot_state;
}

void TwoLegOdometry::updateInternalStates() {
  cout << "void TwoLegOdometry::updateInternalStates(); nothing updated - NOT implemented" << endl;
  return;
}

bool TwoLegOdometry::FootLogic(long utime, float leftz, float rightz) {
  footstep newstep;

  //std::cout << "TwoLegOdometry::FootLogic -- before detect" << std::endl;
  newstep = foot_contact->DetectFootTransistion(utime, leftz, rightz);
  
  if (newstep.foot != -1){
    stepcount++;
    newstep.footprintlocation = AccumulateFootPosition(getPrimaryInLocal(), foot_contact->primary_foot());
    
    std::cout << "NEW STEP ON " << ((foot_contact->secondary_foot()==LEFTFOOT) ? "LEFT" : "RIGHT") << " stepcount: " << stepcount << " at x= " << getSecondaryInLocal().translation().x() << std::endl;
    
  }
  
  //std::cout << "Foot at this point is: " << newstep.foot << std::endl;

  if (newstep.foot == LEFTFOOT || newstep.foot == RIGHTFOOT) {
    //std::cout << "PELVIS AT STEP: " << 57.29*(truth_E - C2e(newstep.footprintlocation.rotation())).transpose() << std::endl;

    //std::cout << "FootLogic adding Footstep " << (newstep.foot == LEFTFOOT ? "LEFT" : "RIGHT") << std::endl;
    foot_contact->setStandingFoot(newstep.foot);
    

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



void TwoLegOdometry::setLegTransforms(const Eigen::Isometry3d &left, const Eigen::Isometry3d &right) {
  pelvis_to_left = left;
  pelvis_to_right = right;
  //std::cout << "TwoLegOdometry::setLegTransforms -- " << left.translation().transpose() << std::endl << left.linear() << std::endl;
  left_to_pelvis = left.inverse();
  right_to_pelvis = right.inverse();

  //std::cout << "TwoLegOdometry::setLegTransforms -- after" << std::endl;
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
  returnval = AccumulateFootPosition(getPrimaryInLocal(), foot_contact->primary_foot());
  //std::cout << "TwoLegOdometry::getSecondaryInLocal -- is making a second call to AccumulateFootPosition\n";
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
  foot_contact->setStandingFoot( LEFTFOOT ); // Not sure that double states should be used, this should probably change TODO
}

int TwoLegOdometry::getStepCount() {
  return stepcount;
}

void TwoLegOdometry::terminate() {
  std::cout << "Terminating and cleaning out TwoLegOdometry object\n";
  foot_contact->terminate();
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

  unfiltered_vel = pelvis_vel_diff.diff((unsigned long long)current_time, current_position);
//  if(usedirectdiff){
//    unfiltered_vel = pelvis_vel_diff.diff((unsigned long long)current_time, current_position);
//  }else{
//    // use a distributed differential
//    unfiltered_vel = d_pelvis_vel_diff.diff((unsigned long long)current_time, current_position);
//  }
//
//  // with or without filtering
//  if(applyfiltering){
//    for(int i=0;i<3;i++){
//      local_velocities(i) = lpfilter[i].processSample(unfiltered_vel(i));
//    }
//  }else{
//    // no filtering on the joints
    overwritePelvisVelocity(unfiltered_vel);
//  }

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
