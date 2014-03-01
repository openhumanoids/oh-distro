#include <foot_contact/FootContact.h>

using namespace TwoLegs;
using namespace std;

FootContact::FootContact(bool _log_data_files,const float atlasWeight){
  cout << "A new FootContact object was created" << endl;

  // Set Parameters (previously all #defined)
  // TODO: expose these in the class API
  low_foot_contact_thresh_ = 0;
  high_foot_contact_thresh_ = 5;
  foot_contact_delay_ = 5000;

  loadsplit_level_ = 0.75;
  min_standing_force_ = 50.0;

  schmitt_level_ = 0.65;
  transition_timeout_ = 4000;
  ////////////////////////////////////////////

  standing_foot = -1;
  standingintermediate = true;

  lcmutime = 0;
  deltautime = 0;
  
  expectedweight = atlasWeight;
  standing_timer = 0;
  standing_delay = 0;
  
  foottransitionintermediateflag = true;
  
  leftforces.x = 0.f;
  leftforces.y = 0.f;
  leftforces.z = 0.f;
  
  rightforces.x = 0.f;
  rightforces.y = 0.f;
  rightforces.z = 0.f;
  
  transition_timespan = 0;
  
  datafile.Open(_log_data_files,"datalog.csv");
  footcontactfile.Open(_log_data_files,"footcontactlog.csv");
  
  _left_contact_state  = new SchmittTrigger(low_foot_contact_thresh_, high_foot_contact_thresh_, foot_contact_delay_, foot_contact_delay_);
  _right_contact_state = new SchmittTrigger(low_foot_contact_thresh_, high_foot_contact_thresh_, foot_contact_delay_, foot_contact_delay_);
  
}

void FootContact::terminate() {
  std::cout << "Terminating and cleaning out FootContact object\n";
  
  delete _left_contact_state;
  delete _right_contact_state;
  
  datafile.Close();
  footcontactfile.Close();
}


void FootContact::updateSingleFootContactStates(long utime, const double left_force, const double right_force) {
  _left_contact_state->UpdateState(utime, left_force);
  _right_contact_state->UpdateState(utime, right_force);
}


float FootContact::leftContactStatus() {
  return _left_contact_state->getState();
}

float FootContact::rightContactStatus() {
  return _right_contact_state->getState();
}

void FootContact::setStandingFoot(int foot) {
  standing_foot = foot;
}

int FootContact::getStandingFoot() {
  return standing_foot;
}

float FootContact::getPrimaryFootZforce() {
  if (standing_foot == LEFTFOOT)
    return leftforces.z;
  return rightforces.z;
}

float FootContact::getSecondaryFootZforce() {
  if (standing_foot == LEFTFOOT)
    return rightforces.z;
  return leftforces.z;
}


int FootContact::primary_foot() {
  return standing_foot;
}

int FootContact::secondary_foot() {
  if (standing_foot == LEFTFOOT)
    return RIGHTFOOT;
  if (standing_foot == RIGHTFOOT)
    return LEFTFOOT;
  std::cout << "FootContact::secondary_foot(): THIS SHOULD NOT HAPPEN THE FOOT NUMBERS ARE INCONSISTENT\n";
  return -99;
}

footstep FootContact::DetectFootTransistion(int64_t utime, float leftz, float rightz) {
  deltautime =  utime - lcmutime;
  lcmutime = utime;
  leftforces.z = leftz;
  rightforces.z = rightz;

  footstep newstep;
  newstep.foot = -1;

  if (getSecondaryFootZforce() - schmitt_level_*expectedweight > getPrimaryFootZforce()) {
    transition_timespan += deltautime;
  }else{
    transition_timespan = 0.;
    foottransitionintermediateflag = true;

    // in the intermediate zone
    // potentially standing
  }

  if (transition_timespan > transition_timeout_ && foottransitionintermediateflag)   {
    Eigen::Isometry3d transform;

    foottransitionintermediateflag = false;

    //mfallon removed: stepcount++;
    //mfallon removed: newstep.foot = secondary_foot();
    //mfallon removed: newstep.footprintlocation = AccumulateFootPosition(getPrimaryInLocal(), primary_foot());
    // if this fails, we need to ensure that botht eh libbot conversions are consistent
    //std::cout << "check4: should be zeros " << (C2e(newstep.footprintlocation.linear()) - q2e_new(local_frame_orientation) ).transpose() << std::endl;

    // TODO - investigate this large delay requirement and tie it to a proper requirements, rather than a fudge factor
    standing_delay = 5*standing_transition_timeout_;
    newstep.foot = secondary_foot();

  }else{
    newstep.foot = -1;

    double loadsplit;
    loadsplit = abs(leftz*leftz - rightz*rightz) < (loadsplit_level_*expectedweight*loadsplit_level_*expectedweight);

    // Second layer of logic testing to isolate the robot standing condition
    if (loadsplit && leftz > min_standing_force_ && rightz > min_standing_force_) {

      if (standing_delay > 0) {
        standing_delay -= deltautime;
        //std::cout << "reducing standing delay to: " << standing_delay << std::endl;
      } else {
        standing_delay = 0;
      }
      standing_timer += deltautime;
    }else{
      if (standing_timer>standing_transition_timeout_){
        standing_timer = standing_transition_timeout_;
        standingintermediate = true;
      }
      standing_timer -= deltautime;
      if (standing_timer<0) {
        standingintermediate = false;
        standing_timer = 0;
        //both_feet_in_contact = false;
      }else{
        ;
      }
    }

    if ((standing_timer > standing_transition_timeout_ && standing_delay<=0) || standingintermediate) {
      //std::cout << "Standing for: " << standing_timer <<  "\n";
      //both_feet_in_contact = true;
    }else{
      ;
    }
  }

  //#if defined( LOG_DATA_FILES )
  // log blocking is handled by the object itself
  stringstream ss (stringstream::in | stringstream::out);
  ss << leftforces.z << ", " << rightforces.z << ", ";
  ss << standing_timer << ", " << standing_delay << ", ";
  ss << ((standingintermediate) ? "1" : "0") << ", ";
  //ss << ((both_feet_in_contact) ? "1" : "0");
  ss << std::endl;
  string datastr = ss.str();
  datafile << datastr;

  stringstream cnct_est (stringstream::in | stringstream::out);
  cnct_est << leftforces.z << ", " << rightforces.z << ", ";
  cnct_est << ( leftContactStatus() > 0.5 ? "1" : "0") << ", ";
  cnct_est << (rightContactStatus() > 0.5 ? "1" : "0");
  cnct_est << "\n";
  footcontactfile << cnct_est.str();
  //#endif

  return newstep;
}
