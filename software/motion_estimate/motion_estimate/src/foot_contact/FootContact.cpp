#include <foot_contact/FootContact.h>

using namespace TwoLegs;
using namespace std;

FootContact::FootContact(bool log_data_files_, float atlas_weight_, float schmitt_level_):
     atlas_weight_(atlas_weight_), schmitt_level_(schmitt_level_){
  cout << "A new FootContact object was created" << endl;

  // was 1400*0.65 for a long time, but this didn't work with toe-off
  //schmitt_level_ = 0.95;//0.65; 
  transition_timeout_ = 4000;
  ////////////////////////////////////////////
  
  standing_foot = FOOT_UNKNOWN;

  lcmutime = 0;
  deltautime = 0;
  
  
  foottransitionintermediateflag = true;
  
  l_foot_force_z = 0.f;
  r_foot_force_z = 0.f;
  
  transition_timespan = 0;
}

footid FootContact::DetectFootTransition(int64_t utime, float leftz, float rightz) {
  deltautime =  utime - lcmutime;
  lcmutime = utime;
  l_foot_force_z = leftz;
  r_foot_force_z = rightz;

  footid new_footstep = FOOT_UNKNOWN;

  if (getSecondaryFootZforce() - schmitt_level_*atlas_weight_ > getPrimaryFootZforce()) {
    transition_timespan += deltautime;
  }else{
    transition_timespan = 0.;
    foottransitionintermediateflag = true;
  }

  if (transition_timespan > transition_timeout_ && foottransitionintermediateflag)   {
    foottransitionintermediateflag = false;
    new_footstep = getSecondaryFoot();
  }else{
    new_footstep = FOOT_UNKNOWN;
  }

  return new_footstep;
}


void FootContact::setStandingFoot(footid foot) {
  standing_foot = foot;
}

footid FootContact::getStandingFoot() {
  return standing_foot;
}

footid FootContact::getSecondaryFoot() {
  if (standing_foot == FOOT_LEFT)
    return FOOT_RIGHT;
  if (standing_foot == FOOT_RIGHT)
    return FOOT_LEFT;
  std::cout << "FootContact::secondary_foot(): THIS SHOULD NOT HAPPEN THE FOOT NUMBERS ARE INCONSISTENT\n";
  return FOOT_UNKNOWN;
}

float FootContact::getPrimaryFootZforce() {
  if (standing_foot == FOOT_LEFT)
    return l_foot_force_z;
  return r_foot_force_z;
}

float FootContact::getSecondaryFootZforce() {
  if (standing_foot == FOOT_LEFT)
    return r_foot_force_z;
  return l_foot_force_z;
}

