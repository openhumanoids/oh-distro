#include <foot_contact_alt/FootContactAlt.h>

using namespace TwoLegs;
using namespace std;

FootContactAlt::FootContactAlt(bool _log_data_files,const float atlasWeight){
  cout << "A new FootContactAlt object was created" << endl;

  // was 1400*0.65 for a long time, but this didn't work with toe lift off stepping
  schmitt_level_ = 0.65; 
  transition_timeout_ = 4000;
  ////////////////////////////////////////////
  
  standing_foot = F_UNKNOWN;

  lcmutime = 0;
  deltautime = 0;
  
  expectedweight = atlasWeight;
  
  foottransitionintermediateflag = true;
  
  l_foot_force_z = 0.f;
  r_foot_force_z = 0.f;
  
  transition_timespan = 0;
  
  // actual noticable toe push off occurs with about 50:50 ratio: 760:760 or even later
  // To reduce tic-tocing you can reduce the first number
  // originally 275/375 was ok, but some upward drift (3/4 of a block height) and backward drift (1/3 of a block length)
  // later 475/525 was better, upward drift (1/2 of block) and backward drift (about the same)
  left_contact_state_strong_  = new SchmittTrigger(525.0, 575.0, 7000, 7000);
  right_contact_state_strong_ = new SchmittTrigger(525.0, 575.0, 7000, 7000); 
  
  left_contact_state_strong_->forceHigh();
  right_contact_state_strong_->forceHigh();
  
  verbose_ =1; // 3 lots, 2 some, 1 v.important
}


contact_status_id FootContactAlt::DetectFootTransition(int64_t utime, float leftz, float rightz) {
  bool lf_state_last = (bool) left_contact_state_strong_->getState();
  bool rf_state_last = (bool) right_contact_state_strong_->getState();
  
  left_contact_state_strong_->UpdateState(utime, leftz);
  right_contact_state_strong_->UpdateState(utime, rightz);  
  
  bool lf_state = (bool) left_contact_state_strong_->getState();
  bool rf_state = (bool) right_contact_state_strong_->getState();
  
  std::stringstream ss;
  ss << (int)standing_foot << " | " << lf_state << " " << rf_state << " ";
  
  if (!lf_state_last && lf_state){
    if (verbose_ >= 1) std::cout << ss.str() << "Left has gone high\n"; 
    standing_foot = F_LEFT;
    return F_LEFT_NEW;
  }else if(!rf_state_last && rf_state){
    if (verbose_ >= 1) std::cout << ss.str() << "Right has gone high\n"; 
    standing_foot = F_RIGHT;
    return F_RIGHT_NEW;
  }else if(lf_state_last && !lf_state){
    if (verbose_ >= 3) std::cout << ss.str() << "Left has gone low\n"; 
    if (standing_foot==F_LEFT){
      if (verbose_ >= 1) std::cout << ss.str() << "Left has gone low when used as standing, force switch to right "<< leftz << " | "<<  rightz <<"\n"; 
      standing_foot = F_RIGHT;
      return F_RIGHT_NEW;
    }else{
      if (verbose_ >= 3) std::cout << ss.str() << "Left has gone low when used not used as standing. Continue with right\n"; 
      return F_RIGHT_FIXED;
    }
  }else if(rf_state_last && !rf_state){
    if (verbose_ >= 3) std::cout << ss.str() << "Right has gone low\n"; 
    if (standing_foot==F_RIGHT){
      if (verbose_ >= 1) std::cout << ss.str() << "Right has gone low when used as standing, force switch to left "<< leftz << " | "<<  rightz <<"\n"; 
      standing_foot = F_LEFT;
      return F_LEFT_NEW;
    }else{
      if (verbose_ >= 3) std::cout << ss.str() << "Right has gone low when used not used as standing. Continue with left\n"; 
      return F_LEFT_FIXED;
    }    
  }else{
    if (standing_foot==F_LEFT){
      if (verbose_ >= 3) std::cout << ss.str() << "Left No change\n";
      return F_LEFT_FIXED;
    }else if (standing_foot==F_RIGHT){
      if (verbose_ >= 3) std::cout << ss.str() << "Right No change\n";
      return F_RIGHT_FIXED;
    }
  }
  
  std::cout << ss.str() << "Situation unknown. Error\n";
  exit(-1);
  return F_STATUS_UNKNOWN;
}

void FootContactAlt::setStandingFoot(footid_alt foot) {
  standing_foot = foot;
}

footid_alt FootContactAlt::getStandingFoot() {
  return standing_foot;
}

footid_alt FootContactAlt::getSecondaryFoot() {
  if (standing_foot == F_LEFT)
    return F_RIGHT;
  if (standing_foot == F_RIGHT)
    return F_LEFT;
  std::cout << "FootContactAlt::secondary_foot(): THIS SHOULD NOT HAPPEN THE FOOT NUMBERS ARE INCONSISTENT\n";
  return F_UNKNOWN;
}

float FootContactAlt::getPrimaryFootZforce() {
  if (standing_foot == F_LEFT)
    return l_foot_force_z;
  return r_foot_force_z;
}

float FootContactAlt::getSecondaryFootZforce() {
  if (standing_foot == F_LEFT)
    return r_foot_force_z;
  return l_foot_force_z;
}

