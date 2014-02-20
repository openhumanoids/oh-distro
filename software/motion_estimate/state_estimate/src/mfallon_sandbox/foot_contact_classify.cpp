#include "foot_contact_classify.hpp"

using namespace std;

foot_contact_classify::foot_contact_classify ( boost::shared_ptr<lcm::LCM> &lcm_publish_ , bool publish_diagnostics_ ):
    lcm_publish_(lcm_publish_), publish_diagnostics_(publish_diagnostics_){
  initialized_ = false;
  mode_ = UNKNOWN;
  verbose_ = 1; // 3 lots, 2 some, 1 v.important
  
  // Vis Config:
  pc_vis_ = new pointcloud_vis( lcm_publish_->getUnderlyingLCM());
  // obj: id name type reset
  pc_vis_->obj_cfg_list.push_back( obj_cfg(2001,"Null Pose [FCC]",5,1) );
  //pc_vis_->obj_cfg_list.push_back( obj_cfg(1002,"Primary Foot",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2004,"Sec FC in sole frame",1,0, 2001,1, { 0.5, 0.5, 0.0} ));  
  
  
  // defaults dehann used: 0, 5, 5000 but foot doesnt 'hang' like in VRC
  left_contact_state_weak_  = new SchmittTrigger(20.0, 30.0, 5000, 5000);
  right_contact_state_weak_ = new SchmittTrigger(20.0, 30.0, 5000, 5000); 
  left_contact_state_strong_  = new SchmittTrigger(275.0, 375.0, 7000, 7000);
  right_contact_state_strong_ = new SchmittTrigger(275.0, 375.0, 7000, 7000); 
  
  last_strike_utime_ = 0;
  last_break_utime_ = 0;
  strike_blackout_duration_ = 95000;// 75ms //was 10.000 
  break_blackout_duration_ = 800000; // 750ms
  
  ///////////////// Contact Points:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr contact_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
  contact_points_ = contact_points;
  contact_points_->width = 4;
  contact_points_->height = 1;
  contact_points_->points.resize (contact_points_->width);
  contact_points_->points[0].getVector4fMap() = Eigen::Vector4f(-0.082,  0.0624435, -0.081119, 0.); // heel
  contact_points_->points[1].getVector4fMap() = Eigen::Vector4f(-0.082, -0.0624435, -0.081119, 0.); // heel 
  contact_points_->points[2].getVector4fMap() = Eigen::Vector4f( 0.178,  0.0624435, -0.081119, 0.); // toe
  contact_points_->points[3].getVector4fMap() = Eigen::Vector4f( 0.178, -0.0624435, -0.081119, 0.); // toe
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cp_moving_prev(new pcl::PointCloud<pcl::PointXYZRGB> ());
  cp_moving_prev_ = cp_moving_prev;
  foot_to_sole_ = Eigen::Isometry3d( Eigen::Isometry3d::Identity() );
  foot_to_sole_.translation().z() = 0.081119;  
}

std::string print_variables(int mode_, bool left_contact, bool right_contact, bool left_contact_break, bool right_contact_break){
  std::stringstream ss;
  ss << left_contact << "" << right_contact << "  " << left_contact_break << "" << right_contact_break << " | " << mode_;
  return ss.str();
}

float foot_contact_classify::update (int64_t utime, Eigen::Isometry3d primary_foot, Eigen::Isometry3d secondary_foot){
  
  // 0. Filter the Foot Forces:
  // TODO: correct none-unity coeff sum at source: (a bug in dehann's code)
  float left_force_filtered  = left_force_;
  float right_force_filtered  = right_force_;
  if (1==0){ // low pass filter filtering?
    double scale = 1/1.0091;
    left_force_filtered  = scale*lpfilter_lfoot_.processSample( left_force_ );
    right_force_filtered  = scale*lpfilter_rfoot_.processSample( right_force_ );
  }
  
  // 1. Determine Weak or Strong Contact:
  left_contact_state_weak_->UpdateState(utime, left_force_filtered);
  right_contact_state_weak_->UpdateState(utime, right_force_filtered );
  bool lf_state = (bool) left_contact_state_weak_->getState();
  bool rf_state = (bool) right_contact_state_weak_->getState();
  left_contact_state_strong_->UpdateState(utime, left_force_filtered);
  right_contact_state_strong_->UpdateState(utime, right_force_filtered);
  bool lf_state_high = (bool) left_contact_state_strong_->getState();
  bool rf_state_high = (bool) right_contact_state_strong_->getState();
  //std::cout << utime << " " << left_force_filtered << " " << lf_state << " " << lf_state_high << "\n";
  //std::cout << utime << " " << right_force_filtered << " " << rf_state << " " << rf_state_high << "\n";
  
  ////////////////////////////////////////
  // 2. if the walking state mode: 
  updateWalkingPhase (utime, lf_state , rf_state, lf_state_high, rf_state_high );
  
  // 3. Determine the contact status:
  bool recent_left_strike = false;
  bool recent_right_strike = false;
  bool recent_left_break = false;
  bool recent_right_break = false;
  if (utime - last_strike_utime_  < strike_blackout_duration_){
    if (verbose_ >= 3) std::cout << utime << " recent contact strike\n";
    if (mode_ <4) 
      recent_right_strike = true;
    else 
      recent_left_strike = true;
  }
  if (utime - last_break_utime_  < break_blackout_duration_){
    if (verbose_ >= 3) std::cout << utime << " recent contact break\n";
    if (mode_ <4) 
      recent_right_break = true;
    else 
      recent_left_break = true;
  }  
  float odometry_status = 0.0; // by default assume accurate
  if ( recent_left_strike || recent_right_strike){
    odometry_status = -1.0; // unuseable
  }else if (recent_left_break || recent_right_break){
    odometry_status = 1.0; // very inaccurate
  }
  
  
  if (publish_diagnostics_){
    // 4. Output Foot Contact Estimates for visualization:
    float vis_scale = 2000; // to scale for use with signal scope
    drc::foot_contact_estimate_t msg_contact_est;
    msg_contact_est.utime = utime;
    msg_contact_est.detection_method = drc::foot_contact_estimate_t::DIFF_SCHMITT_WITH_DELAY;
    msg_contact_est.left_contact  = vis_scale*lf_state  - 100;
    msg_contact_est.right_contact = vis_scale*rf_state - 200;
    lcm_publish_->publish("FOOT_CONTACT_ESTIMATE",&msg_contact_est); // this message was used by the controller

    bool recent_left_contact = false;
    bool recent_right_contact = false;
    if ( recent_left_break || recent_left_strike)
      recent_left_contact = true;
    if ( recent_right_break || recent_right_strike)
      recent_right_contact = true;
    drc::foot_contact_estimate_t msg_contact_classify;
    msg_contact_classify.utime = utime;
    msg_contact_classify.detection_method = drc::foot_contact_estimate_t::DIFF_SCHMITT_WITH_DELAY;
    msg_contact_classify.left_contact =  (float) vis_scale* ((float)recent_left_contact)  - 300;
    msg_contact_classify.right_contact = (float) vis_scale* ((float)recent_right_contact) - 400;
    lcm_publish_->publish("FOOT_CONTACT_CLASSIFY",&msg_contact_classify);  
  }
  
  // Determine which points are in contact with the ground (stub)
  //determineContactPoints(utime, primary_foot, secondary_foot);
  
  return odometry_status; 
}


int foot_contact_classify::updateWalkingPhase (int64_t utime, bool left_contact, bool right_contact,
  bool left_contact_strong, bool right_contact_strong){
  string pv = print_variables(mode_,left_contact,right_contact,left_contact_strong, right_contact_strong);

  
  if (!initialized_){
    if (left_contact && right_contact){
      std::cout << pv << ">0 | Initializing. both in contact with left foot as primary\n";
      mode_ = LEFT_PRIME_RIGHT_STAND;
      initialized_ = true;
      return 2;
    }else{
      std::cout << pv << ">8 | Not initialized yet: both feet are not in contact\n";
      return -1; 
    }
  }
  
  if (mode_ ==LEFT_PRIME_RIGHT_STAND){
    if (left_contact  && !right_contact_strong){
      if (verbose_ >= 2) std::cout << pv << ">1 | primary left. right weak [LEFT_PRIME_RIGHT_BREAK]\n";
      mode_ = LEFT_PRIME_RIGHT_BREAK;
      last_break_utime_ = utime;      
      return 2;
    }else if (!left_contact_strong  && right_contact){
      if (verbose_ >= 1) std::cout << pv << ">5 | SWITCHING primary to right. left breaking [LEFT_BREAK_RIGHT_PRIME]\n"; //LEG ODOM SWITCH
      mode_ = LEFT_BREAK_RIGHT_PRIME;
      last_break_utime_ = utime;
      return 1;
    }else if (left_contact  && right_contact){
      if (verbose_ >= 3) std::cout << pv << ">0 | primary left. both in contact. still [LEFT_PRIME_RIGHT_STAND]\n";
      return 2;      
    }else{
      std::cout << "Unknown LEFT_PRIME_RIGHT_STAND Transition: " << pv<< "\n";
      int blah;
      cin >> blah;
    }
  }
  
  if (mode_ ==LEFT_PRIME_RIGHT_BREAK){
    if (left_contact  && !right_contact){
      std::cout << pv << ">2 | primary left but right now raised. [LEFT_PRIME_RIGHT_SWING]\n";
      mode_ = LEFT_PRIME_RIGHT_SWING;
      return -1;
    }else if (left_contact  && right_contact_strong){
      std::cout << pv << ">0 | primary left. right now in strong contact. [LEFT_PRIME_RIGHT_STAND]\n";
      mode_ = LEFT_PRIME_RIGHT_STAND;
      return -1;
    }else if (left_contact  && !right_contact_strong){
      if (verbose_ >= 3) std::cout << pv << ">1 | primary left still in contact but right weak. still\n";
      return -1;
    }else{
      std::cout << "Unknown LEFT_PRIME_RIGHT_BREAK Transition: " << pv<< "\n";
      int blah;
      cin >> blah;
    }
  }
  
  if (mode_ == LEFT_PRIME_RIGHT_SWING){
    if (left_contact  && !right_contact){
      if (verbose_ >= 3) std::cout << pv << ">2 | primary left. right raised. still\n";
      return 2;
    }else if (left_contact  && right_contact){
      if (verbose_ >= 2) std::cout << pv << ">3 | primary left. right now in contact [LEFT_PRIME_RIGHT_STRIKE]\n";
      mode_ = LEFT_PRIME_RIGHT_STRIKE;
      last_strike_utime_ = utime;
      return 2;
    }else{
      std::cout << "Unknown LEFT_PRIME_RIGHT_SWING Transition: " << pv<< "\n";
      int blah;
      cin >> blah;
    }
  }
  
  if (mode_ == LEFT_PRIME_RIGHT_STRIKE){
    if (left_contact  && right_contact_strong){
      std::cout << pv << ">0 | primary left. right now in strong contact [LEFT_PRIME_RIGHT_STAND] \n";
      mode_ = LEFT_PRIME_RIGHT_STAND;
      return -1;
    }else if (left_contact  && !right_contact_strong){
      if (verbose_ >= 3) std::cout << pv << ">3 | primary left. right weak. still [LEFT_PRIME_RIGHT_STRIKE] \n";
      return -1;
    }else{
      std::cout << "Unknown LEFT_PRIME_RIGHT_STRIKE Transition: " << pv<< "\n";
      int blah;
      cin >> blah;
    }
  }
    
  ///////////////////////////////////////////////  RIGHT PRIME MODES ////////////////////////////////////////////////////////////////////
  
  if (mode_ ==LEFT_STAND_RIGHT_PRIME){
    if (!left_contact_strong  && right_contact){
      if (verbose_ >= 2) std::cout << pv << ">5 | primary right. left weak [LEFT_BREAK_RIGHT_PRIME]\n";
      mode_ = LEFT_BREAK_RIGHT_PRIME;
      last_break_utime_ = utime;
      return 2;
    }else if (left_contact && !right_contact_strong){
      if (verbose_ >= 1) std::cout << pv << ">1 | SWITCHING primary to left. right breaking [LEFT_PRIME_RIGHT_BREAK]\n"; //LEG ODOM SWITCH
      mode_ = LEFT_PRIME_RIGHT_BREAK;
      last_break_utime_ = utime;
      return 1;
    }else if (left_contact  && right_contact){
      if (verbose_ >= 3) std::cout << pv << ">4 | primary left. both in contact. still [LEFT_STAND_RIGHT_PRIME]\n";
      return 2;      
    }else{
      std::cout << "Unknown LEFT_STAND_RIGHT_PRIME Transition: " << pv<< "\n";
      int blah;
      cin >> blah;
    }
  }  
    
    if (mode_ ==LEFT_BREAK_RIGHT_PRIME){
    if (!left_contact  && right_contact){
      std::cout << pv << ">6 | primary right. left now raised [LEFT_SWING_RIGHT_PRIME]\n";
      mode_ = LEFT_SWING_RIGHT_PRIME;
      return -1;
    }else if (left_contact_strong  && right_contact){
      std::cout << pv << ">4 | primary right. left strong contact [LEFT_STAND_RIGHT_PRIME]\n";
      mode_ = LEFT_STAND_RIGHT_PRIME;
      return -1;
    }else if (!left_contact_strong  && right_contact){
      if (verbose_ >= 3) std::cout << pv << ">5 | primary right. left weak. still [LEFT_BREAK_RIGHT_PRIME]\n";
      return -1;
    }else{
      std::cout << "Unknown LEFT_BREAK_RIGHT_PRIME Transition: " << pv<< "\n";
      int blah;
      cin >> blah;
    }
  }    
  
  if (mode_ == LEFT_SWING_RIGHT_PRIME){
    if (!left_contact  && right_contact){
      if (verbose_ >= 3) std::cout << pv << ">6 | primary right. left raised. still. [LEFT_SWING_RIGHT_PRIME]\n";
      return 3;
    }else if (left_contact  && right_contact){
      if (verbose_ >= 2) std::cout << pv << ">7 | primary right. left now in contact [LEFT_STRIKE_RIGHT_PRIME]\n";
      mode_ = LEFT_STRIKE_RIGHT_PRIME;
      last_strike_utime_ = utime;
      return 3;
    }else{
      std::cout << "Unknown LEFT_SWING_RIGHT_PRIME Transition: " << pv<< "\n";
      int blah;
      cin >> blah;
    }
  }
  
  
  if (mode_ == LEFT_STRIKE_RIGHT_PRIME){
    if (left_contact_strong  && right_contact){
      if (verbose_ >= 2) std::cout << pv << ">4 | primary right. left now in strong contact [LEFT_STAND_RIGHT_PRIME] \n";
      mode_ = LEFT_STAND_RIGHT_PRIME;
      return -1;
    }else if (!left_contact_strong  && right_contact){
      if (verbose_ >= 3) std::cout << pv << ">7 | primary right. left weak. still [LEFT_STRIKE_RIGHT_PRIME] \n";
      return -1;
    }else{
      std::cout << "Unknown LEFT_STRIKE_RIGHT_PRIME Transition: " << pv<< "\n";
      int blah;
      cin >> blah;
    }
  }  
  
  
  std::cout << "Unknown fallthrough: " << mode_ << " > " << left_contact << " " << right_contact << "\n";
  int blah;
  cin >> blah;  
  return -2;
}




//////
void foot_contact_classify::determineContactPoints(int64_t utime, Eigen::Isometry3d primary_foot, Eigen::Isometry3d secondary_foot){

  ////////////////////////////////////////////////////////// 
  bool success =false;
  
  // use primary and secondary foot to determine plane that CPs are one
  Isometry3dTime null_pose = Isometry3dTime(utime, Eigen::Isometry3d::Identity() );
  pc_vis_->pose_to_lcm_from_list(2001, null_pose);  
  
  // Determine moving contact points in stationary foot's sole frame:
  // I define the sole frame as a frame directly below the foot frame on the sole
  pcl::PointCloud<PointXYZRGB>::Ptr cp_moving(new pcl::PointCloud<pcl::PointXYZRGB> ());
  Eigen::Isometry3d foot_to_foot =  primary_foot.inverse() * secondary_foot * foot_to_sole_;
  pcl::transformPointCloud (*contact_points_, *cp_moving,
                           foot_to_foot.translation().cast<float>(), Eigen::Quaternionf(foot_to_foot.rotation().cast<float>()));    
  pc_vis_->ptcld_to_lcm_from_list(2004, *cp_moving, utime, utime);
  
  if (cp_moving_prev_->points.size() != 4){
    std::cout << "Previous contact points not four - we have a problem\n";
    success = false;
  }else{
    
    int n_points_in_contact = 0;
    for (size_t i=0; i < 4 ; i++){
      pcl::PointXYZRGB cp = cp_moving->points[i];
      pcl::PointXYZRGB cp_prev = cp_moving_prev_->points[i];
    
      double raise = fabs(cp.z);
       
      if ( raise < 0.02){
        n_points_in_contact++;
        //std::cout <<utime << " "<< raise << " " << (int)i << " cp in contact\n"; 
      }else{
        //std::cout <<utime << " "<< raise << " " << (int)i << " cp NOT in contact\n"; 
      }
    }
    
    if (n_points_in_contact >0){
      std::cout << utime << " " << n_points_in_contact << "\n";
    }
    
   
    
    success = true;
  }
  
  cp_moving_prev_ = cp_moving;

  // determine the velocity of the SF CPs onto the PFCP plane
  // infer the time to contact by differencing
  
  // If the distance of the foot to the plane is less than a certain amount
  // and the time to contact is less than a certain amount
  // then contact is likely
  
}