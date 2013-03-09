#include <iostream>

#include "rng.hpp"

#include "particle.hpp"
// sandymfallon:
// #include <math_utils/math_utils.hpp>


#define VERBOSE_TXT 0

// STATUS OF NOISE DRIFT:
// when vo fails, last dvo is scaled to give dvo with added noise
// - each particle has the same vo except for the added noise.
// TODO:
// smooth the velocity using a fraction of the previous velocity e.g. 50:50
void Particle::MoveParticleDrift(rng *pRng, std::vector<double> move_var,double dtime){
  // Scale velocity to give dpose:
  Eigen::Vector3d v_t(state.velocity.translation());
  Eigen::Quaterniond v_r(state.velocity.rotation());

  if (VERBOSE_TXT){
    std::cout << dtime << "dtime\n";
    std::stringstream ss;// (stringstream::in | stringstream::out);
    print_Isometry3d(state.velocity,ss);
    std::cout << "state.velocity: " << ss.str() << "\n";
  }


  Eigen::Vector3d v_t_scaled (dtime*v_t[0], dtime*v_t[1], dtime*v_t[2]);
  Eigen::Quaterniond v_r_scaled;
  scale_quaternion(dtime,v_r,v_r_scaled);

  Eigen::Isometry3d odom_dpose;
  odom_dpose.setIdentity();
  odom_dpose.translation() << v_t_scaled[0],v_t_scaled[1],v_t_scaled[2];  
  odom_dpose.rotate(v_r_scaled);

  if (VERBOSE_TXT){
    std::stringstream ss2;// (stringstream::in | stringstream::out);
    print_Isometry3d(odom_dpose,ss2);
    std::cout << "odom_dpose: " << ss2.str() << "\n";

    std::stringstream ss3;
    print_Isometry3d(state.pose,ss3);
    std::cout << "state.pose before: " << ss3.str() << "\n";
  }

  state.pose = state.pose*odom_dpose;

  if (VERBOSE_TXT){
    std::stringstream ss4;
    print_Isometry3d(state.pose,ss4);
    std::cout << "state.pose mid: " << ss4.str() << "\n";
  }

  // Apply noise to position:
  double ypr[3];
  ypr[0] =pRng->Normal(0,sqrt(move_var[1]));
  ypr[1] =pRng->Normal(0,sqrt(move_var[1]));
  ypr[2] =pRng->Normal(0,sqrt(move_var[1]));
  Eigen::Quaterniond m = euler_to_quat(ypr[0], ypr[1], ypr[2]);
  Eigen::Isometry3d noise_dpose;
  noise_dpose.setIdentity();
  noise_dpose.translation() << pRng->Normal(0,sqrt(move_var[0])),
      pRng->Normal(0,sqrt(move_var[0])),
      pRng->Normal(0,sqrt(move_var[0]));  
  noise_dpose.rotate(m);

  if (VERBOSE_TXT){
    std::stringstream ss5;
    print_Isometry3d(noise_dpose,ss5);
    std::cout << "noise_dpose: " << ss5.str() << "\n";
  }

  state.pose =state.pose*noise_dpose;

  if (VERBOSE_TXT){
    std::stringstream ss6;
    print_Isometry3d(state.pose,ss6);
    std::cout << "state.pose end: " << ss6.str() << "\n";
    if ( isnan_particle (state.pose.translation().x() )){
      std::cout << "paused\n";
      int pause;
      std::cin >> pause;
    }
  }

  /// To estimate the velocity: 
  // two possible ways:
  // difference in pose + noise (no use of previous term) <-------------- current implementation
  // OR  
  // the above combined with the previous velocity [i.e. smoothed]
  // VELOCITY has NO effect on pose during normal operation

  Eigen::Isometry3d noise_dpose_combined;
  noise_dpose_combined.setIdentity();
  noise_dpose_combined = noise_dpose_combined*odom_dpose;
  // Noise effect removed - for now.
  //noise_dpose_combined = noise_dpose_combined*noise_dpose;

  // Scale it:
  Eigen::Vector3d t_temp(noise_dpose_combined.translation());
  Eigen::Quaterniond r_temp(noise_dpose_combined.rotation());
  Eigen::Vector3d t_temp_scaled (t_temp[0]/dtime, t_temp[1]/dtime, t_temp[2]/dtime);
  Eigen::Quaterniond r_temp_scaled;
  scale_quaternion((1/dtime),r_temp,r_temp_scaled);  
  Eigen::Isometry3d dvel;
  dvel.setIdentity();
  dvel.translation() << t_temp_scaled[0],t_temp_scaled[1],t_temp_scaled[2];  
  dvel.rotate(r_temp_scaled);

  Eigen::Isometry3d noise_dvel;
  noise_dvel.setIdentity();
  noise_dvel.translation() << pRng->Normal(0,sqrt(move_var[2])),
      pRng->Normal(0,sqrt(move_var[2])),
      pRng->Normal(0,sqrt(move_var[2]));  
  noise_dvel.rotate(m);
  state.velocity =dvel*noise_dvel;  



  if (VERBOSE_TXT){
    if ( isnan_particle (  state.velocity.translation().x()      )){
      std::stringstream ss6z;
      print_Isometry3d(dvel,ss6z);
      std::cout << "tt dvel after set: " << ss6z.str() << " ["<< "\n";

      std::stringstream ss6zq;
      print_Isometry3d(noise_dvel,ss6zq);
      std::cout << "tt noise_dvel after set: " << ss6zq.str() << " ["<< "\n";



      std::cout << move_var[2] << " is var2\n";
      std::stringstream ss6;
      print_Isometry3d(state.velocity,ss6);
      std::cout << "tt state.velocity after set: " << ss6.str() << " ["<< "\n";
      std::cout << "paused\n";
      int pause;
      std::cin >> pause;

    }

    Eigen::Quaterniond m_velXXX(state.velocity.rotation());
    if ( isnan_particle (m_velXXX.w() )){
      std::cout << move_var[2] << " is var2\n";
      std::stringstream ss6;
      print_Isometry3d(state.velocity,ss6);
      std::cout << "rr state.velocity after set: " << ss6.str() << " ["<< "\n";
      std::cout << "paused\n";
      int pause;
      std::cin >> pause;
    }
  }


  return ;
}


void Particle::MoveParticle(rng *pRng, pf_state odom_diff,std::vector<double> move_var,double dtime){
  Eigen::Isometry3d odom_dpose = odom_diff.pose;
  state.pose = state.pose*odom_dpose;
  
  // Apply noise to position:
  double ypr[3];
  ypr[0] =pRng->Normal(0,sqrt(move_var[1]));
  ypr[1] =pRng->Normal(0,sqrt(move_var[1]));
  ypr[2] =pRng->Normal(0,sqrt(move_var[1]));
  Eigen::Quaterniond m = euler_to_quat(ypr[0], ypr[1], ypr[2]);
  Eigen::Isometry3d noise_dpose;
  noise_dpose.setIdentity();
    noise_dpose.translation() << pRng->Normal(0,sqrt(move_var[0])),
      pRng->Normal(0,sqrt(move_var[0])),
      pRng->Normal(0,sqrt(move_var[0]));  
  noise_dpose.rotate(m);
  state.pose =state.pose*noise_dpose;
  
  /// To estimate the velocity: 
  // vel_k =  P * vel_(k-1) + (1-P) * ( dVO_k / dt)  + noise  <-------------- current implementation
  //            previous          vo component         noise
  // VELOCITY has NO effect on the pose here

  double prop_prev = 0.8; // set this to zero for
  double prop_vo = 1 - prop_prev;

  // P * vel_(k-1) [ Momentum from Previous Velocity]
  Eigen::Vector3d t_vel(state.velocity.translation());
  Eigen::Quaterniond r_vel(state.velocity.rotation());
  Eigen::Vector3d t_vel_scaled (t_vel[0]*prop_prev, t_vel[1]*prop_prev, t_vel[2]*prop_prev);
  Eigen::Quaterniond r_vel_scaled;
  scale_quaternion( prop_prev ,r_vel,r_vel_scaled);
  Eigen::Isometry3d dvel_prev;
  dvel_prev.setIdentity();
  dvel_prev.translation() << t_vel_scaled[0],t_vel_scaled[1],t_vel_scaled[2];
  dvel_prev.rotate(r_vel_scaled);

  // (1-P) * ( dVO_k / dt)     [ Proportion from current VO ]
  double vo_scale = prop_vo*(1/dtime);   // Scale it:
  Eigen::Isometry3d noise_dpose_combined;
  noise_dpose_combined.setIdentity();
  noise_dpose_combined = noise_dpose_combined*odom_dpose; // Noise effect removed - for now.
  Eigen::Vector3d t_temp(noise_dpose_combined.translation());
  Eigen::Quaterniond r_temp(noise_dpose_combined.rotation());
  Eigen::Vector3d t_temp_scaled (t_temp[0]*vo_scale, t_temp[1]*vo_scale, t_temp[2]*vo_scale);
  Eigen::Quaterniond r_temp_scaled;
  scale_quaternion( vo_scale ,r_temp,r_temp_scaled);
  Eigen::Isometry3d dvel_vo;
  dvel_vo.setIdentity();
  dvel_vo.translation() << t_temp_scaled[0],t_temp_scaled[1],t_temp_scaled[2];
  dvel_vo.rotate(r_temp_scaled);
  
  // Noise
  Eigen::Isometry3d noise_dvel;
  noise_dvel.setIdentity();
    noise_dvel.translation() << pRng->Normal(0,sqrt(move_var[2])),
      pRng->Normal(0,sqrt(move_var[2])),
      pRng->Normal(0,sqrt(move_var[2]));  
  noise_dvel.rotate(m);
  state.velocity =  dvel_prev*dvel_vo*noise_dvel;

  if (VERBOSE_TXT){
    if ( isnan_particle (  state.velocity.translation().x()      )){
      std::cout << move_var[2] << " is var2\n";
      std::stringstream ss6;
      print_Isometry3d(state.velocity,ss6);
      std::cout << "tt state.velocity after set: " << ss6.str() << " ["<< "\n";
      std::cout << "paused\n";
      int pause;
      std::cin >> pause;

    }

    Eigen::Quaterniond m_velXXX(state.velocity.rotation());
    if ( isnan_particle (m_velXXX.w() )){
      std::cout << move_var[2] << " is var2\n";
      std::stringstream ss6;
      print_Isometry3d(state.velocity,ss6);
      std::cout << "rr state.velocity after set: " << ss6.str() << " ["<< "\n";
      std::cout << "paused\n";
      int pause;
      std::cin >> pause;
    }
  }

  return ;
}
  

void Particle::InitializeState(rng *pRng, double init_weight,
	Eigen::Isometry3d init_pose, std::vector <double> initial_var){
  // Add noise to position:
  double ypr[3];
  ypr[0] =pRng->Normal(0,sqrt(initial_var[1]));
  ypr[1] =pRng->Normal(0,sqrt(initial_var[1]));
  ypr[2] =pRng->Normal(0,sqrt(initial_var[1]));
  Eigen::Quaterniond m = euler_to_quat(ypr[0], ypr[1], ypr[2]);
  
  Eigen::Isometry3d noise_dpose;
  noise_dpose.setIdentity();
  noise_dpose.translation() << pRng->Normal(0,sqrt(initial_var[0])),
      pRng->Normal(0,sqrt(initial_var[0])),
      pRng->Normal(0,sqrt(initial_var[0]));  
  noise_dpose.rotate(m);
  state.pose =init_pose * noise_dpose;

  // Set velocity with known state
  double ypr_velocity[3];
  ypr_velocity[0] =pRng->Normal(0,sqrt(initial_var[3]));
  ypr_velocity[1] =pRng->Normal(0,sqrt(initial_var[3]));
  ypr_velocity[2] =pRng->Normal(0,sqrt(initial_var[3]));
  Eigen::Quaterniond m_vel = euler_to_quat(ypr_velocity[0], ypr_velocity[1], ypr_velocity[2]);

  Eigen::Isometry3d init_vel;
  init_vel.setIdentity();
  init_vel.translation() << pRng->Normal(0,sqrt(initial_var[2])),
      pRng->Normal(0,sqrt(initial_var[2])),
      pRng->Normal(0,sqrt(initial_var[2]));
  init_vel.rotate(m_vel);
  state.velocity =init_vel;

  logweight = init_weight;

  if (VERBOSE_TXT){
    if ( isnan_particle (  init_vel.translation().x()      )){
      std::cout << initial_var[2] << " is var2\n";
      std::cout << initial_var[3] << " is var3\n";
      std::stringstream ss6;
      print_Isometry3d(state.velocity,ss6);
      std::cout << "tt state.velocity after set: " << ss6.str() << " ["<< "\n";
      std::cout << "paused\n";
      int pause;
      std::cin >> pause;

    }

    if ( isnan_particle (m_vel.w() )){
      std::cout << initial_var[2] << " is var2\n";
      std::cout << initial_var[3] << " is var3\n";
      std::stringstream ss6;
      print_Isometry3d(state.velocity,ss6);
      std::cout << "rr state.velocity after set: " << ss6.str() << " ["<< "\n";
      std::cout << "paused\n";
      int pause;
      std::cin >> pause;
    }
  }



}  

