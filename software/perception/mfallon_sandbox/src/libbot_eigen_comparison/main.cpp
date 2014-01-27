
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>

#include <bot_core/trans.h>


std::string print_Isometry3d(Eigen::Isometry3d pose){
  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  
  std::stringstream ss;
  ss <<t[0]<<", "<<t[1]<<", "<<t[2]<<", " 
       <<r.w()<<", "<<r.x()<<", "<<r.y()<<", "<<r.z() ;
  return ss.str();
}

std::string print_BotTrans(BotTrans *bt){
  std::stringstream ss;
  ss << bt->trans_vec[0] <<", "<<bt->trans_vec[1]<<", "<<bt->trans_vec[2]<<", " 
       <<bt->rot_quat[0]<<", "<<bt->rot_quat[1]<<", "<<bt->rot_quat[2]<<", "<<bt->rot_quat[3] ;
  return ss.str();
}

void setBotTrans(BotTrans *bt, double x, double y, double z, double qw, double qx, double qy, double qz){
  bt->trans_vec[0] = x;  bt->trans_vec[1] = y;  bt->trans_vec[2] = z;
  bt->rot_quat[0] = qw;  bt->rot_quat[1] = qx;
  bt->rot_quat[2] = qy;  bt->rot_quat[3] = qz;  
}


Eigen::Isometry3d setIsometry3dFromBotTrans(BotTrans *bt){
  Eigen::Isometry3d tf;
  tf.setIdentity();
  tf.translation()  << bt->trans_vec[0], bt->trans_vec[1],bt->trans_vec[2];
  Eigen::Quaterniond q = Eigen::Quaterniond(bt->rot_quat[0], bt->rot_quat[1], bt->rot_quat[2], bt->rot_quat[3]);  
  tf.rotate(q); 

  return tf;
}

int 
main(int argc, char ** argv){
  
   double trans_vec[] = {-2.7363, 0.5958, -1.1588};
   double rot_quat[] = {0.8977, 0.0011, -9.0933e-04, -0.4407};
  
  BotTrans init_vicon, init_est, current_est;
  setBotTrans(&init_vicon, 1.4696,  0.5235, 0.8753, 0.9749, -0.0108, 0.0076, 0.2221);
  setBotTrans(&init_est, 2.1455, 1.8038, 1.1523, 0.8977, -0.0011, 9.0933e-04, 0.4407);
  setBotTrans(&current_est, 2.1456, 1.8038, 1.1523, 0.8976, -0.0011, 9.2603e-04, 0.4407);
  
  Eigen::Isometry3d init_vicon_e = setIsometry3dFromBotTrans(&init_vicon);
  Eigen::Isometry3d init_est_e = setIsometry3dFromBotTrans(&init_est);
  Eigen::Isometry3d current_est_e = setIsometry3dFromBotTrans(&current_est);
  
  std::cout << print_Isometry3d(current_est_e) << " current_est_e\n";  
  std::cout << print_Isometry3d(init_est_e) << " init_est_e\n";  
  Eigen::Isometry3d init_est_e_inv = init_est_e.inverse();
  std::cout << print_Isometry3d(init_est_e_inv) << " init_est_e.inverse\n";    
  
  Eigen::Isometry3d delta_e =  init_est_e_inv * current_est_e;
  std::cout << print_Isometry3d(delta_e) << " delta_e\n";    
  
  
  
  BotTrans init_est_inv;
  BotTrans* init_est_ptr = &init_est;
  bot_trans_copy(&init_est_inv, init_est_ptr);
  
  
  bot_trans_invert ( &init_est_inv );
  
  std::cout << print_BotTrans(&current_est) << " current_est\n";    

  
  std::cout << print_BotTrans(&init_est_inv) << " init_est_inv\n";    
  
  BotTrans delta;
  BotTrans* delta_ptr = &current_est;
  bot_trans_apply_trans( delta_ptr  ,  &init_est_inv );
  std::cout << print_BotTrans(&current_est) << " delta\n";    
  
  
  return 0;
}