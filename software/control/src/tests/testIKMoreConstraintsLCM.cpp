#include <drake/RigidBodyIK.h>
#include <drake/RigidBodyManipulator.h>
#include <drake/RigidBodyConstraint.h>

#include <drake/IKoptions.h>
#include <iostream>
#include <cstdlib>
#include <limits>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace Eigen;

#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/robot_plan_w_keyframes_t.hpp"
#include <lcm/lcm-cpp.hpp>

class App{
  public:
    App(boost::shared_ptr<lcm::LCM> &lcm_);
    
    ~App(){
    }

    void getRobotPlan(int64_t utime_in, Eigen::MatrixXd q, std::vector<std::string> jointNames );
    void getRobotState(drc::robot_state_t& robot_state_msg, int64_t utime_in, Eigen::VectorXd q, std::vector<std::string> jointNames);


  private:
    boost::shared_ptr<lcm::LCM> lcm_;


};

App::App(boost::shared_ptr<lcm::LCM> &lcm_):
                       lcm_(lcm_){

}


Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {
  
  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0 
  // Post DRC Trails: replace these with Eigen's own conversions
  if ( ((roll==M_PI) && (pitch ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,1,0,0);
  }else if( ((pitch==M_PI) && (roll ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,0,1,0);
  }else if( ((yaw==M_PI) && (roll ==0)) && (pitch ==0)){
    return  Eigen::Quaterniond(0,0,0,1);
  }
  
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}


void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


void App::getRobotPlan(int64_t utime_in, Eigen::MatrixXd q, std::vector<std::string> jointNames ){//,  const  drc::force_torque_t& force_torque_msg){

  drc::robot_plan_w_keyframes_t p;

  std::cout << q.cols()  << " samples\n";
  for (int i=0; i < q.cols() ; i++){
    drc::robot_state_t robot_state_msg;
    getRobotState(robot_state_msg, i*1E6, q.col(i) , jointNames);
    p.plan.push_back(robot_state_msg);
  }

  p.num_keyframes =0;
  p.num_breakpoints =0;
  p.num_states = 2;
  p.num_grasp_transitions =0;
  p.num_bytes = 0;
  for (int i=0; i <p.num_states ; i++){
    p.is_keyframe.push_back(0);
    p.is_breakpoint.push_back(0);
    p.plan_info.push_back(1); // snopt number
  }
  lcm_->publish("CANDIDATE_MANIP_PLAN", &p); 
}


void App::getRobotState(drc::robot_state_t& robot_state_msg, int64_t utime_in, Eigen::VectorXd q, std::vector<std::string> jointNames){
  robot_state_msg.utime = utime_in;

  // Pelvis Pose:
  robot_state_msg.pose.translation.x =q(0);
  robot_state_msg.pose.translation.y =q(1);
  robot_state_msg.pose.translation.z =q(2);

  Eigen::Quaterniond quat = euler_to_quat( q(3), q(4), q(5));

  robot_state_msg.pose.rotation.w = quat.w();
  robot_state_msg.pose.rotation.x = quat.x();
  robot_state_msg.pose.rotation.y = quat.y();
  robot_state_msg.pose.rotation.z = quat.z();

  robot_state_msg.twist.linear_velocity.x  = 0;
  robot_state_msg.twist.linear_velocity.y  = 0;
  robot_state_msg.twist.linear_velocity.z  = 0;
  robot_state_msg.twist.angular_velocity.x = 0;
  robot_state_msg.twist.angular_velocity.y = 0;
  robot_state_msg.twist.angular_velocity.z = 0;

  // Joint States:
  for (size_t i = 0; i < jointNames.size(); i++)  {
    robot_state_msg.joint_name.push_back( jointNames[i] );
    robot_state_msg.joint_position.push_back( q(i) );
    robot_state_msg.joint_velocity.push_back( 0);
    robot_state_msg.joint_effort.push_back( 0 );
  }
  robot_state_msg.num_joints = robot_state_msg.joint_position.size();
}





int main(int argc, char *argv[])
{

  std::string file_path = "fname";
  if ((argc > 1)){
    file_path = argv[1];
  }else{
    std::cout << "you need to provide the path to atlas_minimal_contact.urdf\n";
    return 1;
  }

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  App app(lcm);

  RigidBodyManipulator rbm(file_path);
  // RigidBodyManipulator rbm("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  RigidBodyManipulator* model = &rbm;


  int r_hand, l_foot, r_foot;
  for(int i = 0;i<model->num_bodies;i++)
  {
    if(model->bodies[i]->linkname.compare(string("r_hand")) ==0 )
    {
      r_hand = i;
    }
    if(model->bodies[i]->linkname.compare(string("l_foot")) ==0 )
    { 
      l_foot = i;
    }
    if(model->bodies[i]->linkname.compare(string("r_foot")) ==0 )
    {
      r_foot = i;
    }
  }


  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }

  std::vector<string> jointNames;
  for (int i=0 ; i <model->num_positions ; i++){
    std::cout << model->getPositionName(i) << " " << i << "\n";
    jointNames.push_back( model->getPositionName(i) ) ;
  }  


  Vector2d tspan;
  tspan<<0,1;


  //VectorXd q0 = VectorXd::Zero(model->num_positions);
  // The state frame of cpp model does not match with the state frame of MATLAB model, since the dofname_to_dofnum is different in cpp and MATLAB
  //q0(2) = 0.8;

  //VectorXd qstar = VectorXd::Zero(model->num_positions);
  //qstar(2) = 0.8;
  VectorXd qstar(model->num_positions);
qstar <<  -0.0260, 0,  0.8440, 0, 0, 0, 0, 0, 0,  0.2700, 0,  0.0550, -0.5700,  1.1300, -0.5500, -0.0550, -1.3300,  2.1530,  0.5000,  0.0985, 0,  0.0008, -0.2700, 0, -0.0550, -0.5700,  1.1300, -0.5500,  0.0550,  1.3300,  2.1530, -0.5000,  0.0985, 0,  0.0008,  0.2564;
  //model->doKinematics(qstar, v);


  // 1 CoM Constraint
  Vector3d com_lb = Vector3d::Zero(); 
  Vector3d com_ub = Vector3d::Zero(); 
  com_lb(0) = -0.1;
  com_lb(1) = -0.1;
  com_lb(2) = 0.6;
  com_ub(0) = 0.1;
  com_ub(1) = 0.1;
  com_ub(2) = 1.0;
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_lb,com_ub,tspan);

  // 2 Back Posture Constraint
  PostureConstraint* kc_posture_back = new PostureConstraint(model, tspan);
  std::vector<int> back_idx;
  back_idx.push_back(6);
  back_idx.push_back(7);
  back_idx.push_back(8);
  VectorXd back_lb = VectorXd::Zero(3);
  VectorXd back_ub = VectorXd::Zero(3);
  kc_posture_back->setJointLimits(3, back_idx.data(), back_lb, back_ub);

  // 3 Knees Constraint
  PostureConstraint* kc_posture_knees = new PostureConstraint(model, tspan);
  std::vector<int> knee_idx;
  knee_idx.push_back(13);
  knee_idx.push_back(26);
  VectorXd knee_lb = VectorXd::Zero(2);
  knee_lb(0) = 1.0; // usually use 0.6
  knee_lb(1) = 1.0; // usually use 0.6
  VectorXd knee_ub = VectorXd::Zero(2);
  knee_ub(0) = 2.5;
  knee_ub(1) = 2.5;
  kc_posture_knees->setJointLimits(2, knee_idx.data(), knee_lb, knee_ub);

  // 4 Left Arm Constraint
  PostureConstraint* kc_posture_larm = new PostureConstraint(model, tspan);
  std::vector<int> larm_idx;
  larm_idx.push_back(9);
  larm_idx.push_back(16);
  larm_idx.push_back(17);
  larm_idx.push_back(18);
  larm_idx.push_back(19);
  larm_idx.push_back(20);
  larm_idx.push_back(21);
  VectorXd larm_lb = VectorXd::Zero(7);
  larm_lb(0) = 0.27;
  larm_lb(1) = -1.33;
  larm_lb(2) = 2.153;
  larm_lb(3) = 0.500;
  larm_lb(4) = 0.0985;
  larm_lb(5) = 0;
  larm_lb(6) = 0.0008;
  VectorXd larm_ub = larm_lb;//VectorXd::Zero(7);
  kc_posture_larm->setJointLimits(7, larm_idx.data(), larm_lb, larm_ub);

  // 5 Left Foot Position and Orientation Constraints
  Vector3d l_foot_pt = Vector3d::Zero();
  Vector3d lfoot_pos0;
  // = model->forwardKin(l_foot_pt, l_foot, 0, 0, 0).value();
  lfoot_pos0(0) = 0;
  lfoot_pos0(1) = 0.13;
  lfoot_pos0(2) = 0.08;
  Vector3d lfoot_pos_lb = lfoot_pos0;
  lfoot_pos_lb(0) +=0.001;
  lfoot_pos_lb(1) +=0.001;
  lfoot_pos_lb(2) +=0.001;
  Vector3d lfoot_pos_ub = lfoot_pos_lb;
  lfoot_pos_ub(2) += 0.01;
  std::cout << lfoot_pos0.transpose() << " lfoot\n" ;
  WorldPositionConstraint* kc_lfoot_pos = new WorldPositionConstraint(model,l_foot,l_foot_pt,lfoot_pos_lb,lfoot_pos_ub,tspan);
  Eigen::Vector4d quat_des(1,0,0,0);
  double tol = 0.0017453292519943296;
  WorldQuatConstraint* kc_lfoot_quat = new  WorldQuatConstraint(model, l_foot, quat_des, tol, tspan);

  // 5 Right Foot Position and Orientation Constraints
  Vector3d r_foot_pt = Vector3d::Zero();
  Vector3d rfoot_pos0;
  //Vector3d rfoot_pos0 = model->forwardKin(r_foot_pt, r_foot, 0, 0, 0).value();
  rfoot_pos0(0) = 0;
  rfoot_pos0(1) = -0.13;
  rfoot_pos0(2) = 0.08;
  Vector3d rfoot_pos_lb = rfoot_pos0;
  rfoot_pos_lb(0) +=0.001;
  rfoot_pos_lb(1) +=0.001;
  rfoot_pos_lb(2) +=0.001;
  Vector3d rfoot_pos_ub = rfoot_pos_lb;
  rfoot_pos_ub(2) += 0.001;
  std::cout << rfoot_pos0.transpose() << " lfoot\n" ;
  WorldPositionConstraint* kc_rfoot_pos = new WorldPositionConstraint(model,r_foot,r_foot_pt,rfoot_pos_lb,rfoot_pos_ub,tspan);
  WorldQuatConstraint* kc_rfoot_pos_quat = new  WorldQuatConstraint(model, r_foot, quat_des, tol, tspan);


  // 6 Right Position Constraints (actual reaching constraint)

  Vector3d r_hand_pt = Vector3d::Zero();
  Vector3d rhand_pos0;
  //Vector3d rhand_pos0 = model->forwardKin(r_hand_pt, r_hand, 0, 0, 0).value();
  rhand_pos0(0) = 0.2;
  rhand_pos0(1) = -0.5;
  rhand_pos0(2) = 0.4;

  Vector3d rhand_pos_lb = rhand_pos0;
  rhand_pos_lb(0) +=0.05;
  rhand_pos_lb(1) +=0.05;
  rhand_pos_lb(2) +=0.05;
  Vector3d rhand_pos_ub = rhand_pos_lb;
  rhand_pos_ub(2) += 0.05;
  std::cout << rhand_pos_ub.transpose() << " rhand\n" ;
  WorldPositionConstraint* kc_rhand = new WorldPositionConstraint(model,r_hand,r_hand_pt,rhand_pos_lb,rhand_pos_ub,tspan);


  // 7 QuasiStatic Constraints
  QuasiStaticConstraint* kc_quasi = new QuasiStaticConstraint(model, tspan);
  kc_quasi->setShrinkFactor(0.2);
  kc_quasi->setActive(true);
  Eigen::Matrix3Xd l_foot_pts = Eigen::Matrix3Xd::Zero(3, 4);
  l_foot_pts <<    -0.0820 ,  -0.0820 ,   0.1780 ,   0.1780   , 0.0624  , -0.0624  ,  0.0624 ,  -0.0624  , -0.0811,   -0.0811  , -0.0811 ,  -0.0811;
  std::cout << l_foot_pts <<"\n";
  kc_quasi->addContact(1, &l_foot, &l_foot_pts);
  Eigen::Matrix3Xd r_foot_pts = Eigen::Matrix3Xd::Zero(3, 4);
  r_foot_pts <<    -0.0820 ,  -0.0820 ,   0.1780 ,   0.1780   , 0.0624  , -0.0624  ,  0.0624 ,  -0.0624  , -0.0811,   -0.0811  , -0.0811 ,  -0.0811;
  kc_quasi->addContact(1, &r_foot, &r_foot_pts);



  int num_constraints = 9;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = kc_quasi;
  constraint_array[1] = kc_posture_knees;
  constraint_array[2] = kc_lfoot_pos;
  constraint_array[3] = kc_lfoot_quat;
  constraint_array[4] = kc_rfoot_pos;
  constraint_array[5] = kc_rfoot_pos_quat;
  constraint_array[6] = kc_rhand;
  constraint_array[7] = kc_posture_larm;
  constraint_array[8] = kc_posture_back;
  //  constraint_array[0] = com_kc;

  IKoptions ikoptions(model);
  VectorXd q_sol(model->num_positions);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(model,qstar,qstar,num_constraints,constraint_array,q_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    return 1;
  }



  /////////////////////////////////////////
  VectorXd v = VectorXd::Zero(0);
  model->doKinematics(q_sol, v);
  Vector3d com = model->centerOfMass<double>(0).value();
  printf("%5.2f\n%5.2f\n%5.2f\n",com(0),com(1),com(2));
  drc::robot_state_t robot_state_msg;
  app.getRobotState(robot_state_msg, 0*1E6, q_sol , jointNames);

  lcm->publish("EST_ROBOT_STATE",&robot_state_msg);



  delete com_kc;
  delete[] constraint_array;








  return 0;
}
