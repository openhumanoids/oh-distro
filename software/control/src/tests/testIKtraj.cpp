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





int main()
{

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM() );
  App app(lcm);

  RigidBodyManipulator rbm("examples/Atlas/urdf/atlas_minimal_contact.urdf");
  RigidBodyManipulator* model = &rbm;



  if(!model)
  {
    cerr<<"ERROR: Failed to load model"<<endl;
  }

  std::vector<string> jointNames;
  for (int i=0 ; i <model->num_positions ; i++){
//    std::cout << model->getPositionName(i) << " " << i << "\n";
    jointNames.push_back( model->getPositionName(i) ) ;
  }  


  if (1==0){

  Vector2d tspan;
  tspan<<0,1;
  int l_hand;
  int r_hand;
  //int l_foot;
  //int r_foot;
  for(int i = 0;i<model->num_bodies;i++)
  {
    if(model->bodies[i]->linkname.compare(string("l_hand")))
    {
      l_hand = i;
    }
    else if(model->bodies[i]->linkname.compare(string("r_hand")))
    {
      r_hand = i;
    }
    //else if(model->bodies[i].linkname.compare(string("l_foot")))
    //{
    //  l_foot = i;
    //}
    //else if(model->bodies[i].linkname.compare(string("r_foot")))
    //{
    //  r_foot = i;
    //}
  }
  int nq = model->num_positions;
  VectorXd qstar = VectorXd::Zero(nq);
  qstar(2) = 0.8;
  VectorXd v = VectorXd::Zero(0);
  model->doKinematics(qstar, v);
  Vector3d com0 = model->centerOfMass<double>(0).value();

  Vector3d r_hand_pt = Vector3d::Zero();
  Vector3d rhand_pos0 = model->forwardKin(r_hand_pt, r_hand, 0, 0, 0).value();

  int nT = 4;
  double* t = new double[nT];
  double dt = 1.0/(nT-1);
  for(int i = 0;i<nT;i++)
  {
    t[i] = dt*i;
  }
  MatrixXd q0 = qstar.replicate(1,nT);
  VectorXd qdot0 = VectorXd::Zero(model->num_velocities);
  Vector3d com_lb = com0;
  com_lb(0) = std::numeric_limits<double>::quiet_NaN();
  com_lb(1) = std::numeric_limits<double>::quiet_NaN();
  Vector3d com_ub = com0;
  com_ub(0) = std::numeric_limits<double>::quiet_NaN();
  com_ub(1) = std::numeric_limits<double>::quiet_NaN();
  com_ub(2) = com0(2)+0.5;

  com_lb(2) = 0.7;
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_lb,com_ub);

  std::cout << com_lb << " lb\n";
  std::cout << com_ub << " up\n";
  Vector3d rhand_pos_lb = rhand_pos0;
  rhand_pos_lb(0) +=0.1;
  rhand_pos_lb(1) +=0.05;
  rhand_pos_lb(2) +=0.25;
  Vector3d rhand_pos_ub = rhand_pos_lb;
  rhand_pos_ub(2) += 0.25;
  Vector2d tspan_end;
  tspan_end<<t[nT-1],t[nT-1];
  WorldPositionConstraint* kc_rhand = new WorldPositionConstraint(model,r_hand,r_hand_pt,rhand_pos_lb,rhand_pos_ub,tspan_end);
  int num_constraints = 2;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  constraint_array[1] = kc_rhand;
  IKoptions ikoptions(model);
  MatrixXd q_sol(model->num_positions,nT);
  MatrixXd qdot_sol(model->num_velocities,nT);
  MatrixXd qddot_sol(model->num_positions,nT);
  int info = 0;
  vector<string> infeasible_constraint;
  inverseKinTraj(model,nT,t,qdot0,q0,q0,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    cerr<<"Failure"<<endl;
    return 1;
  }
/*
  ikoptions.setFixInitialState(false);
  ikoptions.setMajorIterationsLimit(500);
  inverseKinTraj(model,nT,t,qdot0,q0,q0,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    cerr<<"Failure"<<endl;
    return 1;
  }
*/
/*
  RowVectorXd t_inbetween(5);
  t_inbetween << 0.1,0.15,0.3,0.4,0.6;
  ikoptions.setAdditionaltSamples(t_inbetween);
  inverseKinTraj(model,nT,t,qdot0,q0,q0,num_constraints,constraint_array,q_sol,qdot_sol,qddot_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    cerr<<"Failure"<<endl;
    return 1;
  }
  */




  std::cout << q_sol.transpose() << "\n";

/*
  std::cout << q0.transpose() << "\n";

  std::cout << model->num_positions << " pos\n";

  std::cout << model->num_bodies << " bodies\n";



  std::cout << "[";
  for (int i=0 ; i < model->num_positions; i++){
    std::cout << q_sol(i,3);
    if (i< model->num_positions -1) 
      std::cout << ", " ;
    else
      std::cout << "]\n";
  }  
  */



  app.getRobotPlan(0, q_sol, jointNames);

  delete com_kc;
  delete[] constraint_array;
  delete[] t;

  }else{

  Vector2d tspan;
  tspan<<0,1;
  VectorXd q0 = VectorXd::Zero(model->num_positions);
  // The state frame of cpp model does not match with the state frame of MATLAB model, since the dofname_to_dofnum is different in cpp and MATLAB
  q0(2) = 0.8;
  Vector3d com_lb = Vector3d::Zero(); 
  Vector3d com_ub = Vector3d::Zero(); 
  com_lb(2) = 0.9;
  com_ub(2) = 1.0;
  WorldCoMConstraint* com_kc = new WorldCoMConstraint(model,com_lb,com_ub,tspan);




  int r_hand;
  for(int i = 0;i<model->num_bodies;i++)
  {
    if(model->bodies[i]->linkname.compare(string("r_hand")))
    {
      r_hand = i;
    }
  }
  int nq = model->num_positions;
  VectorXd qstar = VectorXd::Zero(nq);
  qstar(2) = 0.8;
  VectorXd v = VectorXd::Zero(0);
  model->doKinematics(qstar, v);
  Vector3d com0 = model->centerOfMass<double>(0).value();

  Vector3d r_hand_pt = Vector3d::Zero();
  Vector3d rhand_pos0 = model->forwardKin(r_hand_pt, r_hand, 0, 0, 0).value();



  Vector3d rhand_pos_lb = rhand_pos0;
  rhand_pos_lb(0) +=0.2;
  rhand_pos_lb(1) +=0.05;
  rhand_pos_lb(2) +=0.25;
  Vector3d rhand_pos_ub = rhand_pos_lb;
  rhand_pos_ub(2) += 0.25;

  std::cout << rhand_pos_ub.transpose() << " rhand\n" ;
  Vector2d tspan_end;

  WorldPositionConstraint* kc_rhand = new WorldPositionConstraint(model,r_hand,r_hand_pt,rhand_pos_lb,rhand_pos_ub,tspan);





  int num_constraints = 2;
  RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];
  constraint_array[0] = com_kc;
  constraint_array[1] = kc_rhand;

  IKoptions ikoptions(model);
  VectorXd q_sol(model->num_positions);
  int info;
  vector<string> infeasible_constraint;
  inverseKin(model,q0,q0,num_constraints,constraint_array,q_sol,info,infeasible_constraint,ikoptions);
  printf("INFO = %d\n",info);
  if(info != 1)
  {
    return 1;
  }
//  VectorXd v = VectorXd::Zero(0);
  model->doKinematics(q_sol, v);
  Vector3d com = model->centerOfMass<double>(0).value();
  printf("%5.2f\n%5.2f\n%5.2f\n",com(0),com(1),com(2));


    drc::robot_state_t robot_state_msg;
    app.getRobotState(robot_state_msg, 0*1E6, q_sol , jointNames);
  lcm->publish("EST_ROBOT_STATE",&robot_state_msg);

  delete com_kc;
  delete[] constraint_array;




  }






  return 0;
}
