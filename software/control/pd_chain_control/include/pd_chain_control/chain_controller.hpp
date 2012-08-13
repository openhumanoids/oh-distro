#ifndef CHAIN_CONTROLLER_HPP
#define CHAIN_CONTROLLER_HPP


#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <time.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"

#include <boost/function.hpp>
#include <map>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Core> 
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <string>
#include <urdf/model.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>

#include <cmath>
#include "pd_chain_control/eigen_kdl_conversions.hpp"
#include "pd_chain_control/angles.hpp"

namespace pd_chain_control
{
#define USE_J_TRANSPOSE_CONTROLLER 
    // Implementation replicates ROS's IK for PR2
//#define USE_J_PINV_CONTROLLER  // requires Weighted Jacobian and Damped Pinv to work.
    // Implementation is based on the following paper (+ NULL SPACE biasing)
    // T. Sugihara, "Solvability-Unconcerned Inverse Kinematics
    // by the Levenberg-Marquardt method", IEEE Transactions on Robotics, 
    // Vol. 27, No. 5, 2011.  

   enum {STOPPED, RUNNING};

   double getTime_now()
   {
	struct timeval tv;
	gettimeofday (&tv,NULL);
	return (int64_t) tv.tv_sec*1000000+tv.tv_usec;
   };
   


  template <int JOINTS>
  struct Kin
  {
    typedef Eigen::Matrix<double, JOINTS, 1> JointVector;
    typedef Eigen::Matrix<double, 6, JOINTS> Jacobian;
  
    Kin(const KDL::Chain &kdl_chain) :
      fk_solver_(kdl_chain), jac_solver_(kdl_chain),
      kdl_q(JOINTS), kdl_J(JOINTS)
    {
    }
    ~Kin()
    {
    }
  
    void fk(const JointVector &q, Eigen::Affine3d &x)
    {
      kdl_q.data = q;
      KDL::Frame kdl_x;
      fk_solver_.JntToCart(kdl_q, kdl_x);
      transformKDLToEigen(kdl_x, x);
    }
    void jac(const JointVector &q, Jacobian &J)
    {
      kdl_q.data = q;
      jac_solver_.JntToJac(kdl_q, kdl_J);
      J = kdl_J.data;
    }
  
    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainJntToJacSolver jac_solver_;
    KDL::JntArray kdl_q;
    KDL::Jacobian kdl_J;
  }; // end struct Kin


// Example instantiation
//-------------------------------------
// int NrOfJoints  = kdl_chain.getNrOfJoints();
// pd_chain_control::ChainController<NrOfJoints> left_arm_controller(lcm,"LEFT_ARM_CMDS",kdl_chain);

  template <int JOINTS> // non-type template parameter
  class ChainController
  {

  public:
    // Ensure 128-bit alignment for Eigen
    // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //----------------constructor/destructor
    ChainController(boost::shared_ptr<lcm::LCM> &lcm, const std::string &input_cmds_channel,
			 const KDL::Chain &chain, const std::string &robot_name,const urdf::Model &urdf_robot_model);
    ~ChainController();

     void init();
     void update(const std::map<std::string, double> &jointpos_in, const std::map<std::string, double> &jointvel_in, const double &dt);
     void computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d  			&xdes, Eigen::Matrix<double,6,1> &err);
     bool isRunning();
     
    int64_t latest_robotstate_timestamp;
  private:
    //--------type defs
    //enum { JOINTS = 6 };
    typedef Eigen::Matrix<double, JOINTS, 1> JointVector;
    typedef Eigen::Matrix<double, 6, 1> CartVector;
    typedef Eigen::Matrix<double, 6, JOINTS> Jacobian;
    typedef Eigen::Matrix<double,Eigen::Dynamic,1> VectorXq;
   //--------fields
  private:
    std::string _robot_name;
    urdf::Model _urdf_robot_model;
    std::vector<std::string> _chain_joint_names;
    JointVector q_lower_limit,q_upper_limit,q_effort_limit,q_velocity_limit;
   
    
    uint controller_state;

    boost::shared_ptr<lcm::LCM> _lcm;    

   Eigen::Affine3d x, x_desi_, x_desi_filtered_;  
   CartVector xdot_desi_;

   KDL::Chain kdl_chain;
   boost::scoped_ptr<Kin<JOINTS> > kin_; // Kinematics Object 
  
  //--------Gains
  CartVector Kp_x, Kd_x;
  JointVector Kp_j, Kd_j, pclamp_j;
  JointVector Kff_j;
  Eigen::Matrix<double,6,1> W; // Weighting matrix for Error and Jacobian, only used for Jpinv control.

  
  double vel_saturation_trans_, vel_saturation_rot_;
  double pose_command_filter_;
  double joint_vel_filter_;
  double jacobian_inverse_damping_;
  JointVector q_posture_;
  double k_posture_;
  double k_posture_dot_;
  bool use_posture_;
  bool uninitialized_;

  // current joints state; Updated via robotState message callback
  JointVector q_measured_;
  JointVector qdot_measured_;
  JointVector qdot_filtered_;


 int64_t latest_goal_timestamp_;

    //-------------message callback
  private:  
    /*void handleRobotStateMsg(const lcm::ReceiveBuffer* rbuf,
			      const std::string& chan, 
			      const drc::robot_state_t* msg);*/
    void handleEndEffectorCmdMsg(const lcm::ReceiveBuffer* rbuf,
				 const std::string& chan, 
				 const drc::ee_goal_t* msg);
  void enforce_joint_limits(JointVector &q);
  void enforce_joint_vel_limits(JointVector &qdot);
  void enforce_joint_effort_limits(JointVector &tau);
  void enforce_cart_vel_limits(CartVector &xdot);
  void print_joint_vector(std::string &id,JointVector &v);
  void print_cart_vector(std::string &id,CartVector &v);
  void print_3D_position(std::string &id, Eigen::Affine3d &x);

  bool debug_;
public:
    JointVector q_proxy_;
  }; //class ChainController



  template <typename T> 
  int sgn(T val) {
      return (T(0) < val) - (val < T(0));
  }

  //==================constructor 
template <int JOINTS>
ChainController<JOINTS>::ChainController(boost::shared_ptr<lcm::LCM> &lcm, 
	const std::string& input_cmds_channel,
	const KDL::Chain& chain,
	const std::string &robot_name,
	const urdf::Model &urdf_robot_model) : kdl_chain(chain), _lcm(lcm), _robot_name(robot_name), _urdf_robot_model(urdf_robot_model)
{
    //lcm ok?
    std::cout << "\n Constructing ChainController Instance that listens to "<< input_cmds_channel << " channel." << std::endl;
   controller_state = STOPPED;
   
    if(!_lcm->good())
      {
	std::cerr << "\nLCM Not Good: chain_controller" << std::endl;
	return;
      }


  if (kdl_chain.getNrOfJoints() != JOINTS)
  {
    std::cerr << "The ChainController was instantiated with" << JOINTS << " joints, but the chain has "<< kdl_chain.getNrOfJoints() << "joints."<< std::endl; 
    return;
  } 

   std::cout<< "\n Spawning a Chaincontroller for the following joints:"<< std::endl;  
  for (unsigned int i=0; i< kdl_chain.getNrOfSegments() ; i++) {
 
     KDL::Joint joint = kdl_chain.segments[i].getJoint();
     std::cout<< joint.getName() << std::endl;  
       _chain_joint_names.push_back(joint.getName()); 
      q_lower_limit[i] = _urdf_robot_model.getJoint(joint.getName())->limits->lower;
      q_upper_limit[i] = _urdf_robot_model.getJoint(joint.getName())->limits->upper;
      q_velocity_limit[i] = _urdf_robot_model.getJoint(joint.getName())->limits->velocity;
      q_effort_limit[i] = _urdf_robot_model.getJoint(joint.getName())->limits->effort;
  }   
    

  // Kinematics
  kin_.reset(new Kin<JOINTS>(kdl_chain));

  // Cartesian gains
  double kp_trans, kd_trans, kp_rot, kd_rot;


 
#ifdef USE_J_TRANSPOSE_CONTROLLER
{
   
  for (int i = 0; i < JOINTS; ++i){
    Kd_j[i]=0.0;
    Kp_j[i]=0.0;
    Kff_j[i] = 1.0;
  }
  // very stable with inflated inertia on Wrist links
  kp_trans  = 800.0;
  kd_trans = 50.0;  
  kp_rot = 2;  
  kd_rot =2;
  
  // stable without inflated inertia tensor on Wrist links
  kp_trans  = 350.0;
  // kp_trans  = 800.0; // a bit of oscillation
  kd_trans = 50.0;  
  kp_rot = 1e-2;  
  kd_rot =1e-2;
  k_posture_ = 5e-3;
  k_posture_dot_ = 1e-2;
}
#endif

#ifdef USE_J_PINV_CONTROLLER
{
    
  for (int i = 0; i < JOINTS; ++i){
    Kd_j[i]=6;//5e-2
    Kp_j[i]=1e-4;
    Kff_j[i] = 1.0;
  }

  kp_trans  =8;
  kp_rot =1;  
  
  kd_trans = 2.0;  
  kd_rot =1e-2;


  k_posture_ = 5e-3;
  k_posture_dot_ = 1e-2;
  
  // Weighting matrix for Error and Jacobian (Weighted Least Squares)
  // as proposed by  T. Sugihara, "Solvability-Unconcerned Inverse Kinematics
  // by the Levenberg-Marquardt method", IEEE TRO, 2011.  
  W << 1,1,1,0.05,0.05,0.05; 
}
#endif
 
 
 Kp_x << kp_trans, kp_trans, kp_trans,  kp_rot, kp_rot, kp_rot;
 Kd_x << kd_trans, kd_trans, kd_trans,  kd_rot, kd_rot, kd_rot;

  // cart velocity limits (NOT USED!)
  vel_saturation_trans_=7.0;
  vel_saturation_rot_= 4.0;
  
  debug_ =false; 
  use_posture_ = true;
  uninitialized_ = true; // flag to indicate that the controller is in uninitialized_ state
  if(debug_){
    std::string id;
    id="q_lower_limit";
    JointVector temp =q_lower_limit* (180/M_PI); 
    print_joint_vector(id,temp);
    id="q_upper_limit";
    temp =q_upper_limit*(180/M_PI); 
    print_joint_vector(id,temp);
    id="q_velocity_limit";
    print_joint_vector(id,q_velocity_limit);
    id="q_effort_limit";
    print_joint_vector(id,q_effort_limit);
  }
  
  jacobian_inverse_damping_ = 1e-2;
  joint_vel_filter_ = 1.0;
  pose_command_filter_ = 1.0;

 // ---------------------- subscribe to goal commands 
_lcm->subscribe(input_cmds_channel, &pd_chain_control::ChainController<JOINTS>::handleEndEffectorCmdMsg, this); 
   
}

//================== destructor 
template <int JOINTS>
ChainController<JOINTS>::~ChainController() {}



//Controller needs initialization after the first robot state message is received.
template <int JOINTS>
void ChainController<JOINTS>::init() 
{
  std::cout << "Initializing chain controller" << std::endl;

  //Kp_x << 800.0, 800.0, 800.0,   80.0, 80.0, 80.0;
  //Kd_x << 12.0, 12.0, 12.0,   0.0, 0.0, 0.0;

  //Kd_j << 12.0, 12.0, 12.0,   12.0, 12.0, 12.0;
  //Kp_j << 800.0, 800.0, 800.0,  800.0, 800.0, 800.0;

    
  JointVector q = q_measured_;
  
  kin_->fk(q, x);


  q_posture_ = q;
  q_posture_ <<0,0,0,0,0,0; // rest position
  q_proxy_ = q;
  qdot_filtered_.setZero();
  xdot_desi_.setZero();

}

//update() function is called from the robot state msg handler defined in controller_manager.
template <int JOINTS>  
void ChainController<JOINTS>::update(const std::map<std::string, double> &jointpos_in, const std::map<std::string, double> &jointvel_in, const double &dt)
{

 // get the states for the subset of the joints defined in the chain.
 for(std::vector<std::string>::size_type i = 0; i != _chain_joint_names.size(); i++) 
 {
      //std::map<std::string, double>::iterator it;
      //it = jointpos_in.find(_chain_joint_names[i]);
      //q_measured[i] = it->second;
     q_measured_[i] = jointpos_in.find(_chain_joint_names[i])->second;
     qdot_measured_[i] = jointvel_in.find(_chain_joint_names[i])->second;
 }

 if(uninitialized_)
 {
   this->init();  // only run for the first instance. 
   uninitialized_ = false;
   x_desi_filtered_ = x_desi_;
 }

  // ======== Measures current arm state

  // // this is a member variable updated by robotstate message handler
// check timing of q, don't use q that is too old  
//  q_measured_<<0,0,0,0,0*(M_PI/180),0*(M_PI/180);
  JointVector q = q_measured_;

  kin_->fk(q, x);


  Jacobian J;
  kin_->jac(q, J); // update Jacobian for given q


  JointVector qdot_raw = qdot_measured_;

  for (int i = 0; i < JOINTS; ++i)
    qdot_filtered_[i] += joint_vel_filter_ * (qdot_raw[i] - qdot_filtered_[i]);
  JointVector qdot = qdot_filtered_;
  CartVector xdot = J * qdot;

  // ======== Controls to the current pose setpoint

	
  {
    Eigen::Vector3d p0(x_desi_filtered_.translation());
    Eigen::Vector3d p1(x_desi_.translation());
    Eigen::Quaterniond w0(x_desi_filtered_.linear()); 
    Eigen::Quaterniond w1(x_desi_.linear());
    w0.normalize();
    w1.normalize();
    Eigen::Vector3d p = p0 + pose_command_filter_ * (p1 - p0);
    Eigen::Quaterniond w = w0.slerp(pose_command_filter_, w1); // dont use q here
 
    //x_desi_filtered_ = q * Eigen::Translation3d(p);
    x_desi_filtered_ = Eigen::Translation3d(p) * w;
  }


  CartVector x_err;
  computePoseError(x, x_desi_filtered_, x_err);


// Computes pseudo-inverse of J via direct Inverse() (Better approach is to use SVD)
//   Eigen::Matrix<double,6,6> JJt = J * J.transpose();
//   Eigen::Matrix<double,6,6> JJt_inv;
//   JJt_inv = JJt.inverse();
//   Eigen::Matrix<double,JOINTS,6> J_pinv = J.transpose() * JJt_inv;
//   Eigen::Matrix<double,6,6> I6; I6.setIdentity();
//   Eigen::Matrix<double,6,6> JJt_damped = J * J.transpose() + jacobian_inverse_damping_ * I6;
//   Eigen::Matrix<double,6,6> JJt_inv_damped;
//   JJt_inv_damped = JJt_damped.inverse();
//   Eigen::Matrix<double,JOINTS,6> J_pinv_damped = J.transpose() * JJt_inv_damped;
//    // Computes the nullspace of J
//   Eigen::Matrix<double,JOINTS,JOINTS> I;
//   I.setIdentity();
//   Eigen::Matrix<double,JOINTS,JOINTS> N = I - J_pinv * J;
//   N = I - J_pinv_damped * J;
  
  
JointVector  tau;
tau.setZero();


#ifdef USE_J_TRANSPOSE_CONTROLLER
 {
   
   
  // Computes the reference twist from the pose error and the desired twist.
  CartVector xdot_ref = xdot_desi_.array() + (Kp_x.array() / Kd_x.array()) * x_err.array();

  // Applies velocity limits
  // Caps the cartesian velocity
 /* if (vel_saturation_trans_ > 0.0)
  {
    if (fabs(xdot_ref.head<3>().norm()) > vel_saturation_trans_)
      xdot_ref.head<3>() *= (vel_saturation_trans_ / xdot_ref.head<3>().norm());
  }
  if (vel_saturation_rot_ > 0.0)
  {
    if (fabs(xdot_ref.tail<3>().norm()) > vel_saturation_rot_)
      xdot_ref.tail<3>() *= (vel_saturation_rot_ / xdot_ref.tail<3>().norm());
  }//*/
  
  //DEBUGGING      
  if(debug_){

	    std::string id = "x_desi_";
  	    print_3D_position(id,x_desi_);
	    id = "x";
 	    print_3D_position(id,x);
	    id = "x_err";
	    print_cart_vector(id,x_err);
	    
	    id = "gains";
	    CartVector temp =  (Kp_x.array() / Kd_x.array());
	    print_cart_vector(id,temp);
	    id = "xdot_ref";
	    print_cart_vector(id,xdot_ref);
	    id = "xdot";
	    print_cart_vector(id,xdot);

      } 
     
  JointVector tau_posture;
  tau_posture.setZero();
  if (use_posture_)
  {
  
    JointVector posture_err = (q_posture_ - q); 

    //Normalization is required for continuous joints 
    for (size_t j = 0; j < JOINTS; ++j)
    {
      posture_err[j] = normalize_angle(posture_err[j]); 
    }
    
    if(debug_){
      std::string  id = "posture_err";
      JointVector temp =posture_err* (180/M_PI); 
      print_joint_vector(id,temp);
   }    
   
    // Psuedo Inverse Via SVD
    double Enorm = 0.5*x_err.transpose()*W.asDiagonal()*x_err;
    double lambda = Enorm + jacobian_inverse_damping_;
    VectorXq original_Aii,tmp;
    Eigen::JacobiSVD<Jacobian> svd;  
    svd.compute(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    original_Aii = svd.singularValues();   
    for (unsigned int j=0;j<original_Aii.rows();++j) {
	    original_Aii(j) = original_Aii(j)/( original_Aii(j)*original_Aii(j)+lambda);
    }
    // std::cout <<"pinv singular values" <<original_Aii.asDiagonal() << std::endl;
    Eigen::Matrix<double,JOINTS,6> J_pinv_damped;
    J_pinv_damped = svd.matrixV()*(original_Aii.asDiagonal()*svd.matrixU().transpose());
    

  // Computes the nullspace of J
    Eigen::Matrix<double,JOINTS,JOINTS> I;
    I.setIdentity();
    Eigen::Matrix<double,JOINTS,JOINTS> N = I - J_pinv_damped * J;
    N = I - J_pinv_damped * J;
    
    JointVector qdd_posture = k_posture_ * (posture_err) - k_posture_dot_ *(qdot);
    tau_posture = Kff_j.array() * (N * qdd_posture).array();
    
  } 
 
  CartVector F = Kd_x.array() * (xdot_ref - xdot).array(); // effectively a PD controller in cartesian space.
  JointVector tau_pose = J.transpose() * F;
  tau = tau_pose + tau_posture;
	    
  if(debug_){
    std::string id = "F";
    print_cart_vector(id,F);
    id = "tau_pose";
    print_joint_vector(id,tau_pose);
    id = "tau_posture";
    print_joint_vector(id,tau_posture);
 }
}
#endif 

#ifdef USE_J_PINV_CONTROLLER
 {
    // Implementation is based on the following paper (+ NULL SPACE biasing)
    // T. Sugihara, "Solvability-Unconcerned Inverse Kinematics
    // by the Levenberg-Marquardt method", IEEE Transactions on Robotics, 
    // Vol. 27, No. 5, 2011.  
   x_err = W.array()*x_err.array();
   
  // Computes the reference twist from the pose error and the desired twist.
    // PD controller in cartesian space.
  CartVector xdot_ref = (Kd_x.array())*(xdot_desi_-xdot).array() + (Kp_x.array()) * x_err.array();

  // Applies velocity limits
 // Caps the cartesian velocity
 // enforce_cart_vel_limits(xdot_ref);
   
  if(debug_)
  {
  std::string id = "x_desi_";
  print_3D_position(id,x_desi_);
  id = "x";
  print_3D_position(id,x);
  id = "x_err";
  print_cart_vector(id,x_err);	      
  id = "Kp_x";
  print_cart_vector(id,Kp_x);
  id = "Kd_x";
  print_cart_vector(id,Kd_x);
  id = "Kp_j";
  print_joint_vector(id,Kp_j);  
  id = "Kd_j";
  print_joint_vector(id,Kd_j);   
  id = "xdot_ref";
  print_cart_vector(id,xdot_ref);
  id = "xdot";
  print_cart_vector(id,xdot);
  } 
      

  // Computes the desired joint velocities for achieving the pose.
 JointVector qdot_ref;
  qdot_ref.setZero();
  //qdot_ref = J.transpose() * xdot_ref;  // J-transpose
  // qdot_ref = J_pinv_damped * (xdot_ref);  // J-inverse

   // Psuedo Inverse of Weighted Jacobian via SVD
   VectorXq original_Aii,tmp;
   Eigen::Matrix<double,6,JOINTS>  J_weighted;
    J_weighted = W.asDiagonal()*J;  
    Eigen::JacobiSVD<Jacobian> svd;  
    svd.compute(J_weighted, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Reconstruction
    // J_weighted  = svd.matrixU()*(svd.singularValues().asDiagonal()*svd.matrixV().transpose());
    original_Aii = svd.singularValues();   
    
    double Enorm = 0.5*x_err.transpose()*W.asDiagonal()*x_err;
    double lambda = Enorm + jacobian_inverse_damping_;
    // Adaptive error based damping proposed by
    // T. Sugihara, "Solvability-Unconcerned Inverse Kinematics
    // by the Levenberg-Marquardt method", IEEE Transactions on Robotics, 
    // Vol. 27, No. 5, 2011.  
    for (unsigned int j=0;j<original_Aii.rows();++j) {
	    original_Aii(j) = original_Aii(j)/( original_Aii(j)*original_Aii(j)+lambda);
    }

   // J_pinv_damped = svd.matrixV()*(original_Aii.asDiagonal()*svd.matrixU().transpose());
   // qdot_ref = J_pinv_damped * xdot_ref;  
   // The following is equivalent to qdot_ref = J_pinv_damped * (xdot_ref); 
   tmp = svd.matrixU().transpose()*(xdot_ref);
   tmp = original_Aii.cwiseProduct(tmp);
   qdot_ref = svd.matrixV()*tmp;
  //enforce_joint_vel_limits(qdot_ref);        

    
  if (use_posture_)
  {
    JointVector posture_err = q_posture_ - q;
    
    //Normalization is required for continuous joints 
    for (size_t j = 0; j < JOINTS; ++j)
    {
      posture_err[j] = normalize_angle(posture_err[j]); 
    }

    JointVector qd_posture = k_posture_ * posture_err - k_posture_dot_ *(qdot);

    if(debug_){
    std::string  id = "posture_err";
    JointVector temp =posture_err* (180/M_PI); 
    print_joint_vector(id,temp);
    }  
    
    // Psuedo Inverse of J Via SVD
    double Enorm = 0.5*x_err.transpose()*W.asDiagonal()*x_err;
    double lambda = Enorm + jacobian_inverse_damping_;
    VectorXq original_Aii,tmp;
    Eigen::JacobiSVD<Jacobian> svd; 
    svd.compute(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    original_Aii = svd.singularValues();   
    for (unsigned int j=0;j<original_Aii.rows();++j) {
	    original_Aii(j) = original_Aii(j)/( original_Aii(j)*original_Aii(j)+lambda);
    }
    // std::cout <<"pinv singular values" <<original_Aii.asDiagonal() << std::endl;
    Eigen::Matrix<double,JOINTS,6> J_pinv_damped;
    J_pinv_damped = svd.matrixV()*(original_Aii.asDiagonal()*svd.matrixU().transpose());
    

  // Computes the nullspace of J
    Eigen::Matrix<double,JOINTS,JOINTS> I;
    I.setIdentity();
    Eigen::Matrix<double,JOINTS,JOINTS> N = I - J_pinv_damped * J;

    qdot_ref += N * qd_posture;

 }
 if(debug_){
    std::string  id = "qdot_ref";
    print_joint_vector(id,qdot_ref);
    id = "qdot";
    print_joint_vector(id,qdot);
 }
  

 //Clamps the proxy
 //to prevent integral windup
  for (int j = 0; j < JOINTS; ++j){
    double p_clamp = 0.087;
    q_proxy_[j] = std::max(q[j] - p_clamp, std::min(q_proxy_[j], q[j] + p_clamp));    
  }
   enforce_joint_limits(q_proxy_);

  // Convertes the desired joint velocities into joint torques
  // Eq 14: in nakanishi et al, "Comparative Experiments on Task Space Control
  // with Redundancy Resolution"
  // PD control in joint, and P in cartesian space.
  //tau = Kp_j.array() * (q_proxy_ - q).array() +  Kd_j.array()*(qdot_ref - qdot).array();
  
  // A lot more stable when the derivative part is done in cartesian space, since trans
  // and rot error can be separated.
  tau =  Kp_j.array() * (q_proxy_ - q).array() + Kd_j.array()*(qdot_ref).array();
  q_proxy_ += qdot_ref * dt;

}
#endif 


  
  enforce_joint_effort_limits(tau);
  // LShoulderYaw LShoulderPitch LShoulderRoll  LElbowPitch LWristYaw LWristRoll
  //tau<< 0,-0,50,-30,0,0;
    drc::actuator_cmd_t torque_cmd;
    torque_cmd.timestamp = getTime_now();
    torque_cmd.robot_name = _robot_name;
    torque_cmd.num_joints = _chain_joint_names.size();
    for(int i = 0; i < torque_cmd.num_joints; i++){
	torque_cmd.joint_name.push_back(_chain_joint_names[i]);
        torque_cmd.joint_effort.push_back(tau[i]);
	torque_cmd.duration.push_back(10*dt);// expires after 0.1 sec
    }
 
    _lcm->publish("ACTUATOR_CMDS", &torque_cmd); // unstable;
    

    if(debug_){
          std::string id = "Torque";
          print_joint_vector(id,tau);
	  id = "q";
	  JointVector q_deg = q_measured_*(180/M_PI);
	  print_joint_vector(id,q_deg);
	  std::cout <<"\n" << std::endl;
    } 

}//end update



template <int JOINTS>  
void ChainController<JOINTS>::computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err)
{
  
// PR2's error metric
/* err.head<3>() = xdes.translation() - xact.translation();
// cross product is ill-posed at 0 and 180? 
  err.tail<3>()   = 0.5* (xact.linear().col(0).cross(xdes.linear().col(0)) +
                          xact.linear().col(1).cross(xdes.linear().col(1)) +
                          xact.linear().col(2).cross(xdes.linear().col(2)));
*/

// Better error metric from KDL.	    
 KDL::Frame kdl_xdes,kdl_xact;
  transformEigenToKDL(xdes,kdl_xdes);
  transformEigenToKDL(xact,kdl_xact);

  KDL::Twist delta_pos;
  // delta pos required to transform xact to xdes
  delta_pos =KDL::diff(kdl_xact,kdl_xdes,1);
  transformKDLtwistToEigen(delta_pos,err);

// see KDL/src/frames.hpp  
//  
//  * KDL::diff() determines the rotation axis necessary to rotate the frame b1 to the same orientation as frame b2 and the vector
//  * necessary to translate the origin of b1 to the origin of b2, and stores the result in a Twist datastructure.   
// IMETHOD Twist diff(const Frame& F_a_b1,const Frame& F_a_b2,double dt=1);
//   IMETHOD Vector diff(const Rotation& R_a_b1,const Rotation& R_a_b2,double dt) {
// 	Rotation R_b1_b2(R_a_b1.Inverse()*R_a_b2);
// 	return R_a_b1 * R_b1_b2.GetRot() / dt;
// }


/* 
 if(debug_){
  KDL::Vector v(delta_pos[3],delta_pos[4],delta_pos[5]);
  double angle = v.Norm();
  KDL::Rotation M;
  M = KDL::Rotation::Rot(v,angle);
 double roll, pitch, yaw; 
 M = M.Inverse();
 M.GetRPY( roll,pitch, yaw); 
 std::cout<< "RPY: "<< roll *(180/M_PI)<< " "<<  pitch*(180/M_PI) << " "<<  yaw*(180/M_PI) << " " << std::endl;
}*/

} // end computePoseError



  //=============message callbacks


template <int JOINTS>  
void ChainController<JOINTS>::handleEndEffectorCmdMsg(const lcm::ReceiveBuffer* rbuf,
						 const std::string& chan, 
						 const drc::ee_goal_t* msg)						 
  { 
    
    
    if(debug_)
    {
     std::cout<< controller_state << std::endl;
     std::cout << "goal command for end effector received : "<< msg->ee_name  <<" received."<< std::endl;
     std::cout << msg->ee_goal_pos.rotation.x 
        << "," << msg->ee_goal_pos.rotation.y
	<< "," << msg->ee_goal_pos.rotation.z
	<< "," << msg->ee_goal_pos.rotation.w << std::endl;
	Eigen::Affine3d test;
	transformLCMToEigen(msg->ee_goal_pos,test);
	Eigen::Quaterniond q = (Eigen::Quaterniond)test.linear();
	q.normalize();
	std::cout << q.x() 
	   << "," << q.y()
	   << "," << q.z()
	   << "," << q.w() << std::endl;
    }
 /*    drc::ee_goal_t
 ----------------------------
	int64_t timestamp;
	string robot_name;
	string ee_name;
	string root_name;
	position3D_t ee_goal_pos;
	twist_t ee_goal_twist;
	int num_chain_joints;
	bool use_posture_bias;
	double joint_posture_bias [num_chain_joints];
	string chain_joint_names [num_chain_joints];
	boolean stop_ee_controller; // is set when the user no longer want's chain controller
	//e.g. published with this flag set in the destructor of a ee goal object in user interface
*/
 
 if((controller_state==RUNNING)&(msg->halt_ee_controller)){
    controller_state=STOPPED;
    uninitialized_ = true;
 }
 else if((controller_state==STOPPED)&(msg->halt_ee_controller==false))
   controller_state=RUNNING; // as we just received a goal.
 

  if(controller_state==RUNNING)
  { 
    
        latest_goal_timestamp_ = msg->timestamp;

	transformLCMToEigen(msg->ee_goal_pos,x_desi_);
	transformLCMtwistToEigen(msg->ee_goal_twist, xdot_desi_);
    
	//TODO: clamp x_desi_ if it is too far away?
	
	int num_chain_joints = msg->num_chain_joints;

	if(msg->use_posture_bias) // user specified posture bias, otherwise rest position is used for null space bias
	{
	  std::map<std::string, double> joint_posture_map;
	   for (uint i=0; i< (uint) msg->num_chain_joints; i++)
	   { //cast to uint to suppress compiler warning
	      joint_posture_map.insert(make_pair(msg->chain_joint_names[i], msg->joint_posture_bias[i]));
	   }  
	    
	  for(std::vector<std::string>::size_type i = 0; i != _chain_joint_names.size(); i++) 
	   {
	     /*std::vector<double>::iterator it;
	       it = std::find(msg->chain_joint_names.begin(),msg->chain_joint_names.end(),_chain_joint_names[i]);*/
	       q_posture_[i] = joint_posture_map.find(_chain_joint_names[i])->second;  
	   }

	} // end if (msg->use_posture_bias)
	else
	{
		q_posture_ = q_measured_;
	}

  }//if(controller_state==RUNNING)

	
 } // end handleEndEffectorCmdMsg
 
template <int JOINTS>  
bool ChainController<JOINTS>::isRunning()
{
  return (controller_state==RUNNING);

}

template <int JOINTS>  
void ChainController<JOINTS>::enforce_joint_limits(JointVector &q)
{
  for (int j = 0; j < JOINTS; ++j){
    if(q_lower_limit[j]-q_upper_limit[j]!=0)
    q[j] = std::min(std::max(q[j],q_lower_limit[j]),q_upper_limit[j]);
    else
      q[j] =q[j];
  }
}

template <int JOINTS>  
void ChainController<JOINTS>::enforce_joint_vel_limits(JointVector &qdot)
{
  for (int j = 0; j < JOINTS; ++j){
    //q_velocity_limit[j] =  20;
     if(q_velocity_limit[j]!=0)
      qdot[j] = std::min(std::max(qdot[j],-1.0*q_velocity_limit[j]),q_velocity_limit[j]);
     else
       qdot[j] =qdot[j];
  }
}

template <int JOINTS>  
void ChainController<JOINTS>::enforce_cart_vel_limits(CartVector &xdot)
{
  // Applies velocity limits
  // Caps the cartesian velocity
  if (vel_saturation_trans_ > 0.0)
  {
    if (fabs(xdot.head<3>().norm()) > vel_saturation_trans_)
      xdot.head<3>() *= (vel_saturation_trans_ / xdot.head<3>().norm());
  }
  if (vel_saturation_rot_ > 0.0)
  {
    if (fabs(xdot.tail<3>().norm()) > vel_saturation_rot_)
      xdot.tail<3>() *= (vel_saturation_rot_ / xdot.tail<3>().norm());
  }
}

template <int JOINTS>  
void ChainController<JOINTS>::enforce_joint_effort_limits(JointVector &tau)
{
    // ======== Torque Saturation
  for (int j = 0; j < JOINTS; ++j){
     if(q_effort_limit[j]!=0)
      tau[j] = std::min(std::max(tau[j],-q_effort_limit[j]),q_effort_limit[j]);
     else
       tau[j] =tau[j];
  }
  
}

// Some printing functions useful for debugging.
template <int JOINTS>  
void ChainController<JOINTS>::print_joint_vector(std::string &id,JointVector &v)
{
  std::cout<<id<<": ";
  std::cout << v.transpose()  << std::endl;
}

template <int JOINTS>  
void ChainController<JOINTS>::print_cart_vector(std::string &id,CartVector &v)
{
  std::cout<<id<<": ";
  std::cout << v.transpose() << std::endl;
}


template <int JOINTS>  
void ChainController<JOINTS>::print_3D_position(std::string &id,Eigen::Affine3d &x)
{
  std::cout<<id<<": ";
  drc::position3D_t m;
  transformEigenToLCM(x,m);
  std::cout<<  m.translation.x << " "<<  m.translation.y << " "<<  m.translation.z << " "
  <<  m.rotation.x << " "<<  m.rotation.y << " "<<  m.rotation.z << " "<<  m.rotation.w << std::endl;
  double roll, pitch, yaw;
  KDL::Frame k;
  transformEigenToKDL(x,k);
  k.M.GetRPY( roll,pitch, yaw);
  std::cout<< "RPY: "<< roll *(180/M_PI)<< " "<<  pitch*(180/M_PI) << " "<<  yaw*(180/M_PI) << " " << std::endl;

}

  
} //namespace pd_chain_control


#endif //CHAIN_CONTROLLER_HPP

  /// 
  /// IMPORTANT:  The safety controller support is very much PR2 specific, not intended for generic usage.
  /// 
  /// Basic safety controller operation is as follows
  /// 
  /// current safety controllers will take effect on joints outside the position range below:
  ///
  /// position range: [JointSafety::soft_lower_limit  + JointLimits::velocity / JointSafety::k_position, 
  ///                  JointSafety::soft_uppper_limit - JointLimits::velocity / JointSafety::k_position]
  ///
  /// if (joint_position is outside of the position range above)
  ///     velocity_limit_min = -JointLimits::velocity + JointSafety::k_position * (joint_position - JointSafety::soft_lower_limit)
  ///     velocity_limit_max =  JointLimits::velocity + JointSafety::k_position * (joint_position - JointSafety::soft_upper_limit)
  /// else
  ///     velocity_limit_min = -JointLimits::velocity
  ///     velocity_limit_max =  JointLimits::velocity
  ///
  /// velocity range: [velocity_limit_min + JointLimits::effort / JointSafety::k_velocity,
  ///                  velocity_limit_max - JointLimits::effort / JointSafety::k_velocity]
  ///
  /// if (joint_velocity is outside of the velocity range above)
  ///     effort_limit_min = -JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_min)
  ///     effort_limit_max =  JointLimits::effort + JointSafety::k_velocity * (joint_velocity - velocity_limit_max)
  /// else
  ///     effort_limit_min = -JointLimits::effort
  ///     effort_limit_max =  JointLimits::effort
  ///
  /// Final effort command sent to the joint is saturated by [effort_limit_min,effort_limit_max]
  ///
  /// Please see wiki for more details: http://www.ros.org/wiki/pr2_controller_manager/safety_limits
  
  
      
/// Eigen::MatrixXd m = J;//Eigen::MatrixXd::Random(3,2);
/// Eigen::JacobiSVD<MatrixXd> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
/// Eigen::VectorXd rhs=xdot_ref;//(1, 0, 0);
/// std::cout << "A least-squares solution of m*x = rhs is:" <<  std::endl << svd.solve(rhs) <<  std::endl;
/// J_svd  = svd2.matrixU()*(svd2.singularValues().asDiagonal()*svd2.matrixV().transpose());
/// std::cout << "J_svd\n" << J_svd << std::endl;