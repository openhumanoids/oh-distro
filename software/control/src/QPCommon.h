
#include "ControlUtil.h"

#include "drake/fastQP.h"
#include "drake/gurobiQP.h"

//#define TEST_FAST_QP

const double REG = 1e-8;

struct QPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  void* multi_robot; // optional multi rigid body system
  double w; // objective function weight
  double slack_limit; // maximum absolute magnitude of acceleration slack variable values
  VectorXd umin,umax;
  void* map_ptr;
  double Kp_com,Kd_com,Kp_k; // COM-z and angular momentum (k) PD gains 
  std::set<int> active;

  // preallocate memory
  MatrixXd H, H_float, H_act;
  VectorXd C, C_float, C_act;
  MatrixXd B, B_act;
  MatrixXd J, Jdot;
  MatrixXd J_xy, Jdot_xy;
  MatrixXd Hqp;
  RowVectorXd fqp;
  
  // momentum controller-specific
  MatrixXd Ag, Agdot; // centroidal momentum matrix
  MatrixXd W_hdot; // quadratic cost for momentum control: (hdot_des - hdot)'*W*(hdot_des - hdot)
  VectorXd w_qdd; 
  double w_grf; 
  double w_slack; 
  double Kp, Kd; // COM-z PD gains, for momentum controller
  double mass; // total robot mass
  bool smooth_contacts;
  std::set<int> previous_contact_bodies; // list of body indices
  int n_body_accel_inputs;
  int n_body_accel_constraints;
  VectorXd body_accel_input_weights;

  // gurobi active set params
  int *vbasis;
  int *cbasis;
  int vbasis_len;
  int cbasis_len;
};
