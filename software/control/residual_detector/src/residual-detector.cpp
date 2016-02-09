#include "residual-detector.hpp"
#include <model-client/model-client.hpp>

#define RESIDUAL_GAIN 100.0;
#define PUBLISH_CHANNEL "RESIDUAL_OBSERVER_STATE"
using namespace Eigen;


ResidualDetector::ResidualDetector(std::shared_ptr<lcm::LCM> &lcm_, bool verbose_, std::string urdfFilename):
    lcm_(lcm_), verbose_(verbose_), newStateAvailable(false), newResidualStateAvailable(false),
    useFootForceFlag(false), useFootFTFlag(false), useGeometricJacobianFlag(false){



  if (urdfFilename=="none"){
    std::string drcBase = std::string(std::getenv("DRC_BASE"));
    urdfFilename = drcBase + "/software/models/atlas_v5/model_LR_RR.urdf";
    drake_model.addRobotFromURDF(urdfFilename);
    this->contactFilter.addRobotFromURDF(urdfFilename);
  }
  else if (urdfFilename.empty()) {
    do {
      botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
    } while(botparam_ == NULL);
    std::shared_ptr<ModelClient> model = std::shared_ptr<ModelClient>(new ModelClient(lcm_->getUnderlyingLCM(),0));
    drake_model.addRobotFromURDFString(model->getURDFString());
    this->contactFilter.addRobotFromURDFString(model->getURDFString());
  }
  else{
    drake_model.addRobotFromURDF(urdfFilename);
    this->contactFilter.addRobotFromURDF(urdfFilename);
  }

  // if you want to use quaternions. Normally it defaults to ROLLPITCHYAW, need to override by passing optional args
  // see RigidBodyManipulatorURDF.cpp file for the syntax
  // drake_model.addRobotFromURDFString(model->getURDFString(), ".", DrakeJoint::QUATERNION)
  drake_model.compile();

  lcm_->subscribe("EST_ROBOT_STATE", &ResidualDetector::onRobotState, this);
  lcm_->subscribe("FOOT_CONTACT_ESTIMATE", &ResidualDetector::onFootContact, this);
  lcm_->subscribe("FOOT_FORCE_TORQUE", &ResidualDetector::onFootForceTorque, this);
  lcm_->subscribe("EXTERNAL_FORCE_TORQUE", &ResidualDetector::onExternalForceTorque, this);




  //need to initialize the state_driver
//  std::cout << "this should only be printed once" << std::endl;
//  std::vector<std::string> state_coordinate_nadrakemes;

  // build this up manually for now, hacky solution
  this->nq = drake_model.num_positions;
  this->nv = drake_model.num_velocities;

  std::cout << "robot num velocities  = " + this->nv << std::endl;

  for(int i = 0; i < this->nq; i++){
    std::string joint_name = drake_model.getStateName(i);
    this->state_coordinate_names.push_back(joint_name);
  }

  state_driver.reset(new RobotStateDriver(this->state_coordinate_names));


  // initialize foot_contact_driver
  BodyIdsCache body_ids_cache;
  body_ids_cache.r_foot = drake_model.findLinkId("r_foot");
  body_ids_cache.l_foot = drake_model.findLinkId("l_foot");
  body_ids_cache.pelvis = drake_model.findLinkId("pelvis");

  this->foot_body_ids[Side::LEFT] = body_ids_cache.l_foot;
  this->foot_body_ids[Side::RIGHT] = body_ids_cache.r_foot;

  this->args.b_contact_force[Side::LEFT] = false;
  this->args.b_contact_force[Side::RIGHT] = false;

  foot_contact_driver.reset(new FootContactDriver(body_ids_cache));

  // initialize the residual_state
  residual_state.t_prev = 0;
  residual_state.r = VectorXd::Zero(nq);
  residual_state.integral = VectorXd::Zero(nq);
  residual_state.gamma = VectorXd::Zero(nq);
  residual_state.p_0 = VectorXd::Zero(nq);
  residual_state.running = false;

  //DEBUGGING
  residual_state.gravity = VectorXd::Zero(nq);
  residual_state.torque = VectorXd::Zero(nq);
  residual_state.foot_contact_joint_torque = VectorXd::Zero(nq);


  this->residualGain = RESIDUAL_GAIN;
  this->publishChannel = PUBLISH_CHANNEL;

  this->residualGainVector = residualGain*VectorXd::Ones(this->nq);

  this->foot_FT_6_axis_available = false;


  //TEST
  t_prev = 0;
  return;

}

void ResidualDetector::useFootForce(bool useGeometricJacobian) {
  this->publishChannel = this->publishChannel + "_W_FOOT_FORCE";
  this->useFootForceFlag = true;
  this->useFootFTFlag = false;

  if (useGeometricJacobian){
    std::cout << "switching publish channel to use geometric jacobian" << std::endl;
    this->useGeometricJacobianFlag = true;
    this->publishChannel = PUBLISH_CHANNEL;
    this->publishChannel = this->publishChannel + "_W_FOOT_FORCE_GEOMETRIC_JACOBIAN";
  }
}

void ResidualDetector::useFootForceTorque() {
  this->publishChannel = this->publishChannel + "_W_FOOT_FT";
  this->useFootForceFlag = false;
  this->useFootFTFlag = true;
}

void ResidualDetector::onRobotState(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                    const drc::robot_state_t *msg) {

//  if (this->verbose_) {
//    std::cout << "got a robot state message" << std::endl;
//  }

  std::shared_ptr<DrakeRobotStateWithTorque> state(new DrakeRobotStateWithTorque);
  int nq = this->drake_model.num_positions;
  int nv = this->drake_model.num_velocities;
  state->q = VectorXd::Zero(nq);
  state->qd = VectorXd::Zero(nv);
  state->torque = VectorXd::Zero(nq);

  this->state_driver->decodeWithTorque(msg, state.get());

  // acquire a lock and write to the shared_ptr for robot_state_
  std::unique_lock<std::mutex> lck(this->pointerMutex);

  this->args.robot_state = state;

  const drc::force_torque_t& force_torque = msg->force_torque;

  this->args.foot_force_torque_measurement[Side::LEFT].frame_idx = this->foot_body_ids[Side::LEFT];
  this->args.foot_force_torque_measurement[Side::LEFT].wrench << force_torque.l_foot_torque_x, force_torque.l_foot_torque_y, 0.0, 0.0, 0.0, force_torque.l_foot_force_z;

  this->args.foot_force_torque_measurement[Side::RIGHT].frame_idx = this->foot_body_ids[Side::RIGHT];
  this->args.foot_force_torque_measurement[Side::RIGHT].wrench << force_torque.r_foot_torque_x, force_torque.r_foot_torque_y, 0.0, 0.0, 0.0, force_torque.r_foot_force_z;


  lck.unlock();
  this->newStateAvailable = true;


}

void ResidualDetector::onFootContact(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                     const drc::foot_contact_estimate_t *msg) {

  std::unique_lock<std::mutex> lck(pointerMutex);
  this->args.b_contact_force[Side::LEFT] = msg->left_contact > 0.5;
  this->args.b_contact_force[Side::RIGHT] = msg->right_contact > 0.5;
  lck.unlock();
}

void ResidualDetector::onFootForceTorque(const lcm::ReceiveBuffer *rbuf, const std::string &channel,
                                         const drc::foot_force_torque_t *msg) {

  std::unique_lock<std::mutex> lck(pointerMutex);
  this->foot_FT_6_axis_available = true;
  this->args.foot_ft_meas_6_axis[Side::LEFT].wrench << msg->l_foot_torque[0], msg->l_foot_torque[1], msg->l_foot_torque[2], msg->l_foot_force[0], msg->l_foot_force[1], msg->l_foot_force[2];
  this->args.foot_ft_meas_6_axis[Side::RIGHT].wrench << msg->r_foot_torque[0], msg->r_foot_torque[1], msg->r_foot_torque[2], msg->r_foot_force[0], msg->r_foot_force[1], msg->r_foot_force[2];
  lck.unlock();
}

void ResidualDetector::onExternalForceTorque(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                           const drake::lcmt_external_force_torque* msg){
  std::unique_lock<std::mutex> lck(pointerMutex);
  this->linksWithExternalForce.clear();
  for (int i=0; i<msg->num_external_forces; i++){
    this->linksWithExternalForce.push_back(msg->body_names[i]);
  }
  lck.unlock();
}


void ResidualDetector::updateResidualState() {

  // acquire a lock and copy data to local variables
  // need to really understand what data is being copied here, and what is a reference
  std::unique_lock<std::mutex> lck(this->pointerMutex);
  // we create a new DrakeRobotState every time we get a state message so this should be ok
  std::shared_ptr<DrakeRobotStateWithTorque> robot_state = this->args.robot_state;

  std::map<Side, ForceTorqueMeasurement> foot_force_torque_measurement;
  // if we have a 6-axis FT measurement then use it
  if (this->foot_FT_6_axis_available){
    foot_force_torque_measurement = this->args.foot_ft_meas_6_axis;
  }
  else{
    foot_force_torque_measurement  = this->args.foot_force_torque_measurement;
  }
  //NOTE: the above are deep copies, since none of the things being copied are/contain pointers (as far as I know).
  const auto b_contact_force = this->args.b_contact_force;
  this->newStateAvailable = false;
  lck.unlock();


  double t = robot_state->t;
  double dt = t - this->residual_state.t_prev;

  // compute H for current state (q,qd)
  this->drake_model.doKinematics(robot_state->q, robot_state->qd, true);

  // could replace this GradVar declaration with auto?
  GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> gradVar = this->drake_model.massMatrix<double>(1);
  const auto & H = gradVar.value();

  MatrixXd H_grad = gradVar.gradient().value(); // size nq^2 x nq




  MatrixXd footJacobian; // should be 3 x nv
  Vector3d points(0,0,0); //point ifs at origin of foot frame

  //Torques due to foot contact forces: Set to zero by default
  std::map<Side, VectorXd> foot_ft_to_joint_torques;
  foot_ft_to_joint_torques[Side::LEFT] = VectorXd::Zero(this->nv);
  foot_ft_to_joint_torques[Side::RIGHT] = VectorXd::Zero(this->nv);

  for (const auto& side_force: foot_force_torque_measurement){
    const Side& side = side_force.first;
    int body_id = this->foot_body_ids[side];

    if (this->verbose_){
      if (b_contact_force.at(side)){
        std::cout << std::cout << side.toString() << " foot in contact" << std::endl;
      }
      else{
        std::cout << std::cout << side.toString() << " foot NOT in contact" << std::endl;
      }
    }
    // check if this body is actually in contact or not
    // if not set the contact torques coming from the feet to zero.
    // ignore this for
//    if (!b_contact_force.at(side)){
//      foot_ft_to_joint_torques[side] = VectorXd::Zero(this->nq);
//      continue;
//    }

    // Compute the joint torques resulting from the foot contact
    if (this->useFootFTFlag){
//      std::cout << "using Foot FT" << std::endl;
      std::vector<int> v_indices;
      footJacobian = this->drake_model.geometricJacobian<double>(0, body_id, body_id, 0, true, &v_indices).value();
      const Vector6d &wrench = side_force.second.wrench;
      VectorXd joint_torque_at_v_indices = footJacobian.transpose() * wrench;
      VectorXd joint_torque = VectorXd::Zero(this->nq);

      // convert them to a full sized joint_torque vector
      for (int i = 0; i < v_indices.size(); i++){
        joint_torque(v_indices[i]) = joint_torque_at_v_indices(i);
      }

      foot_ft_to_joint_torques[side_force.first] = joint_torque;
    }
    else if (this->useFootForceFlag) {
      std::cout << "using Foot Forces" << std::endl;
      footJacobian = this->drake_model.forwardKinJacobian(points, body_id, 0, 0, true, 0).value();
      const Vector3d &force = side_force.second.wrench.block(3, 0, 3, 1); // extract the force measurement

      foot_ft_to_joint_torques[side_force.first] = footJacobian.transpose() * force;

      if (this->useGeometricJacobianFlag){
        std::cout << "using geometric Jacobian" << std::endl;
        std::vector<int> v_indices;
        footJacobian = this->drake_model.geometricJacobian<double>(0, body_id, body_id, 0, true, &v_indices).value();
        Vector6d wrench = side_force.second.wrench;
        wrench.head<3>().setZero(); // set the wrenches to zero
        VectorXd joint_torque_at_v_indices = footJacobian.transpose() * wrench;
        VectorXd joint_torque = VectorXd::Zero(this->nq);

        // convert them to a full sized joint_torque vector
        for (int i = 0; i < v_indices.size(); i++){
          joint_torque(v_indices[i]) = joint_torque_at_v_indices(i);
        }

        foot_ft_to_joint_torques[side_force.first] = joint_torque;
      }

    }
    else{
      std::cout << "told not to use foot forces, setting foot_ft_to_joint_torques to zero" << std::endl;
      foot_ft_to_joint_torques[Side::LEFT] = VectorXd::Zero(this->nq);
      foot_ft_to_joint_torques[Side::RIGHT] = VectorXd::Zero(this->nq);
    }

    if (this->verbose_){
//      std::cout << side.toString() << " foot force is " << std::endl;
//      std::cout << force << std::endl;

      std::cout << "z force passed through Jacobian is " << foot_ft_to_joint_torques[side](2) << std::endl;
    }
  }

  //compute the gravitational term in manipulator equations, hack by calling doKinematics with zero velocity
  VectorXd qd_zero = VectorXd::Zero(this->nq);
  this->drake_model.doKinematics(robot_state->q, qd_zero, true);
  // dummy of zero external forces
  std::map<int, std::unique_ptr< GradientVar<double, TWIST_SIZE, 1> > > f_ext; //hopefully TWIST_SIZE has been defined in a header i've imported
  VectorXd gravity = this->drake_model.inverseDynamics(f_ext).value();

  // computes a vector whose i^th entry is g_i(q) - 1/2*qd^T*dH/dq_i*qd
  VectorXd alpha = VectorXd::Zero(this->nq);
  for (int i=0; i<this->nq; i++){
    int row_idx = i*this->nq;
    alpha(i) = gravity(i) - 1/2*robot_state->qd.transpose()*H_grad.block(row_idx,0,this->nq, this->nq)*robot_state->qd;
  }


  // this is the generalized momentum
  VectorXd p_momentum = H*robot_state->qd;

  //Special case for the first time we enter the loop
  //if this is the first time we are entering the loop, then set p_0, and return. Residual stays at 0
  if (!this->residual_state.running){
    std::cout << "started residual detector updates" << std::endl;
    this->residual_state.running = true;
    this->residual_state.p_0 = p_momentum;
    return;
  }



  // UPDATE STEP
  // First compute new state for residual, only new information this uses is dt, and p_momentum
  VectorXd integral_new = this->residual_state.integral + dt*this->residual_state.gamma;
  VectorXd r_new = this->residualGain*(p_momentum - this->residual_state.p_0 + integral_new);


  // update the residual state
  this->residual_state.r = r_new;
  this->residual_state.integral = integral_new;
  this->residual_state.gamma = alpha - robot_state->torque - this->residual_state.r -
                               foot_ft_to_joint_torques[Side::LEFT] - foot_ft_to_joint_torques[Side::RIGHT]; // this is the integrand;
  this->residual_state.t_prev = t;
  this->newResidualStateAvailable = true;
  this->newResidualStateAvailableForActiveLink = true;

  //DEBUGGING
  this->residual_state.gravity = alpha;
  this->residual_state.torque = robot_state->torque;
  this->residual_state.foot_contact_joint_torque = foot_ft_to_joint_torques[Side::LEFT] + foot_ft_to_joint_torques[Side::RIGHT];



  if (this->verbose_){
    double Hz = 1/dt;
    std::cout << "residual update running at " << Hz << std::endl;
  }

  //publish over LCM
  this->publishResidualState(this->publishChannel, this->residual_state);
}

void ResidualDetector::publishResidualState(std::string publishChannel, const ResidualDetectorState &residual_state){


  std::unique_lock<std::mutex> lck(pointerMutex);
  this->residual_state_msg.utime = static_cast<int64_t> (this->residual_state.t_prev * 1e6);
  lck.unlock();

  this->residual_state_msg.num_joints = (int16_t) this->nq;
  this->residual_state_msg.joint_name = this->state_coordinate_names;
  this->residual_state_msg.residual.resize(this->nq);
  this->residual_state_msg.gravity.resize(this->nq);
  this->residual_state_msg.internal_torque.resize(this->nq);
  this->residual_state_msg.foot_contact_torque.resize(this->nq);

  for (int i=0; i < this->nq; i++){
    this->residual_state_msg.residual[i] = (float) residual_state.r(i);

    //DEBUGGING
    this->residual_state_msg.gravity[i] = (float) residual_state.gravity(i);
    this->residual_state_msg.internal_torque[i] = (float) residual_state.torque(i);
    this->residual_state_msg.foot_contact_torque[i] = (float) residual_state.foot_contact_joint_torque(i);
  }

  if (this->verbose_){
    std::cout << "base_z residual is " << residual_state.r(2) << std::endl;
    std::cout << "base_z residual in message is " << this->residual_state_msg.residual[2] << std::endl;
  }

  this->lcm_->publish(publishChannel, &residual_state_msg);
}

void ResidualDetector::computeContactFilter(bool publishMostLikely, bool publishAll){
  // copy the required data to local variables
  std::unique_lock<std::mutex> lck(pointerMutex);
  VectorXd residual = this->residual_state.r;
  double t = this->args.robot_state->t;
  VectorXd q = this->args.robot_state->q;
  VectorXd v = this->args.robot_state->qd;
  lck.unlock();

  this->contactFilter.computeLikelihoodFull(t, residual, q, v, publishMostLikely, publishAll);
}

void ResidualDetector::computeActiveLinkContactFilter(bool publish, bool useActiveLinkInfo){
  std::unique_lock<std::mutex> lck(pointerMutex);
  VectorXd residual = this->residual_state.r;
  double t = this->args.robot_state->t;
  VectorXd q = this->args.robot_state->q;
  VectorXd v = this->args.robot_state->qd;
  std::vector<std::string> activeLinkNames = this->linksWithExternalForce;
  lck.unlock();

  if (!useActiveLinkInfo){
    activeLinkNames.clear();
  }

  this->contactFilter.computeActiveLinkForceTorque(t, residual, q, v, publish, activeLinkNames);
}

void ResidualDetector::residualThreadLoop() {
  bool done = false;

  while (!done){
    while(!this->newStateAvailable){
      std::this_thread::yield();
    }
    this->updateResidualState();
  }
}

void ResidualDetector::contactFilterThreadLoop(std::string filename) {
  std::cout << "entered contact filter loop" << std::endl;
//  Vector3d contactPosition(0,0,0);
//  Vector3d contactNormal(0,0,1);
//  std::string filename = "testCFP.csv";
  std::vector<ContactFilterPoint> cfpVec = constructContactFilterPointsFromFile(filename);
  std::cout << "constructed cfpVec" << std::endl;
  this->contactFilter.addContactPoints(cfpVec);

  std::cout << "added all contact points to the ContactFilter object" << std::endl;
  bool publishMostLikely = true;
  bool publishAll = true;
  bool done = false;

  while (!done){

    // only run as fast as the residual state is getting updated,
    // otherwise we are essentially double counting the measurements
    while(!this->residual_state.running || !this->newResidualStateAvailable){
      std::this_thread::yield();
    }
    this->computeContactFilter(publishMostLikely, publishAll);
    this->newResidualStateAvailable = false;
  }
}

void ResidualDetector::activeLinkContactFilterThreadLoop() {
  std::cout << "entered contact filter body wrench loop" << std::endl;
  bool publish = true;
  bool useActiveLinkInfo = true;
  bool done = false;
  while (!done){
    while (!this->residual_state.running || !this->newResidualStateAvailableForActiveLink){
      std::this_thread::yield();
    }
    this->computeActiveLinkContactFilter(publish, useActiveLinkInfo);
    this->newResidualStateAvailableForActiveLink = false;
  }
}


// TESTING
void ResidualDetector::kinematicChain(std::string linkName, int body_id){

  // the default of body_id, then overwrite with linkName
  if (body_id==-1){
    body_id = this->drake_model.findLinkId(linkName);
  }
  VectorXd q = VectorXd::Zero(this->nq);
  VectorXd v = VectorXd::Zero(this->nv);

  // compute H for current state (q,qd)
  this->drake_model.doKinematics(q, v, true);
  std::vector<int> v_indices;
  auto footJacobian = this->drake_model.geometricJacobian<double>(0, body_id, body_id, 0, true, &v_indices).value();

  //now we want to print out what the v_indices correspond to
  std::cout << this->drake_model.getBodyOrFrameName(body_id) << " kinematic chain" << std::endl;
  for (auto &i: v_indices){
    std::cout << this->drake_model.getVelocityName(i) << std::endl;
  }

}


// TESTING
void ResidualDetector::testContactFilterRotationMethod(bool useRandom) {


  Vector3d a(1, 0, 0);
  Vector3d b(-1, 0, 0);
//  if (useRandom){
//    a = Vector3d::Random();
//    b = Vector3d::Random();
//    a.normalize();
//    b.normalize();
//  }

  auto R = rotateVectorToAlign(a,b);

  std::cout << "a is:\n" << a << std::endl;
  std::cout << "b is:\n" << b << std::endl;
  std::cout << "R is:\n" << R << std::endl;
  auto c = R*a;
  std::cout << "R*a is:\n" << c << std::endl;
  Vector3d d = c - b;
  double diffNorm = d.norm();
  std::cout << "norm of c-b is:\n" << diffNorm << std::endl;
}

void ResidualDetector::testQP(){
  VectorXd residual = VectorXd::Zero(this->nv);
  VectorXd q = residual;
  VectorXd v = residual;
  Vector3d contactPosition(0,0,0);
  Vector3d contactNormal(0,0,1);
//  this->contactFilter.computeLikelihood(residual, q, v, contactPosition, contactNormal, body_id);
  this->contactFilter.testQP();
}

// SHOULD DO THIS PROGRAMATICALLY LATER
std::vector<ContactFilterPoint> constructContactFilterPoints(){
  std::vector<ContactFilterPoint> vec;
  ContactFilterPoint cfp;

  cfp.body_id = 1;
  cfp.body_name = "pelvis";
  cfp.name = "ORIGIN";
  cfp.contactNormal = Vector3d(0,0,1);
  cfp.contactPoint = Vector3d(0,0,0);
  vec.push_back(cfp);

  cfp.body_id = 1;
  cfp.body_name = "pelvis";
  cfp.name = "L_PELVIS";
  cfp.contactNormal = Vector3d(0,0,1);
  cfp.contactPoint = Vector3d(-0.092,0.0616,-0.144);
  vec.push_back(cfp);

  cfp.body_id = 1;
  cfp.body_name = "pelvis";
  cfp.name = "R_PELVIS";
  cfp.contactNormal = Vector3d(0,0,1);
  cfp.contactPoint = Vector3d(-0.092,-0.0616,-0.144);
  vec.push_back(cfp);

  cfp.body_id = 1;
  cfp.body_name = "pelvis";
  cfp.name = "M_PELVIS";
  cfp.contactNormal = Vector3d(0,0,1);
  cfp.contactPoint = Vector3d(-0.092,0.0,-0.144);
  vec.push_back(cfp);


  cfp.body_id = 1;
  cfp.body_name = "pelvis";
  cfp.name = "B_PELVIS";
  cfp.contactNormal = Vector3d(1,0,0);
  cfp.contactPoint = Vector3d(-0.15477851,  0.00025932, -0.00269301);
  vec.push_back(cfp);

  return vec;
}


std::vector<std::vector<std::string>> parseCSVFile(std::string filename){
  using namespace std;
  ifstream file (filename);
  std::vector<std::vector<std::string>> result;
  if (!file.is_open()){
    std::cout << "couldn't find the specified file, is it on the path?" << std::endl;
    return result;
  }

  while (!file.eof()){

//    std::cout << "attempting to read a new line" << std::endl;
    //go through every line
    string line;
    string tmp_string;
    vector<string> tmp;
    size_t pos = string::npos;
    getline(file, line);



    while( (pos=line.find_first_of(",")) != string::npos){
//      std::cout << "the line I just read in is " << std::endl;
//      std::cout << line << std::endl;

//      std::cout << "found a comma at position " << pos << endl;
      // extract the component without the ","
      tmp.push_back(line.substr(0,pos));
      tmp_string = line.substr(0,pos);

//      std::cout << "the string I found is =  " << tmp_string << std::endl;

      //erase the val including the ","
      line.erase(0,pos+1);
//      std::cout << "after erasing the line is" << std::endl;
//      std::cout << line << std::endl;
//      std::cout << "made it through the loop" << std::endl;
    }
    if (tmp.size() > 0){
      result.push_back(tmp);
    }


  }


  return result;
}


std::vector<ContactFilterPoint> constructContactFilterPointsFromFile(std::string filename){

  std::string drcBase = std::string(std::getenv("DRC_BASE"));
  std::string filenameWithPath = drcBase + "/software/control/residual_detector/src/particle_grids/" + filename;
  std::vector<std::vector<std::string>> fileString = parseCSVFile(filenameWithPath);

//  // print out the file that you just read in
//  for (auto & line: fileString){
//    for (auto & word: line){
//      std::cout << word << ",";
//    }
//    std::cout << std::endl;
//  }

  std::vector<ContactFilterPoint> cfpVec;
  for (auto & line: fileString){
//    std::cout << "attemping to make cfp object" << std::endl;
//    std::cout << "line has size " << line.size() << std::endl;
    ContactFilterPoint cfp;
    cfp.body_name = line[0];
    Vector3d contactPoint(atof(line[1].c_str()), atof(line[2].c_str()), atof(line[3].c_str()));
    Vector3d contactNormal(atof(line[4].c_str()), atof(line[5].c_str()), atof(line[6].c_str()));
    cfp.contactPoint = contactPoint;
    cfp.contactNormal = contactNormal;

    if (line.size() == 8) {
      cfp.name = line[7];
    }
    cfpVec.push_back(cfp);
  }

  return cfpVec;

}



int main( int argc, char* argv[]){
  bool runTest = false;
  bool runQPTest = false;
  bool isVerbose = false;
  bool runDetectorLoop = true;
  bool runContactFilterLoop = false;
  bool runContactFilterActiveLinkLoop = false;
  bool runCSVTest = false;
  std::string urdfFilename = "";
  std::string filename = "directorDense.csv";
  std::string linkName;


  for (int i=1; i < argc; i++){
    if (std::string(argv[i]) == "--testCSVRead"){
      std::cout << "running CSV test" << std::endl;
      std::string filename = "test.csv";
      constructContactFilterPointsFromFile(filename);
      return 0;
    }

  }



  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr << "Error: lcm is not good()" << std::endl;
  }

  ResidualDetector residualDetector(lcm, isVerbose);

  if (argc==1){
    std::cout << "using foot force and torque" << std::endl;
    residualDetector.useFootForceTorque();
  }


  for (int i=1; i < argc; i++){
    if (i==1) {
      if (std::string(argv[i]) == "--useFootForceTorque") {
        std::cout << "using foot force & torque" << std::endl;
        residualDetector.useFootForceTorque();
      }
      else if (std::string(argv[i]) == "--useFootForce") {
        std::cout << "using foot force only, no torques" << std::endl;
        residualDetector.useFootForce();
      }
      else if (std::string(argv[i]) == "--useFootForceGeometricJacobian") {
        std::cout << "using foot force only, no torques, using geometric jacobian" << std::endl;
        residualDetector.useFootForce(true);
      }
      else if (std::string(argv[i]) == "--kinematicChain"){
        std::cout << "computing kinematicChain";
      }
      else if (std::string(argv[i])== "--runTest"){
        runDetectorLoop = false;
        runTest = true;
      }
      else if (std::string(argv[i]) == "--runQPTest"){
        runQPTest = true;
        runDetectorLoop = false;
      }
      else {
        std::cout << "didn't receive an option" << std::endl;
        std::cout << "using foot force & torque" << std::endl;
        residualDetector.useFootForceTorque();
      }
    }

    if (std::string(argv[i]) == "--runContactFilter") {
      std::cout << "running Contact Filter Loop" << std::endl;
      runContactFilterLoop = true;
    }

    if (std::string(argv[i]) == "--testCSVRead"){
      runContactFilterLoop = false;
      runDetectorLoop = false;
      runCSVTest = true;
    }

    if (std::string(argv[i]) == "--useDirectorCSV"){
      filename = "testDirector.csv";
    }

    if (std::string(argv[i]).find(".csv") != std::string::npos){
      filename = std::string(argv[i]);
    }

  }

  if (runContactFilterLoop){
    std::cout << "attemping to start contact filter thread loop" << std::endl;
    std::thread contactFilterThread(&ResidualDetector::contactFilterThreadLoop, &residualDetector, filename);
    std::cout << "started contact filter thread loop, detaching thread" << std::endl;
    contactFilterThread.detach();
  }

  if (runContactFilterActiveLinkLoop){
    std::cout << "attemping to start contact filter active link thread loop" << std::endl;
    std::thread contactFilterThread(&ResidualDetector::activeLinkContactFilterThreadLoop, &residualDetector);
    std::cout << "started active link contact filter thread loop, detaching thread" << std::endl;
    contactFilterThread.detach();
  }

  if(runTest){
    residualDetector.testContactFilterRotationMethod(true);
  }

  if(runQPTest){
    residualDetector.testQP();
  }

  if (runDetectorLoop){
    std::thread residualThread(&ResidualDetector::residualThreadLoop, &residualDetector);
    std::cout << "started residual thread loop" << std::endl;

    // in the main thread run the lcm handler
    std::cout << "Starting lcm handler loop" << std::endl;
    while(0 == lcm->handle()); // infinite loop, won't ever progress past this
  }

  if (runCSVTest){
    std::string filename = "test.csv";
    constructContactFilterPointsFromFile(filename);
  }

  return 0;
}

