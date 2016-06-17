#include <mutex>
#include <thread>
// #include <chrono>
#include <atomic>
#include <sys/select.h>
#include "drake/lcmt_qp_controller_input.hpp"
#include "drc/controller_state_t.hpp"
#include "drc/controller_status_t.hpp"
#include "bot_core/robot_state_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "drake/systems/controllers/QPCommon.h"
#include "drake/util/drakeGeometryUtil.h"
#include "AtlasCommandDriver.hpp"
#include "FootContactDriver.hpp"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/robotInterfaces/Side.h"
#include "drake/systems/controllers/InstantaneousQPController.h"
#include "sfRobotState.h"
#include <csignal>

using namespace Eigen;
 
namespace {

std::mutex pointerMutex;
std::atomic<bool> hasNewQPIn(false);
std::atomic<bool> hasNewState(false);
std::atomic<bool> hasNewQPOut(false);

bot_core::robot_state_t robot_state_msg;
drake::lcmt_qp_controller_input qp_input_msg;
drc::controller_state_t qp_output_msg;

class LCMHandler {

public:

  std::thread ThreadHandle;
  std::shared_ptr<lcm::LCM> LCMHandle;
  bool ShouldStop;

  LCMHandler()
  {
    this->ShouldStop = false;
    this->InitLCM();
  }

  void InitLCM()
  {
    this->LCMHandle = std::shared_ptr<lcm::LCM>(new lcm::LCM);
    if(!this->LCMHandle->good())
    {
      std::cout << "ERROR: lcm is not good()" << std::endl;
    }
  }

  bool WaitForLCM(double timeout)
  {
    int lcmFd = this->LCMHandle->getFileno();

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout * 1e6;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcmFd, &fds);

    int status = select(lcmFd + 1, &fds, 0, 0, &tv);
    if (status == -1 && errno != EINTR)
    {
      printf("select() returned error: %d\n", errno);
    }
    else if (status == -1 && errno == EINTR)
    {
      printf("select() interrupted\n");
    }
    return (status > 0 && FD_ISSET(lcmFd, &fds));
  }

  void ThreadLoopWithSelect()
  {

    std::cout << "ThreadLoopWithSelect " << std::this_thread::get_id() << std::endl;

    while (!this->ShouldStop)
    {
      const double timeoutInSeconds = 0.3;
      bool lcmReady = this->WaitForLCM(timeoutInSeconds);

      if (this->ShouldStop)
      {
        break;
      }

      if (lcmReady)
      {
        if (this->LCMHandle->handle() != 0)
        {
          printf("lcm->handle() returned non-zero\n");
          break;
        }
      }
    }

    std::cout << "ThreadLoopWithSelect ended " << std::this_thread::get_id() << std::endl;

  }

  void ThreadLoop()
  {
    while (!this->ShouldStop)
    {
      if (this->LCMHandle->handle() != 0)
      {
        printf("lcm->handle() returned non-zero\n");
        break;
      }
    }
  }

  bool IsRunning()
  {
    return this->ThreadHandle.joinable();
  }

  void Start()
  {
    std::cout << "LCMHandler start... " << std::this_thread::get_id() << std::endl;
    if (this->IsRunning())
    {
      std::cout << "already running lcm thread. " << std::this_thread::get_id() << std::endl;
      return;
    }

    this->ShouldStop = false;
    this->ThreadHandle = std::thread(&LCMHandler::ThreadLoopWithSelect, this);
  }

  void Stop()
  {
    this->ShouldStop = true;
    this->ThreadHandle.join();
  }

}; 


class LCMControlReceiver {
public:

  LCMControlReceiver(LCMHandler* lh)
  {
    this->lcmHandler = lh;
  }

  void InitSubscriptions()
  {
    lcm::Subscription* sub;

    sub = this->lcmHandler->LCMHandle->subscribe("QP_CONTROLLER_INPUT", &LCMControlReceiver::qp_inputHandler, this);
    sub->setQueueCapacity(1);

    sub = this->lcmHandler->LCMHandle->subscribe("EST_ROBOT_STATE", &LCMControlReceiver::rsHandler, this);
    sub->setQueueCapacity(1);

    sub = this->lcmHandler->LCMHandle->subscribe("CONTROLLER_STATE", &LCMControlReceiver::qp_outputHandler, this);
    sub->setQueueCapacity(1);
    
    //sub = this->lcmHandler->LCMHandle->subscribe("FOOT_CONTACT_ESTIMATE", &LCMControlReceiver::onFootContact, this);
    //sub->setQueueCapacity(1);
  }

  void qp_inputHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drake::lcmt_qp_controller_input* msg)
  {
    //std::cout << "received qp_input on lcm thread " << std::this_thread::get_id() << std::endl;
    pointerMutex.lock();
    qp_input_msg = *msg; 
    pointerMutex.unlock();
    hasNewQPIn = true;
  }

  void rsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg)
  {
    //std::cout << "received robotstate on lcm thread " << std::this_thread::get_id() << std::endl;

    pointerMutex.lock();
    robot_state_msg = *msg;

    pointerMutex.unlock();
    hasNewState = true;
  }

  void qp_outputHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::controller_state_t* msg)
  {
    pointerMutex.lock();
    qp_output_msg = *msg;
    pointerMutex.unlock();
    hasNewQPOut = true;
  }

  LCMHandler* lcmHandler;
};


} // end anonymous namespace


std::unordered_map<std::string, int> computeBodyOrFrameNameToIdMap(
    const RigidBodyTree& robot) {
  auto id_map = std::unordered_map<std::string, int>();
  for (auto it = robot.bodies.begin(); it != robot.bodies.end(); ++it) {
    id_map[(*it)->linkname] = it - robot.bodies.begin();
  }

  for (auto it = robot.frames.begin(); it != robot.frames.end(); ++it) {
    id_map[(*it)->name] = -(it - robot.frames.begin()) - 2;
  }
  return id_map;
}

typedef Eigen::Matrix<double, 6, 1> Vector6d;


class sfQPInput {
public:
  VectorXd q_d;
  VectorXd qd_d;
  VectorXd qdd_d;

  Vector2d cop_d;
  Vector2d com_d;

  Vector6d pelvdd_d;
  Vector6d footdd_d[2];
};

/*
template <typename Scalar> Matrix<Scalar,6,1> getTaskSpaceVel(const RigidBodyTree &r, const KinematicsCache<Scalar> &cache, int body_or_frame_id, const Vector3d &local_offset = Vector3d::Zero())
{
  Matrix<Scalar,6,1> vel;
  Transform<Scalar, 3, Isometry> H_body_to_frame;
  int body_idx = r.parseBodyOrFrameID(body_or_frame_id, &H_body_to_frame);
  const auto &element = cache.getElement(*(r.bodies[body_idx]));
  Transform<Scalar, 3, Isometry> H_world_to_frame = element.transform_to_world * H_body_to_frame;

  Transform<Scalar, 3, Isometry> H_frame_to_pt(Transform<Scalar, 3, Isometry>::Identity());
  H_frame_to_pt.translation() = local_offset;
  Transform<Scalar, 3, Isometry> H_world_to_pt = H_world_to_frame * H_frame_to_pt;

  // because the new fake frame at point has the same twist as body / frame 
  Matrix<Scalar,6,1> twist_in_body = transformSpatialMotion(H_world_to_pt.inverse(), element.twist_in_world);
  vel.head(3) = H_world_to_pt.linear() * twist_in_body.head(3);
  vel.tail(3) = H_world_to_pt.linear() * twist_in_body.tail(3);

  return vel;  
}

template <typename Scalar> Matrix<Scalar,6,1> getTaskSpaceVel1(const RigidBodyTree &r, const KinematicsCache<Scalar> &cache, int body_or_frame_id, const Vector3d &local_offset = Vector3d::Zero())
{
  Transform<Scalar, 3, Isometry> H_body_to_frame;
  int body_idx = r.parseBodyOrFrameID(body_or_frame_id, &H_body_to_frame);

  const auto &element = cache.getElement(*(r.bodies[body_idx]));
  Vector6d T = element.twist_in_world;
  Vector3d pt = element.transform_to_world.translation();
  
  // get the body's task space vel
  Vector6d v = T;
  v.tail<3>() += v.head<3>().cross(pt); 

  // global offset between pt and body
  auto H_world_to_frame = element.transform_to_world * H_body_to_frame;
  Transform<Scalar, 3, Isometry> H_frame_to_pt(Transform<Scalar, 3, Isometry>::Identity());
  H_frame_to_pt.translation() = local_offset;
  auto H_world_to_pt = H_world_to_frame * H_frame_to_pt; 
  Vector3d world_offset = H_world_to_pt.translation() - element.transform_to_world.translation();
  
  // add the linear vel from the body rotation
  v.tail<3>() += v.head<3>().cross(world_offset);

  return v;
}

// implicityly says body wrt world expressed in world
MatrixXd getTaskSpaceJacobian(const RigidBodyTree &r, KinematicsCache<double> &cache, int body)
{
  KinematicPath body_path = r.findKinematicPath(0, body);
  MatrixXd J = r.geometricJacobian(cache, 0, body, body, true);
  J = r.compactToFull(J, body_path.joint_path, true);
  J.topRows(3) = cache.getElement(*(r.bodies[body])).transform_to_world.linear() * J.topRows(3);
  J.bottomRows(3) = cache.getElement(*(r.bodies[body])).transform_to_world.linear() * J.bottomRows(3);
  return J;
}

void a(const RigidBodyTree &r, KinematicsCache<double> &cache, int body)
{
  const auto &element = cache.getElement(*(r.bodies[body]));
  Vector6d T = element.twist_in_world;
  Vector3d pt = element.transform_to_world.translation();
  
  Vector6d v = T;
  v.tail<3>() += v.head<3>().cross(pt);

  std::cout << "v: " << v.transpose() << std::endl;
  Vector6d v1 = getTaskSpaceVel(r, cache, body);
  std::cout << "v1: " << v1.transpose() << std::endl;
}

void test_velocity()
{
  std::minstd_rand0 g1 (12345);

  std::string urdf("/home/siyuanfeng/code/oh-distro-private/software/models/val_description/urdf/valkyrie_sim_drake.urdf");
  RigidBodyTree r(urdf, DrakeJoint::ROLLPITCHYAW);
  KinematicsCache<double> cache(r.bodies);

  // make state
  //VectorXd q = r.getZeroConfiguration();
  //VectorXd qd = VectorXd::Zero(r.num_velocities);
  //qd[4] = 1;

  VectorXd q = r.getRandomConfiguration(g1);
  VectorXd qd = VectorXd::Random(r.num_velocities);

  cache.initialize(q, qd);
  r.doKinematics(cache, true);

  for (int i = 0; i < r.num_velocities; i++)
    std::cout << i << r.getVelocityName(i) << std::endl;
  
  // body name to idx 
  std::unordered_map<std::string, int> body_or_frame_name_to_id;
  body_or_frame_name_to_id = std::unordered_map<std::string, int>();
  for (auto it = r.bodies.begin(); it != r.bodies.end(); ++it) {
    body_or_frame_name_to_id[(*it)->linkname] = it - r.bodies.begin();
  } 

  // try foot
  int idx = body_or_frame_name_to_id.at("leftFoot");
  KinematicPath body_path = r.findKinematicPath(0, idx);
  // body wrt world, expressed in world
  MatrixXd J_w_w_b = r.geometricJacobian(cache, 0, idx, 0, true);
  J_w_w_b = r.compactToFull(J_w_w_b, body_path.joint_path, true);
  // body wrt world, expressed in body
  MatrixXd J_b_w_b = r.geometricJacobian(cache, 0, idx, idx, true);
  J_b_w_b = r.compactToFull(J_b_w_b, body_path.joint_path, true);

  VectorXd twist_w = J_w_w_b * qd;
  VectorXd twist_b = J_b_w_b * qd;

  Isometry3d b_2_w = r.relativeTransform(cache, 0, idx);
  Matrix<double,6,6> R(Matrix<double,6,6>::Zero());
  R.block<3,3>(0,0) = b_2_w.linear();
  R.block<3,3>(3,3) = b_2_w.linear();

  Vector6d T_w = cache.getElement(*r.bodies[r.parseBodyOrFrameID(idx)]).twist_in_world;
  Vector6d T_w_0 = cache.getElement(*r.bodies[0]).twist_in_world;

  std::cout << "twist_w: " << twist_w.transpose() << std::endl;
  std::cout << "T_w: " << T_w.transpose() << std::endl;
  std::cout << "transform  twist_b: " << transformSpatialMotion(b_2_w, twist_b).transpose() << std::endl;

  std::cout << std::endl;
  std::cout << std::endl;

  std::cout << "R * twist_b: " << (R * twist_b).transpose() << std::endl;
  std::cout << "vel: " << getTaskSpaceVel(r, cache, idx).transpose() << std::endl;
  
  //std::cout << "b_2_w: " << b_2_w.linear() << std::endl;
  //std::cout << "b_2_w: " << b_2_w.translation() << std::endl;

  MatrixXd JJ = r.transformPointsJacobian(cache, Vector3d::Zero(), idx, 0, true);
  MatrixXd Ja = getTaskSpaceJacobian(r, cache, idx);
  MatrixXd Ja1 = getTaskSpaceJacobian1(r, cache, idx, Vector3d::Zero());

  std::cout << "Ja * qd: " << Ja * qd << std::endl;
  std::cout << "Ja = JJ: " << (Ja.bottomRows(3) - JJ).isZero() << std::endl;
  std::cout << "Ja = Ja1: " << (Ja - Ja1).isZero() << std::endl;

  // test velocity
  Vector3d pt(0.3, 0.4, 0.5);
  Vector3d vec = b_2_w.linear() * pt;
  
  Vector6d new_vel = getTaskSpaceVel(r, cache, idx, pt);
  Vector6d new_vel1 = getTaskSpaceVel(r, cache, idx);
  new_vel1.tail(3) += new_vel1.segment<3>(0).cross(vec);
  
  Vector6d new_vel2 = getTaskSpaceVel1(r, cache, idx, pt);

  std::cout << "vel: " << new_vel << std::endl;
  std::cout << "vel1: " << new_vel1 << std::endl;
  std::cout << "vel2: " << new_vel2 << std::endl;

  a(r, cache, idx);

  std::cout << "Jdqd: " << r.transformPointsJacobianDotTimesV(cache, pt, idx, 0).transpose() << std::endl;
  std::cout << "Jdqd1: " << getTaskSpaceJacobianDotTimesV(r, cache, idx, pt).transpose() << std::endl;
  std::cout << "Jdqd2: " << r.geometricJacobianDotTimesV(cache, 0, idx, 0).transpose() << std::endl;
}
*/
  
BatchLogger logger;

void sigHandler(int signum)
{
  std::cout << "CAUGHT SIG " << signum << std::endl;
  if (signum == SIGINT) {
    logger.writeToMRDPLOT("/home/siyuanfeng/logs/mrdplot/bal");
  }
  std::cout << "GOODBYE CAPTAIN\n";
  exit(0);
}

int main ()
{
  signal(SIGINT, sigHandler);

  // start lcm stuff
  LCMHandler lcmHandler;
  LCMControlReceiver controlReceiver(&lcmHandler);

  std::string urdf("/home/siyuanfeng/code/oh-distro-private/software/models/val_description/urdf/valkyrie_sim_drake.urdf");
  sfRobotState rs(std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)));
  sfQPOutput qpout;
 
  // start the lcm 
  lcmHandler.Start();
  controlReceiver.InitSubscriptions();
  
  logger.init(0.002);

  while(1) {
    // proc robot state
    if (hasNewState) {
      if (!rs.hasInit()) {
        rs.init(robot_state_msg);
        rs.addToLog(logger);
      }

      rs.parseMsg(robot_state_msg);
      
      hasNewState = false;

      //for (int i = 0; i < rs.trq.size(); i++) {
      //  std::cout << rs.robot->getPositionName(i) << ": " << rs.trq[i] << ", " << trql[i] << ", " << trqr[i] << std::endl;
      //}
      std::cout << "torsod: " << rs.torso.vel.transpose() << std::endl;

      std::cout << "statics l: " << rs.footFT_w_statics[0] << std::endl;
      std::cout << "statics r: " << rs.footFT_w_statics[1] << std::endl;
      std::cout << "fl: " << rs.footFT_w[0] << std::endl;
      std::cout << "fr: " << rs.footFT_w[1] << std::endl;
      
      std::cout << "com: " << rs.com[0] << ", " << rs.com[1] << std::endl;
      std::cout << "cop: " << rs.cop[0] << ", " << rs.cop[1] << std::endl;

      logger.saveData();
    }
    // proc qp output
    if (hasNewQPOut) {
      if (!qpout.hasInit())
        qpout.init(qp_output_msg);

      if (rs.hasInit())
        qpout.parseMsg(qp_output_msg, rs);

      std::cout << "comdd_qp: " << qp_output_msg.comdd[0] << " " << qp_output_msg.comdd[1] << " " <<  qp_output_msg.comdd[2] << std::endl; 
      std::cout << "comdd: " << qpout.comdd << std::endl;
      std::cout << "lfdd: " << qpout.footdd[0] << std::endl;
      std::cout << "rfdd: " << qpout.footdd[1] << std::endl;
      hasNewQPOut = false;
    }
  }

  return 0;
}
