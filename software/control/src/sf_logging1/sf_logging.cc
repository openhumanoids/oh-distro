#include <mutex>
#include <thread>
#include <atomic>
#include <sys/select.h>
#include "bot_core/robot_state_t.hpp"
#include "bot_core/quaternion_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include "drake/systems/controllers/QPCommon.h"
#include "drake/util/drakeGeometryUtil.h"
#include "../AtlasCommandDriver.hpp"
#include "../FootContactDriver.hpp"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/robotInterfaces/Side.h"
#include "drake/systems/controllers/InstantaneousQPController.h"
#include <csignal>

#include "humanoid_status.h"
#include "qp_controller_state.h"
#include "drake/Path.h"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <ctime>

std::mutex pointerMutex;
std::atomic<bool> hasNewQPIn(false);
std::atomic<bool> hasNewState(false);
std::atomic<bool> hasNewQPOut(false);
bool quit = false;

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

static void sigHandler(int signum)
{
  std::cout << "CAUGHT SIG " << signum << std::endl;
  quit = true;
}

int main(int argc, const char *argv[])
{
  /*
  if (argc != 3 && argc != 1) {
    std::cerr << "usage: sf_logging1 <urdf_path> <config.yaml>\n";
    return -1;
  }
  */

  std::string urdf, config;
  if (argc == 3) {
    urdf = std::string(argv[1]);
    config = std::string(argv[2]);
  }
  else if (argc == 1) {
    config = Drake::getDrakePath() + std::string("/../../config/atlas_sim_mit/plan_eval_config_atlas.yaml");
    urdf = Drake::getDrakePath() + std::string("/examples/Atlas/urdf/atlas_minimal_contact.urdf");
  }
  else if (argc == 2) {
    std::string robot_name = std::string(argv[1]);
    if (robot_name.compare("atlas") == 0) {
      config = Drake::getDrakePath() + std::string("/../../config/atlas_sim_mit/plan_eval_config_atlas.yaml");
      urdf = Drake::getDrakePath() + std::string("/examples/Atlas/urdf/atlas_minimal_contact.urdf");
    }
    else if (robot_name.compare("val") == 0 || robot_name.compare("valkyrie") == 0) {
      config = Drake::getDrakePath() + std::string("/../../config/val_mit/plan_eval_config_valkyrie.yaml");
      urdf = Drake::getDrakePath() + std::string("/../../models/val_description/urdf/valkyrie_sim_drake.urdf");
    }
    else {
      std::cerr << "usage: \"sf_logging1 atlas\" or sf_logging1 val\n";
      return -1;
    }
  }

  struct passwd *pw = getpwuid(getuid());

  std::cout << "Using urdf: " << urdf << std::endl;
  std::cout << "Using config: " << config << std::endl;

  signal(SIGINT, sigHandler);

  // start lcm stuff
  LCMHandler lcmHandler;
  LCMControlReceiver controlReceiver(&lcmHandler);

  HumanoidStatus rs(config, std::unique_ptr<RigidBodyTree>(new RigidBodyTree(urdf, DrakeJoint::ROLLPITCHYAW)));
  QPIO qp_io(rs);

  // start the lcm
  lcmHandler.Start();
  controlReceiver.InitSubscriptions();

  MRDLogger logger;
  logger.set_frequency(500);

  while(!quit) {
    // proc robot state
    if (hasNewState) {
      if (!rs.has_init()) {
        rs.Init(robot_state_msg);
        rs.AddToLog(logger);
      }

      rs.ParseMsg(robot_state_msg);

      hasNewState = false;

      if (qp_io.has_init() && rs.has_init())
        logger.SaveData();
    }
    // set qp input
    if (hasNewQPIn) {
      qp_io.ParseZMPInput(qp_input_msg);
    }
    // proc qp output
    if (hasNewQPOut && rs.has_init()) {
      if (!qp_io.has_init()) {
        qp_io.Init(qp_output_msg);
        qp_io.AddToLog(logger, rs);
      }

      qp_io.ParseMsg(qp_output_msg, rs);

      hasNewQPOut = false;
    }
  }

  std::time_t t = std::time(NULL);
  char buf[100];
  std::strftime(buf, sizeof(buf), "%y_%m_%d_%H_%M_%S.mrd", std::localtime(&t));
  std::string name = std::string(buf);
  name = std::string(pw->pw_dir) + std::string("/logs/mrdplot/") + name;

  logger.WriteToFile(name);
  std::cout << "Saved to: " << name << std::endl;

  lcmHandler.Stop();

  return 0;
}
