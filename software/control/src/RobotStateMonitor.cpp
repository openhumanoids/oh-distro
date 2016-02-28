/* Calls the perception teams' MAP interface API */

#include <mex.h>
#include <ctime>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <sys/select.h>
#include <unistd.h>

// thread and mutex stuff
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

//#ifdef __APPLE__
  #include <atomic>
//#else
//  #include <cstdatomic>
//#endif


#include <Eigen/Dense>
//#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core/robot_state_t.hpp>

#include <sys/time.h>

using namespace Eigen;
using namespace std;

// global variables
lcm::LCM mylcm;
thread* lcm_thread = NULL;
bool b_interrupt_lcm = false;


void lcmThreadMain(void) {
  //  while(0 == pdata->lcm->handle()) {
  while(!b_interrupt_lcm) {
    int fn = mylcm.getFileno();
    fd_set input_set;
    FD_ZERO(&input_set);
    FD_SET(fn, &input_set);
    struct timeval timeout = { 0, 200*1000 };
    int status = select(fn+1, &input_set, NULL, NULL, &timeout);
    if (status == 0) {

    }
    else if (status < 0) {
      mexPrintf("error in lcm pipe"); mexEvalString("drawnow");
      break;
    }
    else if (FD_ISSET(fn, &input_set)) {
      if (0 != mylcm.handle()) {
        mexPrintf("error in lcm handle\n"); mexEvalString("drawnow");
        break;
      }
    }
  }
}

// based on drake.util.Transform
Vector3d quat2rpy(Vector4d& q) {
  double norm=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  if (abs(norm)>1e-12) {
    q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
  }

  double w=q[0], x=q[1], y=q[2], z=q[3];
    
  Vector3d rpy;
  rpy[0] = atan2(2.0*(w*x + y*z), w*w + z*z -(x*x +y*y));
  rpy[1] = asin(2*(w*y - z*x));
  rpy[2] = atan2(2*(w*z + x*y), w*w + x*x-(y*y+z*z));
    
  return rpy;
}

Vector3d angularvel2rpydot(const Vector3d& rpy, const Vector3d& omega) {
  Vector3d rpydot;

  rpydot[0] = cos(rpy[2])/cos(rpy[1])*omega[0] + sin(rpy[2])/cos(rpy[1])*omega[1];
  rpydot[1] = -sin(rpy[2])*omega[0] + cos(rpy[2])*omega[1];
  rpydot[2] = cos(rpy[2])*tan(rpy[1])*omega[0] + tan(rpy[1])*sin(rpy[2])*omega[1] + omega[2];
    
  return rpydot;
}

long get_systime_ms() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec*1000 + t.tv_usec/1000;
}


class RobotStateMonitor {


private:
  string m_robot_name;
  int m_num_joints;
  int m_num_floating_joints;
  map<string,int> m_joint_map;
  map<string,int> m_floating_joint_map;
  
  int m_min_usec_between_msg; // us, min time between messages (useful for setting a maximum rate)
  atomic_long m_last_timestamp; // us
  long m_time_of_last_message; // ms
  const static long m_reset_time=1000; // ms
  atomic_long m_has_new_message;  // atomic_bool was not available on mac clang

  int m_num_x;
  double* m_x;

  mutex m_mutex;
  //  mutex m_received_mutex;
  //  condition_variable m_received;

public:

  RobotStateMonitor(string robot_name, vector<string> joint_name) : m_has_new_message(false), m_num_x(0), m_x(NULL)
  {
    m_robot_name = robot_name;
    m_min_usec_between_msg = 0;

    m_num_joints = 0;
    m_num_floating_joints = 0;
    string prefix("base_");
    for (int i=0; i<joint_name.size(); i++) {
      if (joint_name[i].compare(0, prefix.size(), prefix) == 0) {
        m_floating_joint_map[joint_name[i]] = i;
        m_num_floating_joints++;
      }
      else {
        m_joint_map[joint_name[i]] = i;
        m_num_joints++;
      }
    }

    // local stuff
    m_num_x = 2*(m_num_joints+m_num_floating_joints);
    m_x = new double[m_num_x];
  }

  virtual ~RobotStateMonitor(void)
  {
    mexPrintf("rsm destructor called\n"); mexEvalString("drawnow");
    if (m_x) delete m_x;
  }

  void setMinMicrosecBetweenMsg(int min_usec_between_msg) {
    m_min_usec_between_msg = min_usec_between_msg;
  }

  void handleMessage(const lcm::ReceiveBuffer* rbuf,
          const string& chan,
          const bot_core::robot_state_t* msg)
  {
    long systime = get_systime_ms();

    m_mutex.lock();
    // include a 1 second timeout
    // NOTE: utime/timestamp is in microseconds, systime/time_of_last_message in ms

    if (msg->utime >= (m_last_timestamp+m_min_usec_between_msg) || systime-m_time_of_last_message >= m_reset_time) {

      m_last_timestamp = msg->utime;
      for (int i=0; i<msg->num_joints; i++) {
        map<string,int>::iterator it = m_joint_map.find(msg->joint_name[i]);
        if (it!=m_joint_map.end()) {
          int index = it->second;
          m_x[index] = msg->joint_position[i];
          m_x[index+m_num_joints+m_num_floating_joints] = msg->joint_velocity[i];
        }
      }

      // get floating joint body position and orientation
      // TODO: should be generalized eventually
      map<string,int>::iterator it;
      it = m_floating_joint_map.find("base_x");
      if (it!=m_floating_joint_map.end()) {
        int index = it->second;
        m_x[index] = msg->pose.translation.x;
        m_x[index+m_num_joints+m_num_floating_joints] = msg->twist.linear_velocity.x;
      }
      it = m_floating_joint_map.find("base_y");
      if (it!=m_floating_joint_map.end()) {
        int index = it->second;
        m_x[index] = msg->pose.translation.y;
        m_x[index+m_num_joints+m_num_floating_joints] = msg->twist.linear_velocity.y;
      }
      it = m_floating_joint_map.find("base_z");
      if (it!=m_floating_joint_map.end()) {
        int index = it->second;
        m_x[index] = msg->pose.translation.z;
        m_x[index+m_num_joints+m_num_floating_joints] = msg->twist.linear_velocity.z;
      }

      Vector4d q;
      q[0] = msg->pose.rotation.w;
      q[1] = msg->pose.rotation.x;
      q[2] = msg->pose.rotation.y;
      q[3] = msg->pose.rotation.z;
      Vector3d rpy = quat2rpy(q);

      Vector3d omega;
      omega[0] = msg->twist.angular_velocity.x;
      omega[1] = msg->twist.angular_velocity.y;
      omega[2] = msg->twist.angular_velocity.z;
      Vector3d rpydot = angularvel2rpydot(rpy,omega);

      it = m_floating_joint_map.find("base_roll");
      if (it!=m_floating_joint_map.end()) {
        int index = it->second;
        m_x[index] = rpy[0];
        m_x[index+m_num_joints+m_num_floating_joints] = rpydot[0];
      }

      it = m_floating_joint_map.find("base_pitch");
      if (it!=m_floating_joint_map.end()) {
        int index = it->second; 
        m_x[index] = rpy[1];
        m_x[index+m_num_joints+m_num_floating_joints] = rpydot[1];
      }

      it = m_floating_joint_map.find("base_yaw");
      if (it!=m_floating_joint_map.end()) {
        int index = it->second; 
        m_x[index] = rpy[2];
        m_x[index+m_num_joints+m_num_floating_joints] = rpydot[2];
      }

      m_has_new_message = true;

    }

    m_time_of_last_message = systime;
    m_mutex.unlock();
    //    m_received.notify_all();
  }

#if 1
  mxArray* getNextState(long timeout_ms)
  {
    if (m_has_new_message) {
      return getState();
    } else {
      long systime = get_systime_ms();
      while (!m_has_new_message && (get_systime_ms() - systime) < timeout_ms) {
        usleep(20);
      }
      if (!m_has_new_message) {
        // time out
        mxArray* px;
        px = mxCreateDoubleMatrix(0,0,mxREAL);
        return px;
      } else {
        return getState();
      }
    }
  }
#else
  mxArray* getNextState(long timeout_ms)
  {

    mxArray* px;
    unique_lock<mutex> lk(m_received_mutex);
    if (m_received.wait_for(lk, chrono::milliseconds(timeout_ms)) == 0) {
      // time out
      px = mxCreateDoubleMatrix(0,0,mxREAL);
      return px;
    } else {
      return getState();
    }
  }
#endif

  mxArray* getState(void)
  {
    mxArray* px = mxCreateDoubleMatrix(m_num_x,1,mxREAL);
    m_mutex.lock();
    memcpy(mxGetPr(px),m_x,m_num_x*sizeof(double));
    m_has_new_message = false;
    m_mutex.unlock();
    return px;
  }

  double getTime(void)
  {
    //atomic    m_mutex.lock();
    double time = (double)m_last_timestamp / 1000000.0;
    //    m_mutex.unlock();
    return time;
  }

  void markAsRead(void)
  {
    //atomic    m_mutex.lock();
    m_has_new_message = false;
    //    m_mutex.unlock();
  }
};

// convert Matlab cell array of strings into a C++ vector of strings
vector<string> get_strings(const mxArray *rhs) {
  int num = mxGetNumberOfElements(rhs);
  vector<string> strings(num);
  for (int i=0; i<num; i++) {
    const mxArray *ptr = mxGetCell(rhs,i);
    int buflen = mxGetN(ptr)*sizeof(mxChar)+1;
    char* str = (char*)mxMalloc(buflen);
    mxGetString(ptr, str, buflen);
    strings[i] = string(str);
    mxFree(str);
  }
  return strings;
}

static void cleanupLCM(void) {
//  mexPrintf("interrupting thread\n"); mexEvalString("drawnow");
  //    pdata->lcm_thread->interrupt();
  b_interrupt_lcm = true;
//  mexPrintf("joining thread\n"); mexEvalString("drawnow");
  lcm_thread->join();
//  mexPrintf("destroying thread\n"); mexEvalString("drawnow");
  delete lcm_thread;
//  mexPrintf("done.\n"); mexEvalString("drawnow");
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  // combines the functionality of MessageMonitor.java and RobotStateCoder.java (decoding only)

  const char usage[] = "Usage:\n"
      "  ptr  = RobotStateMonitor(robot_name,joint_names,min_usec_between_msg)  - initialize\n"
      "         RobotStateMonitor(ptr)     - free\n"
      "         RobotStateMonitor(ptr,0,channel) - subscribe\n"
      " (t,x) = RobotStateMonitor(ptr,1,timeout) - get next message\n"
      " (t,x) = RobotStateMonitor(ptr,2) - get current message\n"
      "   t   = RobotStateMonitor(ptr,3) - get last timestamp\n"
      "         RobotStateMonitor(ptr,4[,timestamp]) - mark as read\n";

  RobotStateMonitor* rsm = NULL;

  if (nrhs<1) mexErrMsgIdAndTxt("DRC:RobotStateMonitor:BadInputs",usage);

  if (mxIsChar(prhs[0])) { // ptr  = RobotStateMonitor(robot_name,joint_names,min_ms_between_msg)  - initialize
    char* str = mxArrayToString(prhs[0]);
    string robot_name(str);
    mxFree(str);

    vector<string> joint_names = get_strings(prhs[1]);

    mexLock();
    rsm = new RobotStateMonitor(robot_name, joint_names);

    // return a pointer to the model
    mxClassID cid;
    if (sizeof(rsm)==4) cid = mxUINT32_CLASS;
    else if (sizeof(rsm)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("DRC:RobotStateMonitor:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&rsm,sizeof(rsm));


    if (!lcm_thread) {    // create lcm instance, and fork listener thread
      if (!mylcm.good())
        mexErrMsgIdAndTxt("DRC:RobotStateMonitor:LCMFailed","Failed to create LCM instance");

//      mexPrintf("spawning LCM thread\n"); mexEvalString("drawnow");
      b_interrupt_lcm = false;
      lcm_thread = new std::thread(lcmThreadMain);

      mexAtExit(cleanupLCM);
    }

    return;
  }

  // load the data ptr
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:RobotStateMonitor:BadInputs","the first argument should be the ptr");
  memcpy(&rsm,mxGetData(prhs[0]),sizeof(rsm));

  if (nrhs<2) { // delete: free the memory and exit
    if (rsm) delete rsm;
    mexUnlock();
    return;
  }

  int command = (int) mxGetScalar(prhs[1]);
  switch (command) {
  case 0:  // subscribe
    {
      char* channel = mxArrayToString(prhs[2]);
      mylcm.subscribe(channel, &RobotStateMonitor::handleMessage, rsm);
      mxFree(channel);
    }
    break;
  case 1:  // get next message
    {
      long timeout = (long)mxGetScalar(prhs[2]);
      if (nlhs>0) plhs[0] = rsm->getNextState(timeout);
      if (nlhs>1) plhs[1] = mxCreateDoubleScalar(rsm->getTime());
    }
    break;
  case 2:  // get current message
    {
      if (nlhs>0) plhs[0] = rsm->getState();
      if (nlhs>1) plhs[1] = mxCreateDoubleScalar(rsm->getTime());
    }
    break;
  case 3:  // get last timestamp
    {
      if (nlhs>0) plhs[0] = mxCreateDoubleScalar(rsm->getTime());
    }
    break;
  case 4:  // mark as read
    {
      rsm->markAsRead();
    }
    break;
  case 5: // set msg rate
    {
      int min_usec_between_msg = (int) mxGetScalar(prhs[2]);
      rsm->setMinMicrosecBetweenMsg(min_usec_between_msg);    
    }
    break;
  default:
    mexErrMsgIdAndTxt("DRC:RobotStateMonitor:UnknownCommand","Don't know command %d", command);
  }

}
