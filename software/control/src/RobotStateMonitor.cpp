/* Calls the perception teams' MAP interface API */

#include <mex.h>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <sys/select.h>

#include <Eigen/Dense>
//#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/drc/robot_state_t.hpp>

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
      //
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

class RobotStateMonitor {


private:
  string m_robot_name;
  int m_num_joints;
  int m_num_floating_joints;
  int m_num_x=0;
	map<string,int> m_joint_map;
	map<string,int> m_floating_joint_map;
  long m_last_timestamp;
  long m_time_of_last_message;
  long m_reset_time=1000;
  bool m_has_new_message = false;

	double* m_x=NULL;

	mutex m_mutex;
	condition_variable m_received;

public:

  RobotStateMonitor(string robot_name, vector<string> joint_name)
	{
    m_robot_name = robot_name;

    m_num_joints = 5;
    m_num_floating_joints = 0;
    /*
    for (int i=0; i<joint_name.size(); i++) {
      if (joint_name[i].startsWith("base_")) {
        m_floating_joint_map.insert(joint_name[i],i);
        m_num_floating_joints+=1;
      }
      else {
        m_joint_map.insert(joint_name[i],i);
        m_num_joints+=1;
      }
    }
    */
    m_num_x = 2*(m_num_joints+m_num_floating_joints);
    m_x = new double[m_num_x];
	}
  virtual ~RobotStateMonitor(void)
  {
  	if (m_x) delete m_x;
  }

  void handleMessage(const lcm::ReceiveBuffer* rbuf,
          const string& chan,
          const drc::robot_state_t* msg)
  {
  	cout << "message received." << endl;
  	m_mutex.lock();
  	m_mutex.unlock();
  	m_received.notify_all();
  }

  mxArray* getNextState(long timeout_ms)
  {
/*
  	m_received.wait();
  	if (timeout) {
  		// return null
  	} else {
  	*/
    	mxArray* px = mxCreateDoubleMatrix(m_num_x,1,mxREAL);
    	memcpy(mxGetPr(px),m_x,m_num_x*sizeof(double));
//  	}
  }

  mxArray* getState(void)
  {
  	mxArray* px = mxCreateDoubleMatrix(m_num_x,1,mxREAL);
  	memcpy(mxGetPr(px),m_x,m_num_x*sizeof(double));
  }

  double getTime(void)
  {
  	return 0.0;
  }

  void markAsRead(void)
  {
  	// not implemented yet
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

// todo: implement mexAtExit to clean up the lcm thread
static void cleanupLCM(void) {
	mexPrintf("interrupting thread\n"); mexEvalString("drawnow");
	//    pdata->lcm_thread->interrupt();
	b_interrupt_lcm = true;
	mexPrintf("joining thread\n"); mexEvalString("drawnow");
	lcm_thread->join();
	mexPrintf("destroying thread\n"); mexEvalString("drawnow");
	delete lcm_thread;
	mexPrintf("done.\n"); mexEvalString("drawnow");
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	// combines the functionality of MessageMonitor.java and RobotStateCoder.java (decoding only)

  const char usage[] = "Usage:\n"
  		"  ptr  = RobotStateMonitor(robot_name,joint_names)  - initialize\n"
  		"         RobotStateMonitor(ptr)     - free\n"
  		"         RobotStateMonitor(ptr,0,channel) - subscribe\n"
  		" (t,x) = RobotStateMonitor(ptr,1,timeout) - get next message\n"
  		" (t,x) = RobotStateMonitor(ptr,2) - get current message\n"
  		"   t   = RobotStateMonitor(ptr,3) - get last timestamp\n"
  		"         RobotStateMonitor(ptr,4[,timestamp]) - mark as read\n";

  if (!lcm_thread) {    // create lcm instance, and fork listener thread
    if (!mylcm.good())
      mexErrMsgIdAndTxt("DRC:RobotStateMonitor:LCMFailed","Failed to create LCM instance");

    mexPrintf("spawning LCM thread\n"); mexEvalString("drawnow");
    b_interrupt_lcm = false;
    lcm_thread = new std::thread(lcmThreadMain);

    mexAtExit(cleanupLCM);
  }

  RobotStateMonitor* rsm = NULL;

  if (nrhs<1) mexErrMsgIdAndTxt("DRC:RobotStateMonitor:BadInputs",usage);

  if (mxIsChar(prhs[0])) { // ptr  = RobotStateMonitor(robot_name,joint_names)  - initialize
    char* str = mxArrayToString(prhs[0]);
    string robot_name(str);
    mxFree(str);

  	vector<string> joint_name = get_strings(prhs[1]);

  	rsm = new RobotStateMonitor(robot_name, joint_name);

    // return a pointer to the model
    mxClassID cid;
    if (sizeof(rsm)==4) cid = mxUINT32_CLASS;
    else if (sizeof(rsm)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("DRC:RobotStateMonitor:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&rsm,sizeof(rsm));

    return;
  }

  // load the data ptr
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:RobotStateMonitor:BadInputs","the first argument should be the ptr");
  memcpy(&rsm,mxGetData(prhs[0]),sizeof(rsm));
  
  if (nrhs<2) { // then free the memory and exit
  	if (rsm) delete rsm;
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
			long timeout = 0;  // todo: get from matlab
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
  default:
  	mexErrMsgIdAndTxt("DRC:RobotStateMonitor:UnknownCommand","Don't know command %d", command);
	}

}

