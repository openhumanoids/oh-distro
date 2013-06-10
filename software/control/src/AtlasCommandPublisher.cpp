// faster way to publish AtlasCommand from Matlab
// Michael Kaess, June 2013
// based on AtlasCommandCoder.java

#include <mex.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/atlas_command_t.hpp"


using namespace Eigen;
using namespace std;


class AtlasCommand {

private:

  static lcm::LCM lcm;

  // Atlas plugin has a fixed order for joints, 
  // so number of joints here is fixed
  const int m_num_joints = 28;

  vector<int> drake_to_atlas_joint_map;

  int mode; // mode==1: torque-only, mode==2: position-only, fixed gains
  // TODO: add additional modes (e.g., position w/variable gains, mixed torque-position control
    
  drc::atlas_command_t msg;

public:

  AtlasCommand(const vector<string>& joint_name, const VectorXd& Kp, const VectorXd& Kd) :
    AtlasCommand(joint_name)
	{
    mode = 2;
    int j;
    for (int i=0; i<msg.num_joints; i++) {
      j = drake_to_atlas_joint_map[i];
      msg.kp_position[j] = Kp[i];
      msg.kd_position[j] = Kd[i];
    }
  }

  AtlasCommand(const vector<string>& joint_name) {
    mode = 1;

    if (joint_name.size() != m_num_joints) {
      mexErrMsgTxt("publishAtlasCommand: Length of joint_name wrong");
    }

    // fixed ordering assumed by drcsim interface
    vector<string> atlas_joint_name(m_num_joints);
    atlas_joint_name[0] = "back_lbz";
    atlas_joint_name[1] = "back_mby";
    atlas_joint_name[2] = "back_ubx";
    atlas_joint_name[3] = "neck_ay";
    atlas_joint_name[4] = "l_leg_uhz";
    atlas_joint_name[5] = "l_leg_mhx";
    atlas_joint_name[6] = "l_leg_lhy";
    atlas_joint_name[7] = "l_leg_kny";
    atlas_joint_name[8] = "l_leg_uay";
    atlas_joint_name[9] = "l_leg_lax";
    atlas_joint_name[10] = "r_leg_uhz";
    atlas_joint_name[11] = "r_leg_mhx";
    atlas_joint_name[12] = "r_leg_lhy";
    atlas_joint_name[13] = "r_leg_kny";
    atlas_joint_name[14] = "r_leg_uay";
    atlas_joint_name[15] = "r_leg_lax";
    atlas_joint_name[16] = "l_arm_usy";
    atlas_joint_name[17] = "l_arm_shx";
    atlas_joint_name[18] = "l_arm_ely";
    atlas_joint_name[19] = "l_arm_elx";
    atlas_joint_name[20] = "l_arm_uwy";
    atlas_joint_name[21] = "l_arm_mwx";
    atlas_joint_name[22] = "r_arm_usy";
    atlas_joint_name[23] = "r_arm_shx";
    atlas_joint_name[24] = "r_arm_ely";
    atlas_joint_name[25] = "r_arm_elx";
    atlas_joint_name[26] = "r_arm_uwy";
    atlas_joint_name[27] = "r_arm_mwx";

    drake_to_atlas_joint_map.reserve(m_num_joints);

    for (int i=0; i<m_num_joints; i++) {
      for (int j=0; j<m_num_joints; j++) {
        if (joint_name[i].compare(atlas_joint_name[j]) == 0)
          drake_to_atlas_joint_map[i]=j;
      }
    }

    msg.num_joints = m_num_joints;

    msg.position.resize(msg.num_joints);
    msg.velocity.resize(msg.num_joints);
    msg.effort.resize(msg.num_joints);

    msg.kp_position.resize(msg.num_joints);
    msg.ki_position.resize(msg.num_joints);
    msg.kd_position.resize(msg.num_joints);
    msg.kp_velocity.resize(msg.num_joints);

    msg.i_effort_min.resize(msg.num_joints);
    msg.i_effort_max.resize(msg.num_joints);

    msg.k_effort.resize(msg.num_joints);
    msg.desired_controller_period_ms = 5; // set desired controller rate (ms)

    for (int i=0; i<msg.num_joints; i++) {
      msg.position[i] = 0.0;
      msg.velocity[i] = 0.0;
      msg.effort[i] = 0.0;
      msg.kp_position[i] = 0.0;
      msg.ki_position[i] = 0.0;
      msg.kd_position[i] = 0.0;
      msg.kp_velocity[i] = 0.0;
      msg.i_effort_min[i] = 0.0;
      msg.i_effort_max[i] = 0.0;
      msg.effort[i] = 0.0;
      msg.k_effort[i] = (uint8_t)255; // take complete control of joints (remove BDI control)
    }
  }

  void publish(const string& channel, double t, const VectorXd& x) {
    if (mode==0) {
      mexErrMsgTxt("publishAtlasCommand: publish called before initialization");
    }

    msg.utime = (long)(t*1000000);
    int j;
    for (int i=0; i<m_num_joints; i++) {
      j = drake_to_atlas_joint_map[i];
      if (mode==1)
        msg.effort[j] = x(i);
      else if (mode==2)
        msg.position[j] = x(i);
    }

    lcm.publish(channel, &msg);
  }

};


// globals: static class members
lcm::LCM AtlasCommand::lcm;


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

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs==1 && mxGetNumberOfElements(prhs[0])>1) { // init()
  	if (nlhs<1) return;

  	vector<string> joint_name = get_strings(prhs[0]);

  	AtlasCommand *ac = new AtlasCommand(joint_name);
  	mxClassID cid;
  	if (sizeof(ac)==4) cid = mxUINT32_CLASS;
  	else if (sizeof(ac)==8) cid = mxUINT64_CLASS;
  	else mexErrMsgIdAndTxt("Drake:AtlasCommandPublisher:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
  	plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
  	memcpy(mxGetData(plhs[0]),&ac,sizeof(ac));

  	return;
  }

  if (nrhs==3 && mxGetNumberOfElements(prhs[1]) > 1) { // init2()
  	if (nlhs<1) return;

  	vector<string> joint_name = get_strings(prhs[0]);
  	Map<VectorXd> Kp(mxGetPr(prhs[1]), mxGetNumberOfElements(prhs[1]));
  	Map<VectorXd> Kd(mxGetPr(prhs[2]), mxGetNumberOfElements(prhs[2]));

  	AtlasCommand *ac = new AtlasCommand(joint_name,Kp,Kd);
  	mxClassID cid;
  	if (sizeof(ac)==4) cid = mxUINT32_CLASS;
  	else if (sizeof(ac)==8) cid = mxUINT64_CLASS;
  	else mexErrMsgIdAndTxt("Drake:AtlasCommandPublisher:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
  	plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
  	memcpy(mxGetData(plhs[0]),&ac,sizeof(ac));

  	return;
  }

  if (nlhs!=0) {
    mexErrMsgTxt("AtlasCommandPublisher: does not return anything");
  }

  // retrieve object
  AtlasCommand *ac = NULL;
  if (nrhs==0 || !mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:AtlasCommandPublisher:BadInputs","the first argument should be the mex_ptr");
  memcpy(&ac,mxGetData(prhs[0]),sizeof(ac));

  if (nrhs==1) { // delete()

  	if (ac) delete(ac);

  } else if (nrhs==4) { // publish()

    char* str = mxArrayToString(prhs[1]);
    string channel(str);
    mxFree(str);
    double *t = mxGetPr(prhs[2]);
    Map<VectorXd> x(mxGetPr(prhs[3]), mxGetNumberOfElements(prhs[3]));
    ac->publish(channel, *t, x);

  } else {

    mexErrMsgTxt("AtlasCommandPublisher: wrong number of input arguments");

  }

}
