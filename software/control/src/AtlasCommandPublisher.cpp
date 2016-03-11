#include <mex.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/bot_core/atlas_command_t.hpp"


using namespace Eigen;
using namespace std;


class AtlasCommand {

private:

  static lcm::LCM lcm;

  int m_num_joints; 
  
  vector<int> drake_to_atlas_joint_map;
  bot_core::atlas_command_t msg;

public:
  AtlasCommand(const vector<string>& joint_names, const int atlas_version_number, 
    const VectorXd& k_q_p, 
    const VectorXd& k_q_i, const VectorXd& k_qd_p, const VectorXd& k_f_p, 
    const VectorXd& ff_qd, const VectorXd& ff_qd_d, const VectorXd& ff_f_d,
    const VectorXd& ff_const) {

    if (atlas_version_number == 3 || atlas_version_number == 4) {
      m_num_joints = 28;  
    }
    else if (atlas_version_number == 5){
      m_num_joints = 30;  
    }
    else {
      mexErrMsgTxt("Unknown atlas_version_number");
    }
    
    // fixed ordering assumed by drcsim interface AND atlas api 
    // see: AtlasControlTypes.h 
    vector<string> atlas_joint_names(m_num_joints);
    switch (atlas_version_number) {
      case 3:
        atlas_joint_names[0] = "back_bkz";
        atlas_joint_names[1] = "back_bky";
        atlas_joint_names[2] = "back_bkx";
        atlas_joint_names[3] = "neck_ay";
        atlas_joint_names[4] = "l_leg_hpz";
        atlas_joint_names[5] = "l_leg_hpx";
        atlas_joint_names[6] = "l_leg_hpy";
        atlas_joint_names[7] = "l_leg_kny";
        atlas_joint_names[8] = "l_leg_aky";
        atlas_joint_names[9] = "l_leg_akx";
        atlas_joint_names[10] = "r_leg_hpz";
        atlas_joint_names[11] = "r_leg_hpx";
        atlas_joint_names[12] = "r_leg_hpy";
        atlas_joint_names[13] = "r_leg_kny";
        atlas_joint_names[14] = "r_leg_aky";
        atlas_joint_names[15] = "r_leg_akx";
        atlas_joint_names[16] = "l_arm_usy";
        atlas_joint_names[17] = "l_arm_shx";
        atlas_joint_names[18] = "l_arm_ely";
        atlas_joint_names[19] = "l_arm_elx";
        atlas_joint_names[20] = "l_arm_uwy";
        atlas_joint_names[21] = "l_arm_mwx";
        atlas_joint_names[22] = "r_arm_usy";
        atlas_joint_names[23] = "r_arm_shx";
        atlas_joint_names[24] = "r_arm_ely";
        atlas_joint_names[25] = "r_arm_elx";
        atlas_joint_names[26] = "r_arm_uwy";
        atlas_joint_names[27] = "r_arm_mwx";
        break;
      case 4:
        atlas_joint_names[0] = "back_bkz";
        atlas_joint_names[1] = "back_bky";
        atlas_joint_names[2] = "back_bkx";
        atlas_joint_names[3] = "neck_ay";
        atlas_joint_names[4] = "l_leg_hpz";
        atlas_joint_names[5] = "l_leg_hpx";
        atlas_joint_names[6] = "l_leg_hpy";
        atlas_joint_names[7] = "l_leg_kny";
        atlas_joint_names[8] = "l_leg_aky";
        atlas_joint_names[9] = "l_leg_akx";
        atlas_joint_names[10] = "r_leg_hpz";
        atlas_joint_names[11] = "r_leg_hpx";
        atlas_joint_names[12] = "r_leg_hpy";
        atlas_joint_names[13] = "r_leg_kny";
        atlas_joint_names[14] = "r_leg_aky";
        atlas_joint_names[15] = "r_leg_akx";
        atlas_joint_names[16] = "l_arm_shz";
        atlas_joint_names[17] = "l_arm_shx";
        atlas_joint_names[18] = "l_arm_ely";
        atlas_joint_names[19] = "l_arm_elx";
        atlas_joint_names[20] = "l_arm_uwy";
        atlas_joint_names[21] = "l_arm_mwx";
        atlas_joint_names[22] = "r_arm_shz";
        atlas_joint_names[23] = "r_arm_shx";
        atlas_joint_names[24] = "r_arm_ely";
        atlas_joint_names[25] = "r_arm_elx";
        atlas_joint_names[26] = "r_arm_uwy";
        atlas_joint_names[27] = "r_arm_mwx";
        break;
      case 5:
        atlas_joint_names[0] = "back_bkz";
        atlas_joint_names[1] = "back_bky";
        atlas_joint_names[2] = "back_bkx";
        atlas_joint_names[3] = "neck_ay";
        atlas_joint_names[4] = "l_leg_hpz";
        atlas_joint_names[5] = "l_leg_hpx";
        atlas_joint_names[6] = "l_leg_hpy";
        atlas_joint_names[7] = "l_leg_kny";
        atlas_joint_names[8] = "l_leg_aky";
        atlas_joint_names[9] = "l_leg_akx";
        atlas_joint_names[10] = "r_leg_hpz";
        atlas_joint_names[11] = "r_leg_hpx";
        atlas_joint_names[12] = "r_leg_hpy";
        atlas_joint_names[13] = "r_leg_kny";
        atlas_joint_names[14] = "r_leg_aky";
        atlas_joint_names[15] = "r_leg_akx";
        atlas_joint_names[16] = "l_arm_shz";
        atlas_joint_names[17] = "l_arm_shx";
        atlas_joint_names[18] = "l_arm_ely";
        atlas_joint_names[19] = "l_arm_elx";
        atlas_joint_names[20] = "l_arm_uwy";
        atlas_joint_names[21] = "l_arm_mwx";
        atlas_joint_names[22] = "l_arm_lwy";
        atlas_joint_names[23] = "r_arm_shz";
        atlas_joint_names[24] = "r_arm_shx";
        atlas_joint_names[25] = "r_arm_ely";
        atlas_joint_names[26] = "r_arm_elx";
        atlas_joint_names[27] = "r_arm_uwy";
        atlas_joint_names[28] = "r_arm_mwx";
        atlas_joint_names[29] = "r_arm_lwy";
        break;
      default:
        mexErrMsgTxt("Unknown atlas_version_number");
    }
    

    if (joint_names.size() != m_num_joints)
      mexErrMsgTxt("Length of joint_names must be equal to m_num_joints");
    if (k_q_p.size() != m_num_joints)
      mexErrMsgTxt("Length of k_q_p must be equal to m_num_joints");
    if (k_q_i.size() != m_num_joints)
      mexErrMsgTxt("Length of k_q_i must be equal to m_num_joints");
    if (k_qd_p.size() != m_num_joints)
      mexErrMsgTxt("Length of k_qd_p must be equal to m_num_joints");
    if (k_f_p.size() != m_num_joints)
      mexErrMsgTxt("Length of k_f_p must be equal to m_num_joints");
    if (ff_qd.size() != m_num_joints)
      mexErrMsgTxt("Length of ff_qd must be equal to m_num_joints");
    if (ff_qd_d.size() != m_num_joints)
      mexErrMsgTxt("Length of ff_qd_d must be equal to m_num_joints");
    if (ff_f_d.size() != m_num_joints)
      mexErrMsgTxt("Length of ff_f_d must be equal to m_num_joints");
    if (ff_const.size() != m_num_joints)
      mexErrMsgTxt("Length of ff_const must be equal to m_num_joints");

    drake_to_atlas_joint_map.reserve(m_num_joints);

    msg.num_joints = m_num_joints;
    msg.joint_names.resize(msg.num_joints);

    msg.position.resize(msg.num_joints);
    msg.velocity.resize(msg.num_joints);
    msg.effort.resize(msg.num_joints);

    msg.k_q_p.resize(msg.num_joints);
    msg.k_q_i.resize(msg.num_joints);
    msg.k_qd_p.resize(msg.num_joints);
    msg.k_f_p.resize(msg.num_joints);
    msg.ff_qd.resize(msg.num_joints);
    msg.ff_qd_d.resize(msg.num_joints);
    msg.ff_f_d.resize(msg.num_joints);
    msg.ff_const.resize(msg.num_joints);

    msg.k_effort.resize(msg.num_joints); // only used in sim
    msg.desired_controller_period_ms = 3; // set desired controller rate (ms), only used in sim

    drake_to_atlas_joint_map.reserve(m_num_joints);
    for (int i=0; i<m_num_joints; i++) {
      for (int j=0; j<m_num_joints; j++) {
        if (joint_names[i].compare(atlas_joint_names[j]) == 0)
          drake_to_atlas_joint_map[i]=j;
      }

      msg.position[i] = 0.0;
      msg.velocity[i] = 0.0;
      msg.effort[i] = 0.0;

      msg.joint_names[drake_to_atlas_joint_map[i]] = joint_names[i];
      
      msg.k_q_p[drake_to_atlas_joint_map[i]] = k_q_p[i];
      msg.k_q_i[drake_to_atlas_joint_map[i]] = k_q_i[i];;
      msg.k_qd_p[drake_to_atlas_joint_map[i]] = k_qd_p[i];;
      msg.k_f_p[drake_to_atlas_joint_map[i]] = k_f_p[i];;
      msg.ff_qd[drake_to_atlas_joint_map[i]] = ff_qd[i];;
      msg.ff_qd_d[drake_to_atlas_joint_map[i]] = ff_qd_d[i];;
      msg.ff_f_d[drake_to_atlas_joint_map[i]] = ff_f_d[i];;
      msg.ff_const[drake_to_atlas_joint_map[i]] = ff_const[i];;

      msg.k_effort[i] = (uint8_t)255; // take complete control of joints (remove BDI control), sim only
    }
  }

  void updateGains(const VectorXd& k_q_p, const VectorXd& k_q_i,const VectorXd& k_qd_p, const VectorXd& k_f_p, 
      const VectorXd& ff_qd, const VectorXd& ff_qd_d, const VectorXd& ff_f_d, const VectorXd& ff_const) {

    if (k_q_p.size() != m_num_joints)
      mexErrMsgTxt("Length of k_q_p must be equal to m_num_joints");
    if (k_q_i.size() != m_num_joints)
      mexErrMsgTxt("Length of k_q_i must be equal to m_num_joints");
    if (k_qd_p.size() != m_num_joints)
      mexErrMsgTxt("Length of k_qd_p must be equal to m_num_joints");
    if (k_f_p.size() != m_num_joints)
      mexErrMsgTxt("Length of k_f_p must be equal to m_num_joints");
    if (ff_qd.size() != m_num_joints)
      mexErrMsgTxt("Length of ff_qd must be equal to m_num_joints");
    if (ff_qd_d.size() != m_num_joints)
      mexErrMsgTxt("Length of ff_qd_d must be equal to m_num_joints");
    if (ff_f_d.size() != m_num_joints)
      mexErrMsgTxt("Length of ff_f_d must be equal to m_num_joints");
    if (ff_const.size() != m_num_joints)
      mexErrMsgTxt("Length of ff_const must be equal to m_num_joints");
    
    for (int i=0; i<m_num_joints; i++) {
      msg.k_q_p[drake_to_atlas_joint_map[i]] = k_q_p[i];
      msg.k_q_i[drake_to_atlas_joint_map[i]] = k_q_i[i];
      msg.k_qd_p[drake_to_atlas_joint_map[i]] = k_qd_p[i];
      msg.k_f_p[drake_to_atlas_joint_map[i]] = k_f_p[i];
      msg.ff_qd[drake_to_atlas_joint_map[i]] = ff_qd[i];
      msg.ff_qd_d[drake_to_atlas_joint_map[i]] = ff_qd_d[i];
      msg.ff_f_d[drake_to_atlas_joint_map[i]] = ff_f_d[i];
      msg.ff_const[drake_to_atlas_joint_map[i]] = ff_const[i];
    }
  }

  int dim(void) {
  	return 3*m_num_joints;
  }

  void publish(const string& channel, double t, const VectorXd& x) {

    msg.utime = (long)(t*1000000);
    int j;
    for (int i=0; i<m_num_joints; i++) {
      j = drake_to_atlas_joint_map[i];
      msg.position[j] = x(i);
      msg.velocity[j] = x(m_num_joints+i);
      msg.effort[j] = x(2*m_num_joints+i);
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
  if (nrhs==10 && mxGetNumberOfElements(prhs[0]) > 1) { // init
    
    vector<string> joint_names = get_strings(prhs[0]);
    int atlas_version_number = (int) mxGetScalar(prhs[1]);
    Map<VectorXd> k_q_p(mxGetPr(prhs[2]), mxGetNumberOfElements(prhs[2]));
    Map<VectorXd> k_q_i(mxGetPr(prhs[3]), mxGetNumberOfElements(prhs[3]));
    Map<VectorXd> k_qd_p(mxGetPr(prhs[4]), mxGetNumberOfElements(prhs[4]));
    Map<VectorXd> k_f_p(mxGetPr(prhs[5]), mxGetNumberOfElements(prhs[5]));
    Map<VectorXd> ff_qd(mxGetPr(prhs[6]), mxGetNumberOfElements(prhs[6]));
    Map<VectorXd> ff_qd_d(mxGetPr(prhs[7]), mxGetNumberOfElements(prhs[7]));
    Map<VectorXd> ff_f_d(mxGetPr(prhs[8]), mxGetNumberOfElements(prhs[8]));
    Map<VectorXd> ff_const(mxGetPr(prhs[9]), mxGetNumberOfElements(prhs[9]));
    
    AtlasCommand *ac = new AtlasCommand(joint_names,atlas_version_number,k_q_p,k_q_i,k_qd_p,k_f_p,ff_qd,ff_qd_d,ff_f_d,ff_const);
    mxClassID cid;
    if (sizeof(ac)==4) cid = mxUINT32_CLASS;
    else if (sizeof(ac)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:AtlasCommandPublisher:PointerSize error","Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&ac,sizeof(ac));

    return;
  }

  // retrieve object
  AtlasCommand *ac = NULL;
  if (nrhs==0 || !mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1) {
    mexErrMsgIdAndTxt("Drake:AtlasCommandPublisher:BadInputs","the first argument should be the mex_ptr");
  }
  memcpy(&ac,mxGetData(prhs[0]),sizeof(ac));

  if (nrhs==1) { // delete()
  	if (ac) delete(ac);
  } 
  else if (nrhs==4) { // publish()
    char* str = mxArrayToString(prhs[1]);
    string channel(str);
    mxFree(str);
    double t = mxGetScalar(prhs[2]);
    int n = mxGetNumberOfElements(prhs[3]);
    if (n != ac->dim()) mexErrMsgIdAndTxt("Drake:AtlasCommandPublisher:BadInputs","the dimension of x is wrong");
    Map<VectorXd> x(mxGetPr(prhs[3]), n);
    ac->publish(channel, t, x);
  }
  else if (nrhs==9  && nlhs==1) { // update gains
    Map<VectorXd> k_q_p(mxGetPr(prhs[1]), mxGetNumberOfElements(prhs[1]));
    Map<VectorXd> k_q_i(mxGetPr(prhs[2]), mxGetNumberOfElements(prhs[2]));
    Map<VectorXd> k_qd_p(mxGetPr(prhs[3]), mxGetNumberOfElements(prhs[3]));
    Map<VectorXd> k_f_p(mxGetPr(prhs[4]), mxGetNumberOfElements(prhs[4]));
    Map<VectorXd> ff_qd(mxGetPr(prhs[5]), mxGetNumberOfElements(prhs[5]));
    Map<VectorXd> ff_qd_d(mxGetPr(prhs[6]), mxGetNumberOfElements(prhs[6]));
    Map<VectorXd> ff_f_d(mxGetPr(prhs[7]), mxGetNumberOfElements(prhs[7]));
    Map<VectorXd> ff_const(mxGetPr(prhs[8]), mxGetNumberOfElements(prhs[8]));

    ac->updateGains(k_q_p,k_q_i,k_qd_p,k_f_p,ff_qd,ff_qd_d,ff_f_d,ff_const);

    mxClassID cid;
    if (sizeof(ac)==4) cid = mxUINT32_CLASS;
    else if (sizeof(ac)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:AtlasCommandPublisher:PointerSize error","Are you on a 32-bit machine or 64-bit machine??");
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&ac,sizeof(ac));
  }
  else {
    mexErrMsgTxt("AtlasCommandPublisher: wrong number of input arguments");
  }

}
