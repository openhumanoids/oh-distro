package drc.control;

import java.io.*;
import java.lang.*;
import java.util.Arrays;
import lcm.lcm.*;

public class AtlasCommandCoder implements drake.util.LCMCoder 
{
  final int m_num_joints; // should match Atlas::NUM_JOINTS in BDI API
  int[] drake_to_atlas_joint_map;
  
  bot_core.atlas_command_t msg;
  
  public AtlasCommandCoder(String[] joint_names, int atlas_version_number, double[] k_q_p, double[] k_q_i,
    double[] k_qd_p, double[] k_f_p, double[] ff_qd, double[] ff_qd_d, double[] ff_f_d,
    double[] ff_const) throws Exception {

    switch (atlas_version_number) {
      case 3:
      case 4:
        m_num_joints = 28;
        break;
      case 5:
        m_num_joints = 30;
        break;
      default:
        throw new Exception("Invalid atlas_version_number: " + atlas_version_number);
    }
    if (joint_names.length != m_num_joints)
      throw new Exception("Length of joint_names must be " + m_num_joints);
    if (k_q_p.length != m_num_joints)
      throw new Exception("Length of k_q_p must be " + m_num_joints);
    if (k_q_i.length != m_num_joints)
      throw new Exception("Length of k_q_i must be " + m_num_joints);
    if (k_qd_p.length != m_num_joints)
      throw new Exception("Length of k_qd_p must be " + m_num_joints);
    if (k_f_p.length != m_num_joints)
      throw new Exception("Length of k_f_p must be " + m_num_joints);
    if (ff_qd.length != m_num_joints)
      throw new Exception("Length of ff_qd must be " + m_num_joints);
    if (ff_qd_d.length != m_num_joints)
      throw new Exception("Length of ff_qd_d must be " + m_num_joints);
    if (ff_f_d.length != m_num_joints)
      throw new Exception("Length of ff_f_d must be " + m_num_joints);
    if (ff_const.length != m_num_joints)
      throw new Exception("Length of ff_const must be " + m_num_joints);

    // fixed ordering assumed by drcsim interface AND atlas api 
    // see: AtlasControlTypes.h 
    String[] atlas_joint_names = new String[m_num_joints];
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
    }

    msg = new bot_core.atlas_command_t();
    msg.num_joints = m_num_joints;

    msg.joint_names = new String[msg.num_joints];

    msg.position = new double[msg.num_joints];
    msg.velocity = new double[msg.num_joints];
    msg.effort = new double[msg.num_joints];

    msg.k_q_p = new double[msg.num_joints];
    msg.k_q_i = new double[msg.num_joints];
    msg.k_qd_p = new double[msg.num_joints];
    msg.k_f_p = new double[msg.num_joints];
    msg.ff_qd = new double[msg.num_joints];
    msg.ff_qd_d = new double[msg.num_joints];
    msg.ff_f_d = new double[msg.num_joints];
    msg.ff_const = new double[msg.num_joints];

    msg.k_effort = new byte[msg.num_joints]; // only used in sim
    msg.desired_controller_period_ms = 3; // set desired controller rate (ms), only used in sim

    drake_to_atlas_joint_map = new int[m_num_joints];
    for (int i=0; i<m_num_joints; i++) {
      for (int j=0; j<m_num_joints; j++) {
        if (joint_names[i].equals(atlas_joint_names[j]))
          drake_to_atlas_joint_map[i]=j;
      }

      msg.position[i] = 0.0;
      msg.velocity[i] = 0.0;
      msg.effort[i] = 0.0;

      msg.joint_names[drake_to_atlas_joint_map[i]] = joint_names[i];
      
      k_q_p[drake_to_atlas_joint_map[i]] = k_q_p[i];
      msg.k_q_i[drake_to_atlas_joint_map[i]] = k_q_i[i];
      msg.k_qd_p[drake_to_atlas_joint_map[i]] = k_qd_p[i];
      msg.k_f_p[drake_to_atlas_joint_map[i]] = k_f_p[i];
      msg.ff_qd[drake_to_atlas_joint_map[i]] = ff_qd[i];
      msg.ff_qd_d[drake_to_atlas_joint_map[i]] = ff_qd_d[i];
      msg.ff_f_d[drake_to_atlas_joint_map[i]] = ff_f_d[i];
      msg.ff_const[drake_to_atlas_joint_map[i]] = ff_const[i];

      msg.k_effort[i] = (byte)255; // take complete control of joints (remove BDI control), sim only
    }
  }


  public void updateGains(double[] k_q_p, double[] k_q_i,double[] k_qd_p, double[] k_f_p, 
      double[] ff_qd, double[] ff_qd_d, double[] ff_f_d, double[] ff_const) throws Exception {

    if (k_q_p.length != m_num_joints)
      throw new Exception("Length of k_q_p must be " + m_num_joints);
    if (k_q_i.length != m_num_joints)
      throw new Exception("Length of k_q_i must be " + m_num_joints);
    if (k_qd_p.length != m_num_joints)
      throw new Exception("Length of k_qd_p must be " + m_num_joints);
    if (k_f_p.length != m_num_joints)
      throw new Exception("Length of k_f_p must be " + m_num_joints);
    if (ff_qd.length != m_num_joints)
      throw new Exception("Length of ff_qd must be " + m_num_joints);
    if (ff_qd_d.length != m_num_joints)
      throw new Exception("Length of ff_qd_d must be " + m_num_joints);
    if (ff_f_d.length != m_num_joints)
      throw new Exception("Length of ff_f_d must be " + m_num_joints);
    if (ff_const.length != m_num_joints)
      throw new Exception("Length of ff_const must be " + m_num_joints);
    
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

  public int dim() {
 		return 3*m_num_joints;
  }

  public drake.util.CoordinateFrameData decode(byte[] data) {
    try {
      bot_core.atlas_command_t msg = new bot_core.atlas_command_t(data);
      return decode(msg);
    } catch (IOException ex) {
      System.out.println("Exception: " + ex);
    }
    return null;
  }
  
  public drake.util.CoordinateFrameData decode(bot_core.atlas_command_t msg) {
    drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();

    fdata.val = new double[3*m_num_joints];
    fdata.t = (double)msg.utime / 1000000.0;

    int j;
    for (int i=0; i<m_num_joints; i++) {
      j = drake_to_atlas_joint_map[i];
      fdata.val[i] = msg.position[j];
      fdata.val[m_num_joints+i] = msg.velocity[j];
    	fdata.val[2*m_num_joints+i] = msg.effort[j];
    }
    return fdata;
  }

  public LCMEncodable encode(drake.util.CoordinateFrameData d) {
    msg.utime = (long)(d.t*1000000);
    int j;
    for (int i=0; i<m_num_joints; i++) {
      j = drake_to_atlas_joint_map[i];
    	msg.position[j] = d.val[i];
    	msg.velocity[j] = d.val[m_num_joints+i];
    	msg.effort[j] = d.val[2*m_num_joints+i];
    }        
    return msg;
  }

  public String timestampName() {
    return "utime";
  }
	
	public String[] coordinateNames() {
		String[] coords = new String[dim()];
		Arrays.fill(coords, "");
		return coords;
	}
}
