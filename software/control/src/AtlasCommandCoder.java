import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class AtlasCommandCoder implements drake.util.LCMCoder 
{
    // Atlas plugin has a fixed order for joints, 
    //        so number of joints here is fixed
    final int m_num_joints = 28;
		int[] drake_to_atlas_joint_map;

    int mode = 1; // mode==1: torque-only, mode==2: position-only, fixed gains
    // TODO: add additional modes (e.g., position w/variable gains, mixed torque-position control
    
    drc.atlas_command_t msg;

    public AtlasCommandCoder(String[] joint_name, double[] Kp, double[] Kd) throws Exception
    {
      this(joint_name);
      
      mode = 2;
      int j;
      for (int i=0; i<msg.num_joints; i++) {
        j = drake_to_atlas_joint_map[i];
        msg.kp_position[j] = Kp[i];
        msg.kd_position[j] = Kd[i];
      }
    }
    
    public AtlasCommandCoder(String[] joint_name) throws Exception
    {
      if (joint_name.length != m_num_joints)
        throw new Exception("Length of joint_name must be " + m_num_joints);
      
			// fixed ordering assumed by drcsim interface
      String[] atlas_joint_name = new String[m_num_joints];
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

			drake_to_atlas_joint_map = new int[m_num_joints];
      
      for (int i=0; i<m_num_joints; i++) {
	      for (int j=0; j<m_num_joints; j++) {
					if (joint_name[i].equals(atlas_joint_name[j]))
						drake_to_atlas_joint_map[i]=j;
				}
			}

      msg = new drc.atlas_command_t();
      msg.num_joints = m_num_joints;
      
      msg.position = new double[msg.num_joints];
    	msg.velocity = new double[msg.num_joints];
      msg.effort = new double[msg.num_joints];

      msg.kp_position = new double[msg.num_joints];
    	msg.ki_position = new double[msg.num_joints];
      msg.kd_position = new double[msg.num_joints];
      msg.kp_velocity = new double[msg.num_joints];

    	msg.i_effort_min = new double[msg.num_joints];
      msg.i_effort_max = new double[msg.num_joints];
      
			msg.k_effort = new byte[msg.num_joints];
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
				msg.k_effort[i] = (byte)255; // take complete control of joints (remove BDI control)
      }
    }
    
    public int dim()
    {
      return m_num_joints;
    }
    
    public drake.util.CoordinateFrameData decode(byte[] data)
    {
			// don't need to go in this direction 
      return null;
    }
    
    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
      msg.utime = (long)(d.t*1000000);
      int j;
      for (int i=0; i<m_num_joints; i++) {
        j = drake_to_atlas_joint_map[i];
        if (mode==1)
          msg.effort[j] = d.val[i];
        else if (mode==2)
          msg.position[j] = d.val[i];
      }
      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
