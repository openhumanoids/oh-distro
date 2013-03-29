import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class JointCommandCoderWIntegralGains implements drake.util.LCMCoder 
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_drake_joint_map;
    java.util.TreeMap<Integer,String> m_rev_drake_joint_map; // should replace with a BiMap
    java.util.TreeMap<String,Integer> m_atlas_joint_map;
    java.util.TreeMap<Integer,String> m_rev_atlas_joint_map; // should replace with a BiMap
    
    // Atlas plugin has a fixed order for joints, 
    //        so number of joints here is fixed
    int m_num_joints = 28;

    int mode = 1; // mode==1: torque-only, mode==2: position-only, fixed gains
    // TODO: add additional modes (e.g., position w/variable gains, mixed torque-position control
    
    drc.joint_command_t msg;

    public JointCommandCoderWIntegralGains(String robot_name, String[] joint_name, double[] Kp, double[] Kd, double[] Ki) throws Exception
    {
      this(robot_name, joint_name);
      
      mode = 2;
      int j;
      for (int i=0; i<msg.num_joints; i++) {
        j = m_atlas_joint_map.get(m_rev_drake_joint_map.get(i)).intValue();
        msg.kp_position[j] = Kp[i];
        msg.kd_position[j] = Kd[i];
        msg.ki_position[j] = Ki[i];
      }
    }
    
    public JointCommandCoderWIntegralGains(String robot_name, String[] joint_name) throws Exception
    {
      if (joint_name.length != m_num_joints)
        throw new Exception("Length of joint_name must be " + m_num_joints);
      
      m_robot_name = robot_name;
      m_drake_joint_map = new java.util.TreeMap<String,Integer>();
      m_rev_drake_joint_map = new java.util.TreeMap<Integer,String>();
      m_atlas_joint_map = new java.util.TreeMap<String,Integer>();
      m_rev_atlas_joint_map = new java.util.TreeMap<Integer,String>();
      
      String[] atlas_joint_name = new String[28];
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
      
      for (int i=0; i<m_num_joints; i++) {
        m_drake_joint_map.put(joint_name[i],i);
        m_rev_drake_joint_map.put(i,joint_name[i]);
        m_atlas_joint_map.put(atlas_joint_name[i],i);
        m_rev_atlas_joint_map.put(i,atlas_joint_name[i]);
      }
      
      msg = new drc.joint_command_t();
      msg.robot_name = robot_name;
      msg.num_joints = m_num_joints;
      msg.name = atlas_joint_name; // name matches order in AtlasPlugin.cpp
      
      msg.position = new double[msg.num_joints];
    	msg.velocity = new double[msg.num_joints];
      msg.effort = new double[msg.num_joints];

      msg.kp_position = new double[msg.num_joints];
    	msg.ki_position = new double[msg.num_joints];
      msg.kd_position = new double[msg.num_joints];
      msg.kp_velocity = new double[msg.num_joints];

    	msg.i_effort_min = new double[msg.num_joints];
      msg.i_effort_max = new double[msg.num_joints];
      
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
      }
    }
    
    public int dim()
    {
      return m_num_joints;
    }
    
    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.joint_command_t msg = new drc.joint_command_t(data);
        if (msg.robot_name.equals(m_robot_name)) {
          Integer j;
          int index;
          
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[m_num_joints];
          fdata.t = (double)msg.utime / 1000000.0;
          for (int i=0; i<m_num_joints; i++) {
            j = m_drake_joint_map.get(m_rev_atlas_joint_map.get(i));
            if (j!=null) {
              index = j.intValue();
              if (mode==1)
                fdata.val[index] = msg.effort[i];
              else if (mode==2)
                fdata.val[index] = msg.position[i];
            }
          }
          return fdata;
        }
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
    
    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
      msg.utime = (long)(d.t*1000000);
      int j;
      for (int i=0; i<m_num_joints; i++) {
        j = m_atlas_joint_map.get(m_rev_drake_joint_map.get(i)).intValue();
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
