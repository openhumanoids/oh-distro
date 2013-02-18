
import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class SandiaJointCommandCoder implements drake.util.LCMCoder 
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_drake_joint_map;
    java.util.TreeMap<Integer,String> m_rev_drake_joint_map; // should replace with a BiMap
    java.util.TreeMap<String,Integer> m_gazebo_joint_map;
    java.util.TreeMap<Integer,String> m_rev_gazebo_joint_map; // should replace with a BiMap
    
    // SandiaHandplugin has a fixed order for joints, 
    //        so number of joints here is fixed
    int m_num_joints = 12;

    int mode = 1; // mode==1: torque-only, mode==2: position-only, fixed gains
    // TODO: add additional modes (e.g., position w/variable gains, mixed torque-position control
    
    drc.joint_command_t msg;

    public SandiaJointCommandCoder(String robot_name,boolean floating, String side, String[] joint_name, double[] Kp, double[] Kd) throws Exception
    {
      this(robot_name,floating, side, joint_name);
      
      mode = 2;
      int j;
      for (int i=0; i<msg.num_joints; i++) {
        j = m_gazebo_joint_map.get(m_rev_drake_joint_map.get(i)).intValue();
        msg.kp_position[j] = Kp[i];
        msg.kd_position[j] = Kd[i];
      }

    }
    
    public SandiaJointCommandCoder(String robot_name,boolean floating, String side, String[] joint_name) throws Exception
    {
      if(floating)
      {
        m_num_joints = 18;
      }
    
      if (joint_name.length != m_num_joints)
        throw new Exception("Length of joint_name must be " + m_num_joints);
      
      m_robot_name = robot_name;
      m_drake_joint_map = new java.util.TreeMap<String,Integer>();
      m_rev_drake_joint_map = new java.util.TreeMap<Integer,String>();
      m_gazebo_joint_map = new java.util.TreeMap<String,Integer>();
      m_rev_gazebo_joint_map = new java.util.TreeMap<Integer,String>();
      
      String[] gazebo_joint_name = new String[m_num_joints];
      
      // name matches fixed order in SandiaHandPlugin.cpp
      gazebo_joint_name[0] = side+"_f0_j0";
      gazebo_joint_name[1] = side+"_f0_j1";
      gazebo_joint_name[2] = side+"_f0_j2";
      gazebo_joint_name[3] = side+"_f1_j0";
      gazebo_joint_name[4] = side+"_f1_j1";
      gazebo_joint_name[5] = side+"_f1_j2";
      gazebo_joint_name[6] = side+"_f2_j0";
      gazebo_joint_name[7] = side+"_f2_j1";
      gazebo_joint_name[8] = side+"_f2_j2";
      gazebo_joint_name[9] = side+"_f3_j0";
      gazebo_joint_name[10] = side+"_f3_j1";
      gazebo_joint_name[11] = side+"_f3_j2";
      if(floating){
        gazebo_joint_name[12] = side+"_base_x";
        gazebo_joint_name[13] = side+"_base_y";
        gazebo_joint_name[14] = side+"_base_z";
        gazebo_joint_name[15] = side+"_base_roll";
        gazebo_joint_name[16] = side+"_base_pitch";
        gazebo_joint_name[17] = side+"_base_yaw";
      }
      
      for (int i=0; i<m_num_joints; i++) {
        m_drake_joint_map.put(joint_name[i],i);
        m_rev_drake_joint_map.put(i,joint_name[i]);
        m_gazebo_joint_map.put(gazebo_joint_name[i],i);
        m_rev_gazebo_joint_map.put(i,gazebo_joint_name[i]);
      }
      
      msg = new drc.joint_command_t();
      msg.robot_name = robot_name;
      msg.num_joints = m_num_joints;
      msg.name = gazebo_joint_name; // name matches fixed order in SandiaHandPlugin.cpp
      
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
            j = m_drake_joint_map.get(m_rev_gazebo_joint_map.get(i));
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
        j = m_gazebo_joint_map.get(m_rev_drake_joint_map.get(i)).intValue();
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
