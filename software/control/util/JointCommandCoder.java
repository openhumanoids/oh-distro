import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class JointCommandCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_joint_map;
    int m_num_joints;
    int mode = 1; // mode==1: torque-only, mode==2: position-only, fixed gains
    // TODO: add additional modes (e.g., position w/variable gains, mixed torque-position control
    
    drc.joint_command_t msg;

    public JointCommandCoder(String robot_name, String[] joint_name, double[] Kp, double[] Kd) {
    
      this(robot_name, joint_name);
      mode = 2;
      for (int i=0; i<msg.num_joints; i++) {
        msg.kp_position[i] = Kp[i];
        msg.kd_position[i] = Kd[i];
      }
    }
    
    public JointCommandCoder(String robot_name, String[] joint_name)
    {
      m_num_joints = joint_name.length;
      m_robot_name = robot_name;
      m_joint_map = new java.util.TreeMap<String,Integer>();
      
      for (int i=0; i<m_num_joints; i++) {
        m_joint_map.put(joint_name[i],i);
      }
  
      msg = new drc.joint_command_t();
      msg.robot_name = robot_name;
      msg.num_joints = m_num_joints;
      msg.name = joint_name;
      
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
            j = m_joint_map.get(msg.name[i]);
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
      for (int i=0; i<m_num_joints; i++) {
        if (mode==1)
          msg.effort[i] = d.val[i];
        else if (mode==2)
          msg.position[i] = d.val[i];
      }
      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
