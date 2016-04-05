package drc.control;

import java.io.*;
import java.lang.*;
import java.util.Arrays;
import lcm.lcm.*;

public class JointAnglesCoder implements drake.util.LCMCoder
{    
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_joint_map;
    int m_num_joints;
	
    bot_core.joint_angles_t msg;
    
    public JointAnglesCoder(String robot_name, String[] joint_name)
    {
      m_robot_name = robot_name;
      m_num_joints = joint_name.length;
      m_joint_map = new java.util.TreeMap<String,Integer>();
      
      for (int i=0; i<m_num_joints; i++) {
        m_joint_map.put(joint_name[i],i);
      }
      
      msg = new bot_core.joint_angles_t();
      msg.robot_name = robot_name;
      msg.num_joints = joint_name.length;
      msg.joint_name = joint_name;
      msg.joint_position = new double[m_num_joints];
    }

    public int dim()
    {
      return m_num_joints;
    }
    
    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        bot_core.joint_angles_t msg = new bot_core.joint_angles_t(data);
        if (msg.robot_name.equals(m_robot_name)) {
          Integer j;
          int index;
          
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[m_num_joints];
          fdata.t = (double)msg.utime / 1000000.0;
          for (int i=0; i<msg.num_joints; i++) {
            j = m_joint_map.get(msg.joint_name[i]);
            if (j!=null) {
              index = j.intValue();
              fdata.val[index] = msg.joint_position[i];
              if (fdata.val[index] > Math.PI)
                fdata.val[index] -= 2*Math.PI;
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
        msg.joint_position[i] = (float) d.val[i];
      }
      return msg;
    }
       
    public String timestampName()
    {
      return "utime";
    }

		public String[] coordinateNames() {
			String[] coords = new String[dim()];
			Arrays.fill(coords, "");
			return coords;
		}
}
