import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class RobotInputCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_actuator_map;
    int m_num_actuators;
    
    drc.actuator_cmd_t msg;

    public RobotInputCoder(String robot_name, String[] actuator_name)
    {
      m_num_actuators = actuator_name.length;
      m_robot_name = robot_name;
      m_actuator_map = new java.util.TreeMap<String,Integer>();
      
      for (int i=0; i<m_num_actuators; i++) {
        m_actuator_map.put(actuator_name[i],i);
      }
      
      msg = new drc.actuator_cmd_t();
      msg.robot_name = robot_name;
      msg.num_actuators = m_num_actuators;
      msg.actuator_name = actuator_name;
      msg.actuator_effort = new double[msg.num_actuators];
      msg.effort_duration = new double[msg.num_actuators];
      for (int i=0; i<msg.num_actuators; i++)
        msg.effort_duration[i] = 1.0; // could be changed
    }
    
    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.actuator_cmd_t msg = new drc.actuator_cmd_t(data);
        if (msg.robot_name.equals(m_robot_name)) {
          Integer j;
          int index;
          
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[m_num_actuators];
          fdata.t = (double)msg.utime / 1000000.0;
          for (int i=0; i<m_num_actuators; i++) {
            j = m_actuator_map.get(msg.actuator_effort[i]);
            if (j!=null) {
              index = j.intValue();
              fdata.val[index] = msg.actuator_effort[i];
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
      for (int i=0; i<m_num_actuators; i++) {
        msg.actuator_effort[i] = (float) d.val[i];
      }
      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
