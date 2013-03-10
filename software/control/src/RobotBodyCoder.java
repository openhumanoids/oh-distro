import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class RobotBodyCoder implements drake.util.LCMCoder
{    
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_body_map;
    int m_num_bodies;
	
    drc.robot_body_t msg;   
    
    public RobotBodyCoder(String robot_name, String[] body_name)
    {
      m_robot_name = robot_name;
      m_num_bodies = body_name.length;
      m_body_map = new java.util.TreeMap<String,Integer>();
      
      for (int i=0; i<m_num_bodies; i++) {
        m_body_map.put(body_name[i],i);
      }
      
      msg = new drc.robot_body_t();
      msg.robot_name = robot_name;
      msg.num_bodies = body_name.length;
      msg.body_name = body_name;
      msg.body_flag = new int[m_num_bodies];
    }
    
    public int dim()
    {
      return m_num_bodies;
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.robot_body_t msg = new drc.robot_body_t(data);
        if (msg.robot_name.equals(m_robot_name)) {
          Integer j;
          int index;
          
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[m_num_bodies];
          fdata.t = (double)msg.utime / 1000000.0;
          for (int i=0; i<msg.num_bodies; i++) {
            j = m_body_map.get(msg.body_name[i]);
            if (j!=null) {
              index = j.intValue();
              fdata.val[index] = msg.body_flag[i];
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
      for (int i=0; i<m_num_bodies; i++) {
        msg.body_flag[i] = (int) d.val[i];
      }
      return msg;
    }
       
    public String timestampName()
    {
      return "utime";
    }
}
