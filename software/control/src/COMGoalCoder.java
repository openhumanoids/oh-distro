import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class COMGoalCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    drc.com_goal_t msg;

    public COMGoalCoder(String robot_name)
    {
      m_robot_name = robot_name;
      
      msg = new drc.com_goal_t();
      msg.robot_name = robot_name;
      msg.com_goal = new drc.vector_3d_t();
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.com_goal_t msg = new drc.com_goal_t(data);
        if (msg.robot_name.equals(m_robot_name)) {
        
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[3];
          fdata.t = (double)msg.utime / 1000000.0;
 
          fdata.val[0] = msg.com_goal.x;
          fdata.val[1] = msg.com_goal.y;
          fdata.val[2] = msg.com_goal.z;

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
      msg.com_goal.x = (float) d.val[0];
      msg.com_goal.y = (float) d.val[1];
      msg.com_goal.z = (float) d.val[2];
      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
