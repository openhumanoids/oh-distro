import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class GraspCmdCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    drc.grasp_cmd_t msg;

    public GraspCmdCoder(String robot_name)
    {
      m_robot_name = robot_name;
      
      msg = new drc.grasp_cmd_t();
      msg.robot_name = robot_name;
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.grasp_cmd_t msg = new drc.grasp_cmd_t(data);
        if (msg.robot_name.equals(m_robot_name)) {
        
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[1];
          fdata.t = (double)msg.utime / 1000000.0;
          if (msg.close_hand) {
              fdata.val[0]=1;
          } else {
              fdata.val[0]=0;
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
      if ((float) d.val[0] > .5) {
          msg.close_hand = (boolean) true;
    } else {
        msg.close_hand = (boolean) false;
    }
      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
