
import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class NavGoalCoder implements drake.util.LCMCoder
{    
    String m_robot_name;
    
    public NavGoalCoder(String robot_name)
    {
      m_robot_name = robot_name;
    }
    
    public int dim()
    {
      return 7;
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.nav_goal_timed_t msg = new drc.nav_goal_timed_t(data);
//        if (msg.robot_name.equals(m_robot_name)) {
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[7];
          fdata.t = (double)msg.utime / 1000000.0;
          
          double[] q = new double[4];
          q[0] = msg.goal_pos.rotation.w;
          q[1] = msg.goal_pos.rotation.x;
          q[2] = msg.goal_pos.rotation.y;
          q[3] = msg.goal_pos.rotation.z;
          double[] rpy = drake.util.Transform.quat2rpy(q);

          fdata.val[0] = msg.goal_pos.translation.x;
          fdata.val[1] = msg.goal_pos.translation.y;
          fdata.val[2] = msg.goal_pos.translation.z;
          fdata.val[3] = rpy[0];
          fdata.val[4] = rpy[1];
          fdata.val[5] = rpy[2];
          fdata.val[6] = msg.timeout / 1000000.0;
          return fdata;
//        }
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
    
    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
      // warning: not tested yet
      drc.nav_goal_timed_t msg = new drc.nav_goal_timed_t();
      
      msg.utime = (long)(d.t*1000000);
      
      msg.goal_pos = new drc.position_3d_t();
      msg.goal_pos.translation = new drc.vector_3d_t();
      msg.goal_pos.rotation = new drc.quaternion_t();
      
      msg.goal_pos.translation.x = (float) d.val[0];
      msg.goal_pos.translation.y = (float) d.val[1];
      msg.goal_pos.translation.z = (float) d.val[2];
      
      double[] rpy = new double[3];
      rpy[0] = d.val[3];
      rpy[1] = d.val[4];
      rpy[2] = d.val[5];
      double[] q = drake.util.Transform.rpy2quat(rpy);

      msg.goal_pos.rotation.w = (float) q[0];
      msg.goal_pos.rotation.x = (float) q[1];
      msg.goal_pos.rotation.y = (float) q[2];
      msg.goal_pos.rotation.z = (float) q[3];
      msg.timeout = (long)(d.val[6] * 1000000);
      
      return msg;
    }
       
    public String timestampName()
    {
      return "utime";
    }
}
