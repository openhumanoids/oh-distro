
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
      return 6;
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.nav_goal_timed_t msg = new drc.nav_goal_timed_t(data);
//        if (msg.robot_name.equals(m_robot_name)) {
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[6];
          fdata.t = (double)msg.utime / 1000000.0;
          
          double q1= msg.goal_pos.rotation.x;
          double q2 = msg.goal_pos.rotation.y;
          double q3 = msg.goal_pos.rotation.z;
          double q0 = msg.goal_pos.rotation.w;
          double roll = Math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
          double pitch = Math.asin(2*(q0*q2-q3*q1));
          double yaw = Math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
          fdata.val[0] = msg.goal_pos.translation.x;
          fdata.val[1] = msg.goal_pos.translation.y;
          fdata.val[2] = msg.goal_pos.translation.z;
          fdata.val[3] = roll;
          fdata.val[4] = pitch;
          fdata.val[5] = yaw;
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
      
      // covert rpy to quaternion (drake uses XYZ convention)
      double roll = d.val[3], pitch = d.val[4], yaw = d.val[5];
      double w = Math.cos(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2) - Math.sin(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
      double x = Math.cos(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2) + Math.sin(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2);
      double y = Math.cos(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2) - Math.sin(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2);
      double z = Math.cos(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2) + Math.sin(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2);

      msg.goal_pos.rotation.x = (float) x;
      msg.goal_pos.rotation.y = (float) y;
      msg.goal_pos.rotation.z = (float) z;
      msg.goal_pos.rotation.w = (float) w;
      
      return msg;
    }
       
    public String timestampName()
    {
      return "utime";
    }
}
