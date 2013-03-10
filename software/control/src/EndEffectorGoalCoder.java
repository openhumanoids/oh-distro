import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class EndEffectorGoalCoder implements drake.util.LCMCoder
{
  String m_robot_name;
  String m_ee_name;
  drc.ee_goal_t msg;

  public EndEffectorGoalCoder(String robot_name, String end_effector_name)
  {
    m_robot_name = robot_name;
    m_ee_name = end_effector_name;

    msg = new drc.ee_goal_t();
    msg.robot_name = robot_name;
    msg.ee_name = end_effector_name;

    msg.ee_goal_pos = new drc.position_3d_t();
    msg.ee_goal_pos.translation = new drc.vector_3d_t();
    msg.ee_goal_pos.rotation = new drc.quaternion_t();
  }

  public int dim()
  {
    return 7;
  }

  public drake.util.CoordinateFrameData decode(byte[] data)
  {
    try {
      drc.ee_goal_t msg = new drc.ee_goal_t(data);
      if (msg.robot_name.equals(m_robot_name) && msg.ee_name.equals(m_ee_name)) {
        drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
				fdata.val = new double[7];
				fdata.t = (double)msg.utime / 1000000.0;
				
				// set active/inactive bit
				if (msg.halt_ee_controller) {
					fdata.val[0] = 0;
				}
				else {
					fdata.val[0] = 1;
				}

				fdata.val[1] = msg.ee_goal_pos.translation.x;
				fdata.val[2] = msg.ee_goal_pos.translation.y;
				fdata.val[3] = msg.ee_goal_pos.translation.z;
	      
        // convert quaternion to euler
        // note: drake uses XYZ convention
        double[] q = new double[4];
        q[0] = msg.ee_goal_pos.rotation.w;
        q[1] = msg.ee_goal_pos.rotation.x;
        q[2] = msg.ee_goal_pos.rotation.y;
        q[3] = msg.ee_goal_pos.rotation.z;

        q = quatnormalize(q);
        double[] rpy = threeaxisrot(-2*(q[2]*q[3] - q[0]*q[1]), 
                      q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3], 
                      2*(q[1]*q[3] + q[0]*q[2]), -2.*(q[1]*q[2] - q[0]*q[3]),
                      q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
          
        fdata.val[4] = rpy[0];
        fdata.val[5] = rpy[1];
        fdata.val[6] = rpy[2];
          
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
    msg.halt_ee_controller = (d.val[0]==0);		
    msg.ee_goal_pos.translation.x = (float) d.val[1];
    msg.ee_goal_pos.translation.y = (float) d.val[2];
    msg.ee_goal_pos.translation.z = (float) d.val[3];
    
    float roll = (float) d.val[4];
    float pitch = (float) d.val[5];
    float yaw = (float) d.val[6];

    // covert rpy to quaternion 
    // note: drake uses XYZ convention
    double w = Math.cos(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2) - Math.sin(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
    double x = Math.cos(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2) + Math.sin(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2);
    double y = Math.cos(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2) - Math.sin(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2);
    double z = Math.cos(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2) + Math.sin(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2);

    msg.ee_goal_pos.rotation.x = (float) x;
    msg.ee_goal_pos.rotation.y = (float) y;
    msg.ee_goal_pos.rotation.z = (float) z;
    msg.ee_goal_pos.rotation.w = (float) w;
		
    return msg;
  }
      
  private double[] threeaxisrot(double r11, double r12, double r21, double r31, double r32) { 
    // find angles for rotations about X, Y, and Z axes
    double[] r = new double[3];
    r[0] = Math.atan2(r11, r12);
    r[1] = Math.asin(r21);
    r[2] = Math.atan2(r31, r32);
    return r;
  }
    
  private double[] quatnormalize(double[] q) {
    double norm = Math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    double[] qout = new double[4];
    for (int i=0; i<4; i++)
      qout[i] = q[i]/norm;
    return qout;
  }
    
  public String timestampName()
  {
    return "utime";
  }
}
