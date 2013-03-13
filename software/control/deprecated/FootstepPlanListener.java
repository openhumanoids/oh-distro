import java.io.*;
import lcm.lcm.*;

public class FootstepPlanListener implements LCMSubscriber
{
  String m_robot_name;
	String m_channel_name;
	boolean m_has_new_message = false;
  double[][] plan;

  public FootstepPlanListener(String robot_name, String channel)
  {
    m_robot_name = robot_name;
    m_channel_name = channel;
    
    LCM lcm = LCM.getSingleton();
    lcm.subscribe(channel,this);
  }

  public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins) 
  {
    try {
      drc.ee_goal_sequence_t msg = new drc.ee_goal_sequence_t(dins);
      if (msg.robot_name.equals(m_robot_name)) {
        // plan columns are: left/right bit, t, xyz, rpy 
        plan = new double[8][msg.num_goals];

        for (int i=0;i<msg.num_goals;i++) {
          
          if (msg.goals[i].ee_name.equals("r_foot")) {
            plan[0][i] = 1;
          }  
          else { //if (msg.goals[i].ee_name.equals("l_foot")) {
            plan[0][i] = 0;
          }
          
          plan[1][i] = msg.goal_times[i];
          plan[2][i] = msg.goals[i].ee_goal_pos.translation.x;
          plan[3][i] = msg.goals[i].ee_goal_pos.translation.y;
          plan[4][i] = msg.goals[i].ee_goal_pos.translation.z;
          
          // convert quaternion to euler
          // note: drake uses XYZ convention
          double[] q = new double[4];
          q[0] = msg.goals[i].ee_goal_pos.rotation.w;
          q[1] = msg.goals[i].ee_goal_pos.rotation.x;
          q[2] = msg.goals[i].ee_goal_pos.rotation.y;
          q[3] = msg.goals[i].ee_goal_pos.rotation.z;

          q = quatnormalize(q);
          double[] rpy = threeaxisrot(-2*(q[2]*q[3] - q[0]*q[1]), 
                      q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3], 
                      2*(q[1]*q[3] + q[0]*q[2]), -2.*(q[1]*q[2] - q[0]*q[3]),
                      q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
          
          plan[5][i] = rpy[0];
          plan[6][i] = rpy[1];
          plan[7][i] = rpy[2];
        }

        m_has_new_message = true;
      }
    } catch (IOException ex) {
      System.out.println("Exception: " + ex);
    }
  }

  public synchronized double[][] getNextMessage(long timeout_ms)
  {
    if (m_has_new_message) {
      m_has_new_message = false;
      return plan;
    }

    if(timeout_ms == 0)
      return null;

    try {
      if(timeout_ms > 0)
        wait(timeout_ms);
      else
        wait();

      if (m_has_new_message) {
        m_has_new_message = false;
        return plan;
      }
    } catch (InterruptedException xcp) { }

    return null;
  }

  public synchronized double[][] getNextMessage()
  {
    return getNextMessage(-1);
  }

  public synchronized double[][] getState()
  {
    m_has_new_message = false;
    return plan;
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
}
