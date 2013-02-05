import java.io.*;
import lcm.lcm.*;

public class RobotPlanListener implements LCMSubscriber
{
    String m_robot_name;
    boolean m_has_new_message = false;
    boolean has_floating_base;
    int nonfloating_joint_start_ndx;
    java.util.TreeMap<String,Integer> m_joint_map;
    double[][] plan;
    
    public RobotPlanListener(String robot_name, String[] joint_name, boolean floating_base, String channel)
    {
      m_robot_name = robot_name;
      has_floating_base = floating_base;
      
      nonfloating_joint_start_ndx = 0;
      if (has_floating_base) {
        nonfloating_joint_start_ndx = 6;
      }
      
      int num_nonfloating_joints = joint_name.length - nonfloating_joint_start_ndx;
      m_joint_map = new java.util.TreeMap<String,Integer>();
      for (int i=nonfloating_joint_start_ndx; i<joint_name.length; i++) {
        m_joint_map.put(joint_name[i],i);
      }
 
      LCM lcm = LCM.getSingleton();
      lcm.subscribe(channel,this);
    }
    
    
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins) 
    {
      try {
        drc.robot_plan_t msg = new drc.robot_plan_t(dins);
        if (msg.robot_name.equals(m_robot_name)) {
          int numq = msg.plan[0].num_joints+nonfloating_joint_start_ndx;
          plan = new double[2*numq][msg.num_states];
          
          int index;
          for (int i=0;i<msg.num_states;i++) {
            
            if (nonfloating_joint_start_ndx == 6) {
              plan[0][i] = msg.plan[i].origin_position.translation.x;
              plan[1][i] = msg.plan[i].origin_position.translation.y;
              plan[2][i] = msg.plan[i].origin_position.translation.z;
              plan[numq+0][i] = msg.plan[i].origin_twist.linear_velocity.x;
              plan[numq+1][i] = msg.plan[i].origin_twist.linear_velocity.y;
              plan[numq+2][i] = msg.plan[i].origin_twist.linear_velocity.z;
      
              double[] q = new double[4];
              q[0] = msg.plan[i].origin_position.rotation.w;
              q[1] = msg.plan[i].origin_position.rotation.x;
              q[2] = msg.plan[i].origin_position.rotation.y;
              q[3] = msg.plan[i].origin_position.rotation.z;

              q = quatnormalize(q);
              double[] rpy = threeaxisrot(-2*(q[2]*q[3] - q[0]*q[1]), 
                                q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3], 
                                2*(q[1]*q[3] + q[0]*q[2]), -2.*(q[1]*q[2] - q[0]*q[3]),
                                q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
              
              plan[3][i] = rpy[0];
              plan[4][i] = rpy[1];
              plan[5][i] = rpy[2];
              
              plan[numq+3][i] = msg.plan[i].origin_twist.angular_velocity.x;
              plan[numq+4][i] = msg.plan[i].origin_twist.angular_velocity.y;
              plan[numq+5][i] = msg.plan[i].origin_twist.angular_velocity.z;
            }
            
            for (int j=0;j<msg.plan[i].num_joints;j++) {
              index = m_joint_map.get(msg.plan[i].joint_name[j]).intValue();
              plan[index][i] = msg.plan[i].joint_position[j];
              plan[index+numq][i] = msg.plan[i].joint_velocity[j];
            }
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
