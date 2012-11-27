import java.io.*;
import lcm.lcm.*;

public class RobotNavGoalListener implements LCMSubscriber
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_joint_map;
    boolean m_has_new_message = false;
    double[] nav_translation;
    double[] nav_goal;
    public RobotNavGoalListener(String robot_name, String channel)
    {
	m_robot_name = robot_name;

	LCM lcm = LCM.getSingleton();
	lcm.subscribe(channel,this);
    }

    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins) 
    {
	int index;
	try { 
	    drc.nav_goal_timed_t msg = new drc.nav_goal_timed_t(dins);
	    //if (msg.robot_name.equals(m_robot_name)) {
		nav_translation = new double[3];
		nav_translation[0] = msg.goal_pos.translation.x;
		nav_translation[1] = msg.goal_pos.translation.y;
		nav_translation[2] = msg.goal_pos.translation.z;
		
		double q1= msg.goal_pos.rotation.x;
		double q2 = msg.goal_pos.rotation.y;
		double q3 = msg.goal_pos.rotation.z;
		double q0 = msg.goal_pos.rotation.w;
		double roll = Math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
		double pitch = Math.asin(2*(q0*q2-q3*q1));
		double yaw = Math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
		nav_goal = new double[6];
		nav_goal[0] = nav_translation[0];
		nav_goal[1] = nav_translation[1];
		nav_goal[2] = nav_translation[2];
		nav_goal[3] = roll;
		nav_goal[4] = pitch;
		nav_goal[5] = yaw;
		//}
		m_has_new_message = true;
	} catch (IOException ex) {
	    System.out.println("Exception: " + ex);
	}
    }

    public synchronized double[] getNextMessage(long timeout_ms)
    {
	if (m_has_new_message) {
	    m_has_new_message = false;
	    return nav_goal;
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
                return nav_goal;
            }
        } catch (InterruptedException xcp) { }

        return null;
    }

    public synchronized double[] getNextMessage()
    {
	return getNextMessage(-1);
    }

    public synchronized double[] getState()
    {
	m_has_new_message = false;
	return nav_goal;
    }

}
