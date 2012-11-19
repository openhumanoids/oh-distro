import java.io.*;
import lcm.lcm.*;

public class RobotNavGoalListener implements LCMSubscriber
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_joint_map;
    boolean m_has_new_message = false;
    double[] nav_translation
    double[] nav_quaternion

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
	    drc.robot_nav_goal_timed_t msg = new drc.robot_nav_goal_timed_t(dins);
	    if (msg.robot_name.equals(m_robot_name)) {
		nav_position = new double[3];
		nav_position[0] = msg.goal_pos.translation.x;
		nav_position[1] = msg.goal_pos.translation.y;
		nav_position[2] = msg.goal_pos.translation.z;
		nav_quaternion = new double[4];
		nav_quaternion[0] = msg.goal_pos.rotation.x;
		nav_quaternion[1] = msg.goal_pos.rotation.y;
		nav_quaternion[2] = msg.goal_pos.rotation.z;
		nav_quaternion[3] = msg.goal_pos.rotation.w;
		}
		m_has_new_message = true;
	    }
	} catch (IOException ex) {
	    System.out.println("Exception: " + ex);
	}
    }

    public synchronized double[] getNextMessage(long timeout_ms)
    {
	if (m_has_new_message) {
	    m_has_new_message = false;
	    return m_x;
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
                return m_x;
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
	return m_x;
    }

}
