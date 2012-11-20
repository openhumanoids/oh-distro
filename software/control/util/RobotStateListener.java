import java.io.*;
import lcm.lcm.*;

public class RobotStateListener implements LCMSubscriber
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_joint_map;
    boolean m_has_new_message = false;
    int m_num_joints;
    long m_timestamp;
    double[] m_x;

    public RobotStateListener(String robot_name, String[] joint_name, String channel)
    {
        m_num_joints = joint_name.length;
        m_robot_name = robot_name;
        m_joint_map = new java.util.TreeMap<String,Integer>();

        for (int i=0; i<m_num_joints; i++) {
            m_joint_map.put(joint_name[i],i);
        }
        m_x = new double[2*m_num_joints];
        m_timestamp = 0;
        
        LCM lcm = LCM.getSingleton();
        lcm.subscribe(channel,this);
    }

    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins) 
    {
        int index;
        try { 
            drc.robot_state_t msg = new drc.robot_state_t(dins);
            if (msg.robot_name.equals(m_robot_name)) {
                m_timestamp = msg.utime;
                for (int i=0; i<msg.num_joints; i++) {
                    Integer j = m_joint_map.get(msg.joint_name[i]);
                    if (j!=null) {
                        index = j.intValue();
                        m_x[index] = msg.joint_position[i];
                        m_x[index + m_num_joints] = msg.joint_velocity[i];
                    }
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

    public synchronized long getLastTimestamp()
    {
        return m_timestamp;
    }
    
    public synchronized double[] getState()
    {
        m_has_new_message = false;
        return m_x;
    }

}
