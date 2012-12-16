import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class RobotStateListener implements LCMSubscriber
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_joint_map;
    boolean m_has_new_message = false;
    int m_num_joints;
    double m_timestamp;
    double[] m_x;

    public RobotStateListener(String robot_name, String[] joint_name)
    {
        m_num_joints = joint_name.length;
        m_robot_name = robot_name;
        m_joint_map = new java.util.TreeMap<String,Integer>();

        for (int i=0; i<m_num_joints; i++) {
            m_joint_map.put(joint_name[i],i);
        }
        m_x = new double[2*m_num_joints];
        m_timestamp = 0;
    }

    public void subscribe(String channel)
    {
      LCM lcm = LCM.getSingleton();
      lcm.subscribe(channel,this);
    }
    
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream dins) 
    {
        int index;
        try { 
            drc.robot_state_t msg = new drc.robot_state_t(dins);
            if (msg.robot_name.equals(m_robot_name)) {
                m_timestamp = (double)msg.utime / 100000.0;
                for (int i=0; i<msg.num_joints; i++) {
                    Integer j = m_joint_map.get(msg.joint_name[i]);
                    if (j!=null) {
                        index = j.intValue();
                        m_x[index] = msg.joint_position[i];
                        m_x[index + m_num_joints] = msg.joint_velocity[i];
                    }
                }
                
                // get body position and orientation
                index = m_joint_map.get("base_x").intValue();
                m_x[index] = msg.origin_position.translation.x;
                m_x[index + m_num_joints] = msg.origin_twist.linear_velocity.x;
                index = m_joint_map.get("base_y").intValue();
                m_x[index] = msg.origin_position.translation.y;
                m_x[index + m_num_joints] = msg.origin_twist.linear_velocity.y;
                index = m_joint_map.get("base_z").intValue();
                m_x[index] = msg.origin_position.translation.z;
                m_x[index + m_num_joints] = msg.origin_twist.linear_velocity.z;
                
                // convert quaternion to euler
                double x = msg.origin_position.rotation.x;
                double y = msg.origin_position.rotation.y;
                double z = msg.origin_position.rotation.z;
                double w = msg.origin_position.rotation.w;

                index = m_joint_map.get("base_roll").intValue();
                m_x[index] = Math.atan2(2*(x*y + z*w),1-2*(y*y+z*z));
                m_x[index + m_num_joints] = msg.origin_twist.angular_velocity.x;

                index = m_joint_map.get("base_pitch").intValue();
                m_x[index] = Math.asin(2*(x*z - w*y));
                m_x[index + m_num_joints] = msg.origin_twist.angular_velocity.y;

                index = m_joint_map.get("base_yaw").intValue();
                m_x[index] = Math.atan2(2*(x*w + y*z),1-2*(z*z+w*w))+Math.PI;
                if (m_x[index] > Math.PI)
                   m_x[index] -= 2*Math.PI;
                m_x[index + m_num_joints] = msg.origin_twist.angular_velocity.z;
                
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

    public synchronized double getLastTimestamp()
    {
        return m_timestamp;
    }
    
    public synchronized double[] getState()
    {
        m_has_new_message = false;
        return m_x;
    }

}
