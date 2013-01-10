import java.io.*;
import lcm.lcm.*;

public class PositionCmdPublisher
{
    drc.joint_angles_t msg;
    String channel_name;

    public PositionCmdPublisher(String robot_name, String[] joint_name, String channel)
    {
        msg = new drc.joint_angles_t();
        msg.robot_name = robot_name;

        msg.num_joints = joint_name.length;
        msg.joint_name = joint_name;

        channel_name = channel;
    }

    public void publish(double[] u, long utime) 
    {
        // TODO: generalize to subsets of joints
        LCM lcm = LCM.getSingleton();
        msg.utime = utime;
        msg.joint_position = u;
        lcm.publish(channel_name,msg);
    }
}
