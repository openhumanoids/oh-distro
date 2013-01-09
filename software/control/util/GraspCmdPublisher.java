import java.io.*;
import lcm.lcm.*;

public class GraspCmdPublisher
{
    drc.grasp_cmd_t msg;
    String channel_name;

    public GraspCmdPublisher(String robot_name, String channel)
    {
        msg = new drc.grasp_cmd_t();
        msg.robot_name = robot_name;

        channel_name = channel;
    }

    public void publish(boolean close_hand, long utime) 
    {
        LCM lcm = LCM.getSingleton();
        msg.utime = utime;
        msg.close_hand = close_hand;
        lcm.publish(channel_name,msg);
    }
}
