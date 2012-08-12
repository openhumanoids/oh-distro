import java.io.*;
import lcm.lcm.*;

public class ActuatorCmdPublisher
{
    drc.actuator_cmd_t msg;
    String channel_name;

    public ActuatorCmdPublisher(String robot_name, String[] joint_name, String channel)
    {
	msg = new drc.actuator_cmd_t();
	msg.robot_name = robot_name;

	msg.num_joints = joint_name.length;
	msg.joint_name = joint_name;

	//	msg.joint_effort = new double[msg.num_joints];
	msg.duration = new double[msg.num_joints];
	for (int i=0; i<msg.num_joints; i++)
	    msg.duration[i] = 1.0;  

	channel_name = channel;
    }

    public void publish(double[] u) // use doubles here for compatibility w/ matlab
    {
	LCM lcm = LCM.getSingleton();
	msg.timestamp = System.nanoTime();
	msg.joint_effort = u;
	lcm.publish(channel_name,msg);
    }

}
