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

	msg.num_actuators = joint_name.length;
	msg.actuator_name = joint_name;

	//	msg.joint_effort = new double[msg.num_joints];
	msg.effort_duration = new double[msg.num_actuators];
	for (int i=0; i<msg.num_actuators; i++)
	    msg.effort_duration[i] = 1.0;  

	channel_name = channel;
    }

    public void publish(double[] u) // use doubles here for compatibility w/ matlab
    {
	LCM lcm = LCM.getSingleton();
	msg.utime = System.nanoTime()/1000;
	msg.actuator_effort = u;
	lcm.publish(channel_name,msg);
    }

}
