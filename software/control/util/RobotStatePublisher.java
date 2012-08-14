import java.io.*;
import lcm.lcm.*;

public class RobotStatePublisher
{
    drc.robot_state_t msg;
    String channel_name;

    public RobotStatePublisher(String robot_name, String[] joint_name, String channel)
    {
	msg = new drc.robot_state_t();
	msg.robot_name = robot_name;

	msg.origin_position = new drc.position_3d_t();
	msg.origin_position.translation = new drc.vector_3d_t();
	msg.origin_position.rotation = new drc.quaternion_t();
	msg.origin_position.rotation.w = 1.0;

	msg.origin_twist = new drc.twist_t();
	msg.origin_twist.linear_velocity = new drc.vector_3d_t();
	msg.origin_twist.angular_velocity = new drc.vector_3d_t();

	msg.origin_cov = new drc.covariance_t();

	msg.num_joints = joint_name.length;
	msg.joint_name = joint_name;
	msg.joint_position = new float[msg.num_joints];
	msg.joint_velocity = new float[msg.num_joints];
	msg.measured_effort = new float[msg.num_joints];
	
	msg.joint_cov = new drc.joint_covariance_t[msg.num_joints];
	for (int i=0; i<msg.num_joints; i++) 
	    msg.joint_cov[i] = new drc.joint_covariance_t();

	msg.contacts = new drc.contact_state_t();
	msg.contacts.num_contacts = 0;

	channel_name = channel;
    }

    public void publish(double[] x) // use doubles here for compatibility w/ matlab
    {
	LCM lcm = LCM.getSingleton();
	msg.utime = System.nanoTime();  //must be in usec - sisir
	for (int i=0; i<msg.num_joints; i++) {
	    msg.joint_position[i] = (float) x[i];
	    msg.joint_velocity[i] = (float) x[i+msg.num_joints];
	}
	lcm.publish(channel_name,msg);
    }

}
