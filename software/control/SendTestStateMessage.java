
import java.io.*;
import lcm.lcm.*;

public class SendTestStateMessage
{
    public static void main(String args[])
    {
	LCM lcm = LCM.getSingleton();
	
	drc.robot_state_t msg = new drc.robot_state_t();
	msg.utime = System.nanoTime(); //must be in usec - sisir
	msg.robot_name = "Acrobot";

	msg.origin_position = new drc.position_3d_t();
	msg.origin_position.translation = new drc.vector_3d_t();
	msg.origin_position.rotation = new drc.quaternion_t();
	msg.origin_position.rotation.w = 1.0;

	msg.origin_twist = new drc.twist_t();
	msg.origin_twist.linear_velocity = new drc.vector_3d_t();
	msg.origin_twist.angular_velocity = new drc.vector_3d_t();

	msg.origin_cov = new drc.covariance_t();

	msg.num_joints = 2;
	msg.joint_name = new String[] { "shoulder", "elbow" };
	msg.joint_position = new float[] { 0.0f, 0.1f };
	msg.joint_velocity = new float[] { 0.2f, 0.3f };
	msg.measured_effort = new float[] { 0.4f, 0.5f };
	
	msg.joint_cov = new drc.joint_covariance_t[2];
	msg.joint_cov[0] = new drc.joint_covariance_t();
	msg.joint_cov[1] = new drc.joint_covariance_t();
	msg.contacts = new drc.contact_state_t();
	msg.contacts.num_contacts = 0;

	lcm.publish("true_robot_state", msg);
    }
}
