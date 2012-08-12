
import java.io.*;
import lcm.lcm.*;

public class SendTestStateMessage
{
    public static void main(String args[])
    {
	LCM lcm = LCM.getSingleton();
	
	drc.robot_state_t msg = new drc.robot_state_t();
	msg.timestamp = System.nanoTime();
	msg.robot_name = "Acrobot";

	msg.origin_position = new drc.position3D_t();
	msg.origin_position.translation = new drc.vector3_t();
	msg.origin_position.rotation = new drc.quaternion_t();
	msg.origin_position.rotation.w = 1.0;

	msg.origin_twist = new drc.twist_t();
	msg.origin_twist.linear_velocity = new drc.vector3_t();
	msg.origin_twist.angular_velocity = new drc.vector3_t();

	msg.origin_cov = new drc.covariance_t();

	msg.num_joints = 2;
	msg.joint_name = new String[] { "shoulder", "elbow" };
	msg.angular_position = new float[] { 0.0f, 0.1f };
	msg.angular_velocity = new float[] { 0.2f, 0.3f };
	msg.measured_torque = new float[] { 0.4f, 0.5f };
	
	msg.joint_cov = new drc.joint_covariance_t[2];
	msg.joint_cov[0] = new drc.joint_covariance_t();
	msg.joint_cov[1] = new drc.joint_covariance_t();
	msg.ground_contacts = new drc.ground_contact_state_t();
	msg.ground_contacts.num_contacts = 0;

	lcm.publish("true_robot_state", msg);
    }
}
