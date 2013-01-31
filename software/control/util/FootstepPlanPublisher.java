import java.io.*;
import lcm.lcm.*;

public class FootstepPlanPublisher
{
	drc.ee_goal_sequence_t msg;
	String channel_name;
	String[2] actuator_names;

	public FootstepPlanPublisher(String robot_name, String[2] actuator_names, String channel)
	{
		msg = new drc.ee_goal_sequence_t();
		msg.robot_name = robot_name
		channel_name = channel;

	}

	public void allocate(int num_goals)
	{
		msg.num_goals = num_goals;
		msg.goals = new drc.ee_goal_t[msg.num_goals];
		for (int i=0; i < msg.num_goals; i++) {
			msg.goals[i] = new drc.ee_goal_t();
			msg.goals[i].robot_name = msg.robot_name;
			msg.goals[i].ee_goal_pos = new drc.position_3d_t();
			msg.goals[i].ee_goal_pos.translation = new drc.vector_3d_t();
			msg.goals[i].ee_goal_pos.rotation = new drc.quaternion_t();
		}
	}

	public void publish(double[] t, double[6][] x)
	{
		LCM lcm = LCM.getSingleton();
		msg.utime = System.nanoTime()/1000;

		if (t.length * 2 != msg.num_goals) {
			allocate(t.length * 2);
		}
		if (x.length != 2 * t.length) {
			System.out.format("Error: state vector length does not match time vector length (state vector should be twice as long as time vector)");
			return;
		}
		for (int i=0; i < x.length; i++) {
			if (i < t.length) {
				msg.goals[i].utime = (long) (t[i]*1000000+msg.utime);
				msg.goals[i].ee_name = "r_foot";
			} else {
				msg.goals[i].utime = (long) (t[i - t.length]*1000000 + msg.utime);
				msg.goals[i].ee_name = "l_foot";
			}

			msg.goals[i].ee_goal_pos.translation.x = (float) x[0][i];
			msg.goals[i].ee_goal_pos.translation.y = (float) x[1][i];
			msg.goals[i].ee_goal_pos.translation.z = (float) x[2][i];

			double roll = x[3][i];
			double pitch = x[4][i];
			double yaw = x[5][i];
			msg.goals[i].ee_goal_pos.rotation.x = Math.sin(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2)-Math.cos(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
			msg.goals[i].ee_goal_pos.rotation.y = Math.cos(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2)+Math.sin(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2);
			msg.goals[i].ee_goal_pos.rotation.z = Math.cos(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2)-Math.sin(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2);
			msg.goals[i].ee_goal_pos.rotation.w = Math.cos(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2)+Math.sin(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
		}

		lcm.publish(channel_name, msg);
	}
}
