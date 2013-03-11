import java.io.*;
import lcm.lcm.*;

public class FootstepPlanPublisher
{
	drc.ee_goal_sequence_t msg;
	String m_channel_name;
	String m_left_foot_name, m_right_foot_name;

	public FootstepPlanPublisher(String robot_name, String right_foot_name, String left_foot_name, String channel)
	{
		msg = new drc.ee_goal_sequence_t();
		msg.robot_name = robot_name;
    msg.num_goals = 0;
		m_channel_name = channel;
    m_left_foot_name = left_foot_name;
    m_right_foot_name = right_foot_name;
	}

	public void allocate(int num_goals)
	{
		msg.num_goals = num_goals;
		msg.goals = new drc.ee_goal_t[num_goals];
		for (int i=0; i < num_goals; i++) {
			msg.goals[i] = new drc.ee_goal_t();
			msg.goals[i].robot_name = msg.robot_name;
      msg.goals[i].root_name = "";
			msg.goals[i].ee_goal_pos = new drc.position_3d_t();
			msg.goals[i].ee_goal_pos.translation = new drc.vector_3d_t();
			msg.goals[i].ee_goal_pos.rotation = new drc.quaternion_t();
      msg.goals[i].ee_goal_twist = new drc.twist_t();
      msg.goals[i].ee_goal_twist.linear_velocity = new drc.vector_3d_t();
      msg.goals[i].ee_goal_twist.angular_velocity = new drc.vector_3d_t();
      msg.goals[i].num_chain_joints = 0;
      msg.goals[i].use_posture_bias = false;
      msg.goals[i].halt_ee_controller = false;
		}
    msg.goal_times = new long[num_goals];
	}

	public void publish(double[] left_foot_times, double[][] left_foot_pose, double[] right_foot_times, double[][] right_foot_pose)
	{
		LCM lcm = LCM.getSingleton();
		msg.utime = 0; //System.nanoTime()/1000;

    if (left_foot_pose[0].length != left_foot_times.length) {
			System.out.format("Error: left vector length %d does not match time vector length %d\n", left_foot_pose[0].length, left_foot_times.length);
			return;
    }      
    if (right_foot_pose[0].length != right_foot_times.length) {
			System.out.format("Error: right vector length %d does not match time vector length %d\n", right_foot_pose[0].length, right_foot_times.length);
			return;
		}
    
		if ((left_foot_times.length + right_foot_times.length) != msg.num_goals) {
			allocate(left_foot_times.length + right_foot_times.length);
		}

    double x,y,z,roll,pitch,yaw; int offset = right_foot_times.length;
		for (int i=0; i < msg.num_goals; i++) {
      if (i<right_foot_times.length) {
        msg.goals[i].utime = (long) (right_foot_times[i]*1000000+msg.utime);
        msg.goals[i].ee_name = m_right_foot_name;
        x=right_foot_pose[0][i]; y=right_foot_pose[1][i]; z=right_foot_pose[2][i];
        roll=right_foot_pose[3][i]; pitch=right_foot_pose[4][i]; yaw=right_foot_pose[5][i];
      } else {
        msg.goals[i].utime = (long) (left_foot_times[i-offset]*1000000+msg.utime);
        msg.goals[i].ee_name = m_left_foot_name;
        x=left_foot_pose[0][i-offset]; y=left_foot_pose[1][i-offset]; z=left_foot_pose[2][i-offset];
        roll=left_foot_pose[3][i-offset]; pitch=left_foot_pose[4][i-offset]; yaw=left_foot_pose[5][i-offset];
      }

			msg.goals[i].ee_goal_pos.translation.x = (float) x;
			msg.goals[i].ee_goal_pos.translation.y = (float) y;
			msg.goals[i].ee_goal_pos.translation.z = (float) z;

			msg.goals[i].ee_goal_pos.rotation.x = Math.sin(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2)-Math.cos(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
			msg.goals[i].ee_goal_pos.rotation.y = Math.cos(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2)+Math.sin(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2);
			msg.goals[i].ee_goal_pos.rotation.z = Math.cos(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2)-Math.sin(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2);
			msg.goals[i].ee_goal_pos.rotation.w = Math.cos(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2)+Math.sin(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
		}

		lcm.publish(m_channel_name, msg);
	}
}
