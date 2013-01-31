import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class FootstepPlanCoder implements drake.util.LCMCoder
{
	String m_robot_name;
	drc.ee_goal_sequence_t msg;

	public FootstepPlanCoder(String robot_name)
	{
		m_robot_name = robot_name;
		msg = new drc.ee_goal_sequence_t();
		msg.robot_name = robot_name;
	}

	public drake.util.CoordinateFrameData decode(byte[] data)
	{
		try {
			drc.ee_goal_sequence_t msg = new drc.ee_goal_sequence_t(data);
			if (msg.robot_name.equals(m_robot_name)) {
				drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
				fdata.val = new double[msg.num_goals * 9];
				fdata.t = (double) msg.utime / 1000000.0;
				for (int i = 0; i < msg.num_goals; i++) {
					fdata.val[i*9 + 0] = msg.goals[i].ee_goal_pos.translation.x;
					fdata.val[i*9 + 1] = msg.goals[i].ee_goal_pos.translation.y;
					fdata.val[i*9 + 2] = msg.goals[i].ee_goal_pos.translation.z;

					fdata.val[i*9 + 3] = msg.goals[i].ee_goal_pos.rotation.x;
					fdata.val[i*9 + 4] = msg.goals[i].ee_goal_pos.rotation.y;
					fdata.val[i*9 + 5] = msg.goals[i].ee_goal_pos.rotation.z;
					fdata.val[i*9 + 6] = msg.goals[i].ee_goal_pos.rotation.w;

					fdata.val[i*9 + 7] = (double)msg.goal_times[i] / 1000000.0;
					if (msg.goals[i].ee_name == "r_foot") {
						fdata.val[i*9 + 8] = 1;
					} else {
						fdata.val[i*9 + 8] = 0;
					}
				}
				return fdata;
			}
		} catch (IOException ex) {
			System.out.println("Exception: " + ex);
		}
		return null;
	}

	public LCMEncodable encode(drake.util.CoordinateFrameData d)
	{
		msg.utime = (long)(d.t*1000000);
		msg.num_goals = (int)d.val.length/9;
		msg.goals = new drc.ee_goal_t[msg.num_goals];
		msg.goal_times = new long[msg.num_goals];
		for (int i=0; i < msg.num_goals; i++) {
			msg.goals[i] = new drc.ee_goal_t();
			msg.goals[i].ee_goal_pos = new drc.position_3d_t();
			msg.goals[i].ee_goal_pos.translation = new drc.vector_3d_t();
			msg.goals[i].ee_goal_pos.rotation = new drc.quaternion_t();

			msg.goals[i].ee_goal_pos.translation.x = d.val[i*9 + 0];
			msg.goals[i].ee_goal_pos.translation.y = d.val[i*9 + 1];
			msg.goals[i].ee_goal_pos.translation.z = d.val[i*9 + 2];

			msg.goals[i].ee_goal_pos.rotation.x = d.val[i*9 + 3];
			msg.goals[i].ee_goal_pos.rotation.y = d.val[i*9 + 4];
			msg.goals[i].ee_goal_pos.rotation.z = d.val[i*9 + 5];
			msg.goals[i].ee_goal_pos.rotation.w = d.val[i*9 + 6];

			msg.goal_times[i] = (long)(d.val[i*9 + 7]*1000000);

			if (d.val[i*9 + 8] != 0) {
				msg.goals[i].ee_name = "r_foot";
			} else {
				msg.goals[i].ee_name = "l_foot";
			}
		}
		return msg;
	}

	public String timestampName()
	{
		return "utime";
	}
}



