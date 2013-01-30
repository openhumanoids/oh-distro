import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class NavGoalCoder implements drake.util.LCMCoder
{
	String m_robot_name;
	drc.nav_goal_timed_t msg;

	public NavGoalCoder(String robot_name)
	{
		m_robot_name = robot_name;
		msg = new drc.nav_goal_timed_t();
		msg.robot_name = robot_name;
		msg.timeout = new Integer(0);
		msg.goal_pos = new drc.position_3d_t();
		msg.goal_pos.translation = new drc.vector_3d_t();
		msg.goal_pos.rotation = new drc.quaternion_t();
	}

	public drake.util.CoordinateFrameData decode(byte[] data)
	{
		try {
			drc.nav_goal_timed_t msg = new drc.nav_goal_timed_t(data);
			if (msg.robot_name.equals(m_robot_name))
			{
				drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
				fdata.val = new double[8];
				fdata.t = (double)msg.utime / 1000000.0;

				fdata.val[0] = msg.goal_pos.translation.x;
				fdata.val[1] = msg.goal_pos.translation.y;
				fdata.val[2] = msg.goal_pos.translation.z;

				fdata.val[3] = msg.goal_pos.rotation.x;
				fdata.val[4] = msg.goal_pos.rotation.y;
				fdata.val[5] = msg.goal_pos.rotation.z;
				fdata.val[6] = msg.goal_pos.rotation.w;

				fdata.val[7] = msg.timeout / 1000000.0; // fdata.val[7] is the goal timeout in ms

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
		msg.goal_pos.translation.x = (float) d.val[0];
		msg.goal_pos.translation.y = (float) d.val[1];
		msg.goal_pos.translation.z = (float) d.val[2];

		msg.goal_pos.rotation.x = (float) d.val[3];
		msg.goal_pos.rotation.y = (float) d.val[4];
		msg.goal_pos.rotation.z = (float) d.val[5];
		msg.goal_pos.rotation.w = (float) d.val[6];

		msg.timeout = (long)(d.val[7] * 1000000);

		return msg;
	}

	public String timestampName()
	{
		return "utime";
	}
}



