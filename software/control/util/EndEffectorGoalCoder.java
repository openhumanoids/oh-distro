import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class EndEffectorGoalCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    String m_ee_name;
    drc.ee_goal_t msg;

    public EndEffectorGoalCoder(String robot_name, String end_effector_name)
    {
      	m_robot_name = robot_name;
      	m_ee_name = end_effector_name;
      
      	msg = new drc.ee_goal_t();
      	msg.robot_name = robot_name;
      	msg.ee_name = end_effector_name;

      	msg.ee_goal_pos = new drc.position_3d_t();
      	msg.ee_goal_pos.translation = new drc.vector_3d_t();
      	msg.ee_goal_pos.rotation = new drc.quaternion_t();
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      	try {
        	drc.ee_goal_t msg = new drc.ee_goal_t(data);
        	if (msg.robot_name.equals(m_robot_name) && msg.ee_name.equals(m_ee_name)) {
				drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
				fdata.val = new double[4];
				fdata.t = (double)msg.utime / 1000000.0;
				
				// set active/inactive bit
				if (msg.halt_ee_controller) {
					fdata.val[0] = 0;
				}
				else {
					fdata.val[0] = 1;
				}

				fdata.val[1] = msg.ee_goal_pos.translation.x;
				fdata.val[2] = msg.ee_goal_pos.translation.y;
				fdata.val[3] = msg.ee_goal_pos.translation.z;
	
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
		msg.halt_ee_controller = (d.val[0]==0);		
		msg.ee_goal_pos.translation.x = (float) d.val[1];
		msg.ee_goal_pos.translation.y = (float) d.val[2];
		msg.ee_goal_pos.translation.z = (float) d.val[3];
		
		return msg;
    }
    
    public String timestampName()
    {
 	    return "utime";
    }
}
