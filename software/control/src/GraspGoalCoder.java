import java.io.*;
import java.lang.*;
import lcm.lcm.*;
import static java.lang.System.*;

public class GraspGoalCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    String m_ee_name;
    int m_num_joints;
    java.util.TreeMap<String,Integer> m_joint_map;
    drc.ee_goal_t msg;

    public GraspGoalCoder(String robot_name, String end_effector_name, String[] joint_name)
    {
      	m_robot_name = robot_name;
      	m_ee_name = end_effector_name;
        m_num_joints = joint_name.length;
        //System.out.println(m_num_joints);
        
        msg = new drc.ee_goal_t();
      	msg.robot_name = robot_name;
      	msg.ee_name = end_effector_name;

      	msg.ee_goal_pos = new drc.position_3d_t();
      	msg.ee_goal_pos.translation = new drc.vector_3d_t();
      	msg.ee_goal_pos.rotation = new drc.quaternion_t();
        
        m_joint_map = new java.util.TreeMap<String,Integer>();
        
      
        for (int i=0; i<m_num_joints; i++) {
                m_joint_map.put(joint_name[i],i);
        }
        
        //System.out.println(m_joint_map);
        msg.num_chain_joints = joint_name.length;
        msg.chain_joint_names = joint_name;
        msg.joint_posture_bias = new double[m_num_joints];
        //System.out.println(msg);
    }

    public int dim()
    {
      return 3+4+m_num_joints;
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      	try {
        	drc.ee_goal_t msg = new drc.ee_goal_t(data);
        	if (msg.robot_name.equals(m_robot_name) && msg.ee_name.equals(m_ee_name)) {
				drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
                int num = 3+4+m_num_joints;
                //System.out.println(num);
                //out.println(num);
				fdata.val = new double[num];
				fdata.t = (double)msg.utime / 1000000.0;
				
				// set active/inactive bit
				//if (msg.halt_ee_controller) {
				//	fdata.val[0] = 0;
				//}
				//else {
				//	fdata.val[0] = 1;
				//}

				fdata.val[0] = msg.ee_goal_pos.translation.x;
				fdata.val[1] = msg.ee_goal_pos.translation.y;
				fdata.val[2] = msg.ee_goal_pos.translation.z;
                fdata.val[3] = msg.ee_goal_pos.rotation.x;
                fdata.val[4] = msg.ee_goal_pos.rotation.y;
                fdata.val[5] = msg.ee_goal_pos.rotation.z;
                fdata.val[6] = msg.ee_goal_pos.rotation.w;
                
                Integer j;
                int index;

                for (int i=7; i<num; i++) {
                  j = m_joint_map.get(msg.chain_joint_names[i-7]);
                  if (j!=null) {
                    index = j.intValue();
                    fdata.val[index+7] = msg.joint_posture_bias[i-7];
                  if (fdata.val[index+7] > Math.PI)
                    fdata.val[index+7] -= 2*Math.PI;
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
		//msg.halt_ee_controller = (d.val[0]==0);		
		msg.ee_goal_pos.translation.x = (float) d.val[0];
		msg.ee_goal_pos.translation.y = (float) d.val[1];
		msg.ee_goal_pos.translation.z = (float) d.val[2];
        msg.ee_goal_pos.rotation.x = (float) d.val[3];
        msg.ee_goal_pos.rotation.y = (float) d.val[4];
        msg.ee_goal_pos.rotation.z = (float) d.val[5];
        msg.ee_goal_pos.rotation.w = (float) d.val[6];
        
        for (int i=0; i<m_num_joints; i++) {
          msg.joint_posture_bias[i] = (float) d.val[i+7];
        }
       
		return msg;
    }
    
    public String timestampName()
    {
 	    return "utime";
    }
}
