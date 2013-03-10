import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class GraspStateCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    String m_object_name;
    String m_geometry_name;
    drc.desired_grasp_state_t pmsg;
        
    java.util.TreeMap<String,Integer> m_l_joints_map; // maps are required to make the encoding and decoding order insensitive of the joint_name data struc.
    java.util.TreeMap<String,Integer> m_r_joints_map; // maps are required to make the encoding and decoding order insensitive of the joint_name data struc.
    int m_num_l_joints;
    int m_num_r_joints;


    public GraspStateCoder(String robot_name, String[] l_joint_name, String[] r_joint_name)
    {
      m_num_l_joints = l_joint_name.length;
      m_num_r_joints = r_joint_name.length;
      
      m_robot_name = robot_name;
      m_object_name = " ";
      m_geometry_name =  " "; 
          
      pmsg = new drc.desired_grasp_state_t();
      pmsg.robot_name = m_robot_name;
      pmsg.object_name = m_object_name;
      pmsg.geometry_name = m_geometry_name;
      
      pmsg.grasp_type = pmsg.SANDIA_LEFT;// SANDIA_LEFT by default (options: SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT_OR_RIGHT=3, IROBOT_BOTH=4; )
      pmsg.l_hand_pose = new drc.position_3d_t();
      pmsg.l_hand_pose.translation = new drc.vector_3d_t();
      pmsg.l_hand_pose.rotation = new drc.quaternion_t();
      pmsg.l_hand_pose.rotation.w = 1.0;
      pmsg.r_hand_pose = new drc.position_3d_t();
      pmsg.r_hand_pose.translation = new drc.vector_3d_t();
      pmsg.r_hand_pose.rotation = new drc.quaternion_t();
      pmsg.r_hand_pose.rotation.w = 1.0;
      
      m_l_joints_map = new java.util.TreeMap<String,Integer>();
      m_r_joints_map = new java.util.TreeMap<String,Integer>();
      
      pmsg.l_joint_name = l_joint_name;
      pmsg.r_joint_name = r_joint_name;
      pmsg.num_l_joints = m_num_l_joints;
      pmsg.num_r_joints = m_num_r_joints;
      pmsg.l_joint_position = new double[pmsg.num_l_joints];
      pmsg.r_joint_position = new double[pmsg.num_r_joints];

	    for (int i=0; i<pmsg.num_l_joints; i++)
	    {
	      pmsg.l_joint_position[i] = 0.0;  
	      // also add them to the map
        m_l_joints_map.put(l_joint_name[i],i);
      }
      
      for (int i=0; i<pmsg.num_r_joints; i++)
	    {
	      pmsg.r_joint_position[i] = 0.0;  
	      // also add them to the map
        m_r_joints_map.put(r_joint_name[i],i);
      }
      
      pmsg.optimization_status = pmsg.SUCCESS; // is this field necessary?
    }

    public int dim()
    {
      return 18+m_num_l_joints+m_num_r_joints;
    }
    
    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {

        drc.desired_grasp_state_t msg = new drc.desired_grasp_state_t(data);
        //if (msg.robot_name.equals(m_robot_name)) {
          m_robot_name =  msg.robot_name;
          m_object_name =  msg.object_name;
          m_geometry_name =  msg.geometry_name;
        
          Integer j;
          int index;
  
          // frame is encoded as
          // fdata.val = [unique_id;grasp_type;
          //              l_hand_pose.translation;l_hand_pose.rotation;
          //              r_hand_pose.translation;r_hand_pose.rotation;
          //              num_l_joints;num_r_joints;
          //              l_joint_position[1, ...,num_l_joints]
          //              r_joint_position[1, ...,num_r_joints]];
          
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.t = (double)msg.utime / 100000.0;
          fdata.val = new double[2+2*7+2+m_num_l_joints+m_num_r_joints]; 
          
          fdata.val[0] = (double)msg.unique_id;
          fdata.val[1] = (double)msg.grasp_type;
          fdata.val[2] = msg.l_hand_pose.translation.x;
          fdata.val[3] = msg.l_hand_pose.translation.y;
          fdata.val[4] = msg.l_hand_pose.translation.z;
          fdata.val[5] = msg.l_hand_pose.rotation.x;
          fdata.val[6] = msg.l_hand_pose.rotation.y;
          fdata.val[7] = msg.l_hand_pose.rotation.z;
          fdata.val[8] = msg.l_hand_pose.rotation.w;
          fdata.val[9] = msg.r_hand_pose.translation.x;
          fdata.val[10] = msg.r_hand_pose.translation.y;
          fdata.val[11] = msg.r_hand_pose.translation.z;
          fdata.val[12] = msg.r_hand_pose.rotation.x;
          fdata.val[13] = msg.r_hand_pose.rotation.y;
          fdata.val[14] = msg.r_hand_pose.rotation.z;
          fdata.val[15] = msg.r_hand_pose.rotation.w;
          fdata.val[16] = (double)msg.num_l_joints;
          fdata.val[17] = (double)msg.num_r_joints;
          
          int offset = 18;
          for (int i=0; i<msg.num_l_joints; i++) {
            j = m_l_joints_map.get(msg.l_joint_name[i]);
            if (j!=null) {
              index = offset+j.intValue();
              fdata.val[index] = msg.l_joint_position[i];
              if (fdata.val[index] > Math.PI)
                fdata.val[index] -= 2*Math.PI;
            }
          }
          
          //what if m_num_l_joints ! = msg.num_l_joints?
          offset = 18+m_num_l_joints;
          for (int i=0; i<msg.num_r_joints; i++) {
            j = m_r_joints_map.get(msg.r_joint_name[i]);
            if (j!=null) {
              index = offset+j.intValue();
              fdata.val[index] = msg.r_joint_position[i];
              if (fdata.val[index] > Math.PI)
                fdata.val[index] -= 2*Math.PI;
            }
          }          
       
          return fdata;
       // }
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {

      pmsg.utime = (long)(d.t*100000);
      pmsg.robot_name = m_robot_name;
      pmsg.object_name = m_object_name;
      pmsg.geometry_name = m_geometry_name;
      pmsg.unique_id = (int) d.val[0];
      pmsg.grasp_type = (short) d.val[1];
      pmsg.l_hand_pose.translation.x = d.val[2];
      pmsg.l_hand_pose.translation.y = d.val[3];
      pmsg.l_hand_pose.translation.z = d.val[4];
      pmsg.l_hand_pose.rotation.x = d.val[5]; 
      pmsg.l_hand_pose.rotation.y = d.val[6];
      pmsg.l_hand_pose.rotation.z = d.val[7]; 
      pmsg.l_hand_pose.rotation.w = d.val[8];
      pmsg.r_hand_pose.translation.x = d.val[9];
      pmsg.r_hand_pose.translation.y = d.val[10];
      pmsg.r_hand_pose.translation.z = d.val[11];
      pmsg.r_hand_pose.rotation.x = d.val[12]; 
      pmsg.r_hand_pose.rotation.y = d.val[13];
      pmsg.r_hand_pose.rotation.z = d.val[14]; 
      pmsg.r_hand_pose.rotation.w = d.val[15];
      pmsg.num_l_joints = (int) d.val[16];
      pmsg.num_r_joints = (int) d.val[17];
      
      Integer j;
      int index;
      
      int offset = 18;
      for (int i=0; i<m_num_l_joints; i++) {
        j = m_l_joints_map.get(pmsg.l_joint_name[i]);
        if (j!=null) {
          index = offset+j.intValue();
          pmsg.l_joint_position[i] = d.val[index];
        }
      }
      offset = 18+m_num_l_joints;
      for (int i=0; i<m_num_r_joints; i++) {
        j = m_r_joints_map.get(pmsg.r_joint_name[i]);
        if (j!=null) {
          index = offset+j.intValue();
          pmsg.r_joint_position[i] = d.val[index];
        }
      }  
          
      return pmsg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
    public String getObjectName()
    {
      return m_object_name;
    }
    
    public String getGeometryName()
    {
      return m_geometry_name;
    }
}
