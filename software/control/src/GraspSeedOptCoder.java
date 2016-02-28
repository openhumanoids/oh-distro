package drc.control;

import java.io.*;
import java.lang.*;
import java.util.Arrays;
import lcm.lcm.*;

public class GraspSeedOptCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    String m_object_name;
    String m_geometry_name;
    drc.grasp_opt_control_t pmsg;

    public GraspSeedOptCoder(String robot_name)
    {
      //initialization
      m_robot_name = robot_name;
      m_object_name = " ";
      m_geometry_name =  " "; 
          
      pmsg = new drc.grasp_opt_control_t();
      
      // Identification
      pmsg.robot_name = m_robot_name;
      pmsg.object_name = m_object_name;
      pmsg.geometry_name = m_geometry_name;
      pmsg.unique_id = 0;//unique ID for optimization
      
      
      // Optimization control
      pmsg.drake_control = pmsg.NEW;
      pmsg.grasp_type = pmsg.SANDIA_LEFT;// SANDIA_LEFT by default (options: SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT_OR_RIGHT=3, IROBOT_BOTH=4; )
      pmsg.contact_mask = pmsg.ALL;
      
      
      // object information
      pmsg.geometry_type =pmsg.SPHERE;// SPHERE=0, CYLINDER=1, BOX=2, TORUS=4; 
      pmsg.num_dims = 0;
      pmsg.dims = new double[pmsg.num_dims];
      
      // initial hand pose
      pmsg.l_hand_init_pose = new bot_core.position_3d_t();
      pmsg.l_hand_init_pose.translation = new bot_core.vector_3d_t();
      pmsg.l_hand_init_pose.rotation = new bot_core.quaternion_t();
      pmsg.l_hand_init_pose.rotation.w = 1.0;
      
      pmsg.r_hand_init_pose = new bot_core.position_3d_t();
      pmsg.r_hand_init_pose.translation = new bot_core.vector_3d_t();
      pmsg.r_hand_init_pose.rotation = new bot_core.quaternion_t();
      pmsg.r_hand_init_pose.rotation.w = 1.0;

    }

    public int dim()
    {
      return -1;  // this class has a variable size (which technically shouldn't be allowed by an LCMCoder)
    }
    
    public drake.util.CoordinateFrameData decode(byte[] data)
    {
 
      try {
        drc.grasp_opt_control_t msg = new drc.grasp_opt_control_t(data);
        //if (msg.robot_name.equals(m_robot_name)) {
          m_object_name = msg.object_name;
          m_geometry_name =  msg.geometry_name; 
          Integer j;
          int index;
        
          // frame is encoded as
          // fdata.val = [unique_id;drake_control;grasp_type;contact_mask;
          //              l_hand_init_pose.translation;l_hand_init_pose.rotation;
          //              r_hand_init_pose.translation;r_hand_init_pose.rotation;
          //              geometry_type;num_dims;dims[1, ...,num_dims]];
          
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
         
          fdata.t = (double)msg.utime / 100000.0;
          fdata.val = new double[4+2*7+2+msg.num_dims]; 
         
          fdata.val[0] = (double)msg.unique_id;
          fdata.val[1] = (double)msg.drake_control;
          fdata.val[2] = (double)msg.grasp_type;
          fdata.val[3] = (double)msg.contact_mask;
          fdata.val[4] = msg.l_hand_init_pose.translation.x;
          fdata.val[5] = msg.l_hand_init_pose.translation.y;
          fdata.val[6] = msg.l_hand_init_pose.translation.z;
          fdata.val[7] = msg.l_hand_init_pose.rotation.x;
          fdata.val[8] = msg.l_hand_init_pose.rotation.y;
          fdata.val[9] = msg.l_hand_init_pose.rotation.z;
          fdata.val[10] = msg.l_hand_init_pose.rotation.w;
          fdata.val[11] = msg.r_hand_init_pose.translation.x;
          fdata.val[12] = msg.r_hand_init_pose.translation.y;
          fdata.val[13] = msg.r_hand_init_pose.translation.z;
          fdata.val[14] = msg.r_hand_init_pose.rotation.x;
          fdata.val[15] = msg.r_hand_init_pose.rotation.y;
          fdata.val[16] = msg.r_hand_init_pose.rotation.z;
          fdata.val[17] = msg.r_hand_init_pose.rotation.w;
          fdata.val[18] = (double)msg.geometry_type;
          fdata.val[19] = (double)msg.num_dims;
           int offset = 20;
          for (int i=0; i<msg.num_dims; i++) {
            fdata.val[offset + i] = msg.dims[i];
          }

          return fdata;
        //}
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
      pmsg.drake_control = (short) d.val[1];
      pmsg.grasp_type = (short) d.val[2];
      pmsg.contact_mask = (short) d.val[3];
      pmsg.l_hand_init_pose.translation.x = d.val[4];
      pmsg.l_hand_init_pose.translation.y = d.val[5];
      pmsg.l_hand_init_pose.translation.z = d.val[6];
      pmsg.l_hand_init_pose.rotation.x = d.val[7]; 
      pmsg.l_hand_init_pose.rotation.y = d.val[8];
      pmsg.l_hand_init_pose.rotation.z = d.val[9]; 
      pmsg.l_hand_init_pose.rotation.w = d.val[10];
      pmsg.r_hand_init_pose.translation.x = d.val[11];
      pmsg.r_hand_init_pose.translation.y = d.val[12];
      pmsg.r_hand_init_pose.translation.z = d.val[13];
      pmsg.r_hand_init_pose.rotation.x = d.val[14]; 
      pmsg.r_hand_init_pose.rotation.y = d.val[15];
      pmsg.r_hand_init_pose.rotation.z = d.val[16]; 
      pmsg.r_hand_init_pose.rotation.w = d.val[17];
      pmsg.geometry_type = (short) d.val[18];
      pmsg.num_dims = (int) d.val[19];

      int offset = 20;
      pmsg.dims = new double[pmsg.num_dims];
      for (int i=0; i<pmsg.num_dims; i++) {
        pmsg.dims[i] = d.val[i+offset];
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
		
		public String[] coordinateNames() {
			String[] coords = new String[dim()];
			Arrays.fill(coords, "");
			return coords;
		}
}
