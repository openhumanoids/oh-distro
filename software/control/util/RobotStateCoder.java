import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class RobotStateCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    java.util.TreeMap<String,Integer> m_joint_map;
    int m_num_joints;
    drc.robot_state_t pmsg;

    public RobotStateCoder(String robot_name, String[] joint_name)
    {
      m_num_joints = joint_name.length;
      m_robot_name = robot_name;
      m_joint_map = new java.util.TreeMap<String,Integer>();
      
      for (int i=0; i<m_num_joints; i++) {
        m_joint_map.put(joint_name[i],i);
      }
      
      pmsg = new drc.robot_state_t();
      pmsg.robot_name = robot_name;

      pmsg.origin_position = new drc.position_3d_t();
      pmsg.origin_position.translation = new drc.vector_3d_t();
      pmsg.origin_position.rotation = new drc.quaternion_t();
      pmsg.origin_position.rotation.w = 1.0;

      pmsg.origin_twist = new drc.twist_t();
      pmsg.origin_twist.linear_velocity = new drc.vector_3d_t();
      pmsg.origin_twist.angular_velocity = new drc.vector_3d_t();

      pmsg.origin_cov = new drc.covariance_t();

      int num_nonfloating_joints = 0;
      for (int i=0; i<joint_name.length; i++)
        if (!(joint_name[i].startsWith("base_"))) 
          num_nonfloating_joints+=1;
      String[] nonfloating_joint_name = new String[num_nonfloating_joints];
      int j=0;
      for (int i=0; i<joint_name.length; i++)
        if (!(joint_name[i].startsWith("base_"))) 
          nonfloating_joint_name[j++] = joint_name[i];
      pmsg.num_joints = num_nonfloating_joints;
      pmsg.joint_name = nonfloating_joint_name;
      pmsg.joint_position = new float[m_num_joints];
      pmsg.joint_velocity = new float[m_num_joints];
      pmsg.measured_effort = new float[m_num_joints];
	
      pmsg.joint_cov = new drc.joint_covariance_t[m_num_joints];
      for (int i=0; i<m_num_joints; i++)
        pmsg.joint_cov[i] = new drc.joint_covariance_t();

      pmsg.contacts = new drc.contact_state_t();
      pmsg.contacts.num_contacts = 0;
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.robot_state_t msg = new drc.robot_state_t(data);
        if (msg.robot_name.equals(m_robot_name)) {
          Integer j;
          int index;
          
          drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
          fdata.val = new double[2*m_num_joints];
          fdata.t = (double)msg.utime / 1000000.0;
          for (int i=0; i<msg.num_joints; i++) {
            j = m_joint_map.get(msg.joint_name[i]);
            if (j!=null) {
              index = j.intValue();
              fdata.val[index] = msg.joint_position[i];
              if (fdata.val[index] > Math.PI)
                fdata.val[index] -= 2*Math.PI;
              fdata.val[index + m_num_joints] = msg.joint_velocity[i];
            }
          }
          
          // get body position and orientation
          j = m_joint_map.get("base_x");
          if (j!=null) {
            index = j.intValue();
            fdata.val[index] = msg.origin_position.translation.x;
            fdata.val[index + m_num_joints] = msg.origin_twist.linear_velocity.x;
          }
          j = m_joint_map.get("base_y");
          if (j!=null) {
            index = j.intValue();
            fdata.val[index] = msg.origin_position.translation.y;
            fdata.val[index + m_num_joints] = msg.origin_twist.linear_velocity.y;
          }
          j = m_joint_map.get("base_z");
          if (j!=null) {
            index = j.intValue();
            fdata.val[index] = msg.origin_position.translation.z;
            fdata.val[index + m_num_joints] = msg.origin_twist.linear_velocity.z;
          }
          
          // convert quaternion to euler
          double x = msg.origin_position.rotation.x;
          double y = msg.origin_position.rotation.y;
          double z = msg.origin_position.rotation.z;
          double w = msg.origin_position.rotation.w;
          
          j = m_joint_map.get("base_roll");
          if (j!=null) {
            index = j.intValue();
            fdata.val[index] = Math.atan2(2*(x*y + z*w),1-2*(y*y+z*z));
            if (fdata.val[index] > Math.PI)
              fdata.val[index] -= 2*Math.PI;
            fdata.val[index + m_num_joints] = msg.origin_twist.angular_velocity.x;
          }
          
          j = m_joint_map.get("base_pitch");
          if (j!=null) {
            index = j.intValue();
            fdata.val[index] = -Math.asin(2*(x*z - w*y));
            if (fdata.val[index] > Math.PI)
              fdata.val[index] -= 2*Math.PI;
            fdata.val[index + m_num_joints] = msg.origin_twist.angular_velocity.y;
          }
          
          j = m_joint_map.get("base_yaw");
          if (j!=null) {
            index = j.intValue();
            fdata.val[index] = Math.atan2(2*(x*w + y*z),1-2*(z*z+w*w))+Math.PI;
            if (fdata.val[index] > Math.PI)
              fdata.val[index] -= 2*Math.PI;
            fdata.val[index + m_num_joints] = msg.origin_twist.angular_velocity.z;
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
      pmsg.utime = (long)(d.t*1000000);
      for (int i=0; i<m_num_joints; i++) {
        pmsg.joint_position[i] = (float) d.val[i];
        pmsg.joint_velocity[i] = (float) d.val[i+m_num_joints];
      }
      return pmsg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
