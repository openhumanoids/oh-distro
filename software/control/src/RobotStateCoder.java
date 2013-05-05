import java.io.*;
import java.lang.*;
import lcm.lcm.*;

public class RobotStateCoder implements drake.util.LCMCoder
{
    String m_robot_name;
    int m_num_joints;
    int m_num_floating_joints;
    java.util.TreeMap<String,Integer> m_joint_map;
    java.util.TreeMap<String,Integer> m_floating_joint_map;
    drc.robot_state_t msg;

    public RobotStateCoder(String robot_name, String[] joint_name)
    {
      m_robot_name = robot_name;
      m_joint_map = new java.util.TreeMap<String,Integer>();
      m_floating_joint_map = new java.util.TreeMap<String,Integer>();
      
      m_num_joints = 0;
      m_num_floating_joints = 0;
      for (int i=0; i<joint_name.length; i++) {
        if (joint_name[i].startsWith("base_")) {  
          m_floating_joint_map.put(joint_name[i],i);
          m_num_floating_joints+=1;
        }
        else {
          m_joint_map.put(joint_name[i],i);
          m_num_joints+=1;
        }
      }     
      
      msg = new drc.robot_state_t();
      msg.robot_name = robot_name;

      msg.origin_position = new drc.position_3d_t();
      msg.origin_position.translation = new drc.vector_3d_t();
      msg.origin_position.rotation = new drc.quaternion_t();
      
      if (m_num_floating_joints == 0) {
        // Atlas specific stuff
        msg.origin_position.rotation.w = 1.0;
        msg.origin_position.translation.z = 0.927;
      }
      msg.origin_twist = new drc.twist_t();
      msg.origin_twist.linear_velocity = new drc.vector_3d_t();
      msg.origin_twist.angular_velocity = new drc.vector_3d_t();

      msg.origin_cov = new drc.covariance_t();

      msg.num_joints = m_num_joints;
      msg.joint_name = new String[m_num_joints];
      int j=0;
      for (int i=0; i<joint_name.length; i++) {
        if (!(joint_name[i].startsWith("base_")))  
          msg.joint_name[j++] = joint_name[i];
      }     
      msg.joint_position = new float[m_num_joints];
      msg.joint_velocity = new float[m_num_joints];
      msg.measured_effort = new float[m_num_joints];
	
      msg.joint_cov = new drc.joint_covariance_t[m_num_joints];
      for (int i=0; i<m_num_joints; i++)
        msg.joint_cov[i] = new drc.joint_covariance_t();

      msg.contacts = new drc.contact_state_t();
      msg.contacts.num_contacts = 0;
    }
    
    public int dim()
    {
      return 2*(m_num_joints+m_num_floating_joints);
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        drc.robot_state_t msg = new drc.robot_state_t(data);
        return decode(msg);
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public drake.util.CoordinateFrameData decode(drc.robot_state_t msg)
    {
      if (msg.robot_name.equals(m_robot_name)) {
        Integer j;
        int index;

        drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
        fdata.val = new double[2*(m_num_joints+m_num_floating_joints)];
        fdata.t = (double)msg.utime / 1000000.0;
        for (int i=0; i<msg.num_joints; i++) {
          j = m_joint_map.get(msg.joint_name[i]);
          if (j!=null) {
            index = j.intValue();
            fdata.val[index] = msg.joint_position[i];
            if (fdata.val[index] > Math.PI)
              fdata.val[index] -= 2*Math.PI;
            fdata.val[index+m_num_joints+m_num_floating_joints] = msg.joint_velocity[i];
          }
        }

        // get floating joint body position and orientation
        // TODO: should be generalized eventually
        j = m_floating_joint_map.get("base_x");
        if (j!=null) {
          index = j.intValue();
          fdata.val[index] = msg.origin_position.translation.x;
          fdata.val[index+m_num_joints+m_num_floating_joints] = msg.origin_twist.linear_velocity.x;
        }
        j = m_floating_joint_map.get("base_y");
        if (j!=null) {
          index = j.intValue();
          fdata.val[index] = msg.origin_position.translation.y;
          fdata.val[index+m_num_joints+m_num_floating_joints] = msg.origin_twist.linear_velocity.y;
        }
        j = m_floating_joint_map.get("base_z");
        if (j!=null) {
          index = j.intValue();
          fdata.val[index] = msg.origin_position.translation.z;
          fdata.val[index+m_num_joints+m_num_floating_joints] = msg.origin_twist.linear_velocity.z;
        }

        double[] q = new double[4];
        q[0] = msg.origin_position.rotation.w;
        q[1] = msg.origin_position.rotation.x;
        q[2] = msg.origin_position.rotation.y;
        q[3] = msg.origin_position.rotation.z;
        double[] rpy = drake.util.Transform.quat2rpy(q);

        j = m_floating_joint_map.get("base_roll");
        if (j!=null) {
          index = j.intValue();
          fdata.val[index] = rpy[0];
          if (fdata.val[index] > Math.PI)
            fdata.val[index] -= 2*Math.PI;
          fdata.val[index+m_num_joints+m_num_floating_joints] = msg.origin_twist.angular_velocity.x;
        }

        j = m_floating_joint_map.get("base_pitch");
        if (j!=null) {
          index = j.intValue();
          fdata.val[index] = rpy[1];
          if (fdata.val[index] > Math.PI)
            fdata.val[index] -= 2*Math.PI;
          fdata.val[index+m_num_joints+m_num_floating_joints] = msg.origin_twist.angular_velocity.y;
        }

        j = m_floating_joint_map.get("base_yaw");
        if (j!=null) {
          index = j.intValue();
          fdata.val[index] = rpy[2];
          if (fdata.val[index] > Math.PI)
            fdata.val[index] -= 2*Math.PI;
          fdata.val[index+m_num_joints+m_num_floating_joints] = msg.origin_twist.angular_velocity.z;
        }

        return fdata;
      }
      return null;
    }

    public LCMEncodable encode(drake.util.CoordinateFrameData d)
    {
      msg.utime = (long)(d.t*1000000);
      Integer j;
      int index;

      for (int i=0; i<m_num_joints; i++) {
        j = m_joint_map.get(msg.joint_name[i]);
        if (j!=null) {
          index = j.intValue();
          msg.joint_position[i] = (float) d.val[index];
          msg.joint_velocity[i] = (float) d.val[index+m_num_joints+m_num_floating_joints];
        }
      }

      if (m_num_floating_joints != 0) {
        // TODO: should be generalized eventually
        j = m_floating_joint_map.get("base_x");
        if (j!=null) {
          index = j.intValue();
          msg.origin_position.translation.x = (float) d.val[index];
          msg.origin_twist.linear_velocity.x = (float) d.val[index+m_num_joints+m_num_floating_joints];
        }
        j = m_floating_joint_map.get("base_y");
        if (j!=null) {
          index = j.intValue();
          msg.origin_position.translation.y = (float) d.val[index];
          msg.origin_twist.linear_velocity.y = (float) d.val[index+m_num_joints+m_num_floating_joints];
        }
        j = m_floating_joint_map.get("base_z");
        if (j!=null) {
          index = j.intValue();
          msg.origin_position.translation.z = (float) d.val[index];
          msg.origin_twist.linear_velocity.z = (float) d.val[index+m_num_joints+m_num_floating_joints];
        }

        double[] rpy = new double[3];
        index = m_floating_joint_map.get("base_roll").intValue();
        rpy[0] = (float) d.val[index];
        msg.origin_twist.angular_velocity.x = (float) d.val[index+m_num_joints+m_num_floating_joints];

        index = m_floating_joint_map.get("base_pitch").intValue();
        rpy[1] = (float) d.val[index];
        msg.origin_twist.angular_velocity.y = (float) d.val[index+m_num_joints+m_num_floating_joints]; 

        index = m_floating_joint_map.get("base_yaw").intValue();
        rpy[2] = (float) d.val[index];
        msg.origin_twist.angular_velocity.z = (float) d.val[index+m_num_joints+m_num_floating_joints];

        // covert rpy to quaternion 
        double[] q = drake.util.Transform.rpy2quat(rpy);

        msg.origin_position.rotation.w = (float) q[0];
        msg.origin_position.rotation.x = (float) q[1];
        msg.origin_position.rotation.y = (float) q[2];
        msg.origin_position.rotation.z = (float) q[3];
      }
      
      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
}
