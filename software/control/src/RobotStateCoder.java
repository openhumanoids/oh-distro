package drc.control;

import java.io.*;
import java.lang.*;
import java.util.Arrays;
import lcm.lcm.*;

public class RobotStateCoder implements drake.util.LCMCoder
{
    short m_num_joints;
    int m_num_floating_joints;
    java.util.TreeMap<String,Integer> m_joint_map;
    java.util.TreeMap<String,Integer> m_floating_joint_map;
    bot_core.robot_state_t msg;
   
    boolean include_torques;
    
    public RobotStateCoder(String[] joint_name, boolean incl_torques) {
      this(joint_name);
      include_torques = incl_torques;
    }
    
    public RobotStateCoder(String[] joint_name) {
      m_joint_map = new java.util.TreeMap<String,Integer>();
      m_floating_joint_map = new java.util.TreeMap<String,Integer>();
      include_torques = false;
      
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
      
      msg = new bot_core.robot_state_t();
      
      msg.pose = new bot_core.position_3d_t();
      msg.pose.translation = new bot_core.vector_3d_t();
      msg.pose.rotation = new bot_core.quaternion_t();
      
      if (m_num_floating_joints == 0) {
        msg.pose.rotation.w = 1.0;
        msg.pose.translation.z = 0.927;
      }
      msg.twist = new bot_core.twist_t();
      msg.twist.linear_velocity = new bot_core.vector_3d_t();
      msg.twist.angular_velocity = new bot_core.vector_3d_t();

      msg.num_joints = m_num_joints;
      msg.joint_name = new String[m_num_joints];
      int j=0;
      for (int i=0; i<joint_name.length; i++) {
        if (!(joint_name[i].startsWith("base_")))  
          msg.joint_name[j++] = joint_name[i];
      }     
      msg.joint_position = new float[m_num_joints];
      msg.joint_velocity = new float[m_num_joints];
      msg.joint_effort = new float[m_num_joints];
    }
    
    public int dim()
    {
      if (include_torques)
        return 3*(m_num_joints+m_num_floating_joints);
      else
        return 2*(m_num_joints+m_num_floating_joints);
    }

    public drake.util.CoordinateFrameData decode(byte[] data)
    {
      try {
        bot_core.robot_state_t msg = new bot_core.robot_state_t(data);
        return decode(msg);
      } catch (IOException ex) {
        System.out.println("Exception: " + ex);
      }
      return null;
    }
 
    public drake.util.CoordinateFrameData decode(bot_core.robot_state_t msg)
    {
      Integer j;
      int index;

      drake.util.CoordinateFrameData fdata = new drake.util.CoordinateFrameData();
      fdata.val = new double[dim()];
      fdata.t = (double)msg.utime / 1000000.0;
      for (int i=0; i<msg.num_joints; i++) {
        j = m_joint_map.get(msg.joint_name[i]);
        if (j!=null) {
          index = j.intValue();
          fdata.val[index] = msg.joint_position[i];
          fdata.val[index+m_num_joints+m_num_floating_joints] = msg.joint_velocity[i];
          if (include_torques) 
            fdata.val[index+2*(m_num_joints+m_num_floating_joints)] = msg.joint_effort[i];
        }
      }

      // get floating joint body position and orientation
      // TODO: should be generalized eventually
      j = m_floating_joint_map.get("base_x");
      if (j!=null) {
        index = j.intValue();
        fdata.val[index] = msg.pose.translation.x;
        fdata.val[index+m_num_joints+m_num_floating_joints] = msg.twist.linear_velocity.x;
      }
      j = m_floating_joint_map.get("base_y");
      if (j!=null) {
        index = j.intValue();
        fdata.val[index] = msg.pose.translation.y;
        fdata.val[index+m_num_joints+m_num_floating_joints] = msg.twist.linear_velocity.y;
      }
      j = m_floating_joint_map.get("base_z");
      if (j!=null) {
        index = j.intValue();
        fdata.val[index] = msg.pose.translation.z;
        fdata.val[index+m_num_joints+m_num_floating_joints] = msg.twist.linear_velocity.z;
      }

      double[] q = new double[4];
      q[0] = msg.pose.rotation.w;
      q[1] = msg.pose.rotation.x;
      q[2] = msg.pose.rotation.y;
      q[3] = msg.pose.rotation.z;
      double[] rpy = drake.util.Transform.quat2rpy(q);

    	double[] omega = new double[3];
    	omega[0] = msg.twist.angular_velocity.x;
    	omega[1] = msg.twist.angular_velocity.y;
    	omega[2] = msg.twist.angular_velocity.z;
      double[] rpydot = drake.util.Transform.angularvel2rpydot(rpy,omega);

      j = m_floating_joint_map.get("base_roll");
      if (j!=null) {
        index = j.intValue();
        fdata.val[index] = rpy[0];
        fdata.val[index+m_num_joints+m_num_floating_joints] = rpydot[0];
      }

      j = m_floating_joint_map.get("base_pitch");
      if (j!=null) {
        index = j.intValue();
        fdata.val[index] = rpy[1];
        fdata.val[index+m_num_joints+m_num_floating_joints] = rpydot[1];
      }

      j = m_floating_joint_map.get("base_yaw");
      if (j!=null) {
        index = j.intValue();
        fdata.val[index] = rpy[2];
        fdata.val[index+m_num_joints+m_num_floating_joints] = rpydot[2];
      }

      return fdata;
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
          if (include_torques)
            msg.joint_effort[i] = (float) d.val[index+2*(m_num_joints+m_num_floating_joints)];
        }
      }

      if (m_num_floating_joints != 0) {
        // TODO: should be generalized eventually
        j = m_floating_joint_map.get("base_x");
        if (j!=null) {
          index = j.intValue();
          msg.pose.translation.x = (float) d.val[index];
          msg.twist.linear_velocity.x = (float) d.val[index+m_num_joints+m_num_floating_joints];
        }
        j = m_floating_joint_map.get("base_y");
        if (j!=null) {
          index = j.intValue();
          msg.pose.translation.y = (float) d.val[index];
          msg.twist.linear_velocity.y = (float) d.val[index+m_num_joints+m_num_floating_joints];
        }
        j = m_floating_joint_map.get("base_z");
        if (j!=null) {
          index = j.intValue();
          msg.pose.translation.z = (float) d.val[index];
          msg.twist.linear_velocity.z = (float) d.val[index+m_num_joints+m_num_floating_joints];
        }

        double[] rpy = new double[3];
      	double[] rpydot = new double[3];
        index = m_floating_joint_map.get("base_roll").intValue();
        rpy[0] = d.val[index];
        rpydot[0] = d.val[index+m_num_joints+m_num_floating_joints];

        index = m_floating_joint_map.get("base_pitch").intValue();
        rpy[1] = d.val[index];
        rpydot[1] = d.val[index+m_num_joints+m_num_floating_joints]; 

        index = m_floating_joint_map.get("base_yaw").intValue();
        rpy[2] = d.val[index];
        rpydot[2] = d.val[index+m_num_joints+m_num_floating_joints];

        // convert rpy to quaternion 
        double[] q = drake.util.Transform.rpy2quat(rpy);
        msg.pose.rotation.w = (float) q[0];
        msg.pose.rotation.x = (float) q[1];
        msg.pose.rotation.y = (float) q[2];
        msg.pose.rotation.z = (float) q[3];

        double[] omega = drake.util.Transform.rpydot2angularvel(rpy,rpydot);
      	msg.twist.angular_velocity.x = (float) omega[0];
      	msg.twist.angular_velocity.y = (float) omega[1];
      	msg.twist.angular_velocity.z = (float) omega[2];
      }
      
      msg.force_torque =  new  bot_core.force_torque_t();
      
      return msg;
    }
    
    public String timestampName()
    {
      return "utime";
    }
		
		public String[] coordinateNames() {
			String[] coords = new String[dim()];
			Arrays.fill(coords, "");
			return coords;
		}
}
