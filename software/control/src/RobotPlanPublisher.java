package drc.control;

import java.io.*;
import lcm.lcm.*;

public class RobotPlanPublisher
{
    drc.robot_plan_t msg;
    String channel_name;
    String[] nonfloating_joint_name;
    boolean has_floating_base;
    int nonfloating_joint_start_ndx;

    public RobotPlanPublisher(String[] joint_name,boolean floating_base, String channel)
    {
      msg = new drc.robot_plan_t();
      msg.robot_name = "atlas";
      channel_name = channel;
      has_floating_base = floating_base;
      
      nonfloating_joint_start_ndx = 0;
      //System.out.println(has_floating_base);
      if (has_floating_base)
        nonfloating_joint_start_ndx = 6;
      int num_nonfloating_joints = joint_name.length - nonfloating_joint_start_ndx;
      nonfloating_joint_name = new String[num_nonfloating_joints];
      for (int i=nonfloating_joint_start_ndx,j=0; i<joint_name.length; i++)
          nonfloating_joint_name[j++] = joint_name[i];
        // if (!(joint_name[i].startsWith("pelvis_"))) 
          // num_nonfloating_joints+=1;

      // for (int i=0,j=0; i<joint_name.length; i++)
      //   if (!(joint_name[i].startsWith("pelvis_"))) 
    }
    
    public RobotPlanPublisher(String[] joint_name,boolean has_floating_base, String channel, int num_states)
    {
      this(joint_name,has_floating_base,channel);
      allocate(num_states);
    }

    public void allocate(int num_states)
    {
      msg.num_states = num_states;

      msg.plan = new bot_core.robot_state_t[msg.num_states];
      for (int i=0;i<msg.num_states;i++)
      {
        msg.plan[i] = new bot_core.robot_state_t();
        msg.plan[i].num_joints = (short) nonfloating_joint_name.length;
        msg.plan[i].joint_name = nonfloating_joint_name;
        msg.plan[i].pose = new bot_core.position_3d_t();
        msg.plan[i].pose.translation = new bot_core.vector_3d_t();
        msg.plan[i].pose.rotation = new bot_core.quaternion_t();
        msg.plan[i].pose.rotation.w = 1.0;
        msg.plan[i].twist = new bot_core.twist_t();
        msg.plan[i].twist.linear_velocity = new bot_core.vector_3d_t();
        msg.plan[i].twist.angular_velocity = new bot_core.vector_3d_t();
        msg.plan[i].joint_position = new float[msg.plan[i].num_joints];
        msg.plan[i].joint_velocity = new float[msg.plan[i].num_joints];
        msg.plan[i].joint_effort = new float[msg.plan[i].num_joints];
        msg.plan[i].force_torque = new bot_core.force_torque_t();
      }
      msg.plan_info = new int[num_states];
    }
    
    public LCMEncodable encode(double[] t, double[][] x)
    {
      msg.utime = 0; //System.nanoTime()/1000;
      //System.out.format("Time is %d\n",msg.utime);
      
      for(int i = 0;i<msg.num_states;i++)
      {
        if (has_floating_base) {
          msg.plan[i].utime = (long) (t[i]*1000000+msg.utime);        
          msg.plan[i].pose.translation.x = (float) x[0][i];
          msg.plan[i].pose.translation.y = (float) x[1][i];
          msg.plan[i].pose.translation.z = (float) x[2][i];
          
          double[] rpy = new double[3];
          rpy[0] = x[3][i];
          rpy[1] = x[4][i];
          rpy[2] = x[5][i];
          double[] q = drake.util.Transform.rpy2quat(rpy);
          
          msg.plan[i].pose.rotation.w = (float) q[0];
          msg.plan[i].pose.rotation.x = (float) q[1];
          msg.plan[i].pose.rotation.y = (float) q[2];
          msg.plan[i].pose.rotation.z = (float) q[3];

          msg.plan[i].twist.linear_velocity.x = x[6+msg.plan[i].num_joints][i];
          msg.plan[i].twist.linear_velocity.y = x[6+msg.plan[i].num_joints+1][i];
          msg.plan[i].twist.linear_velocity.z = x[6+msg.plan[i].num_joints+2][i];
          msg.plan[i].twist.angular_velocity.x = x[6+msg.plan[i].num_joints+3][i];
          msg.plan[i].twist.angular_velocity.y = x[6+msg.plan[i].num_joints+4][i];
          msg.plan[i].twist.angular_velocity.z = x[6+msg.plan[i].num_joints+5][i];
        }
        
        //System.out.format("The %d's state is at time %d, origin's translation is %f, %f, %f, angles are %f,%f,%f; the velocity of origin is %f, %f, %f, %f, %f, %f\n",i+1,msg.plan[i].utime,msg.plan[i].pose.translation.x,msg.plan[i].pose.translation.y,msg.plan[i].pose.translation.z,msg.plan[i].pose.rotation.x,msg.plan[i].pose.rotation.y,msg.plan[i].pose.rotation.z,msg.plan[i].twist.linear_velocity.x,msg.plan[i].twist.linear_velocity.y,msg.plan[i].twist.linear_velocity.z,msg.plan[i].twist.angular_velocity.x,msg.plan[i].twist.angular_velocity.y,msg.plan[i].twist.angular_velocity.z);
        
        for(int j = 0;j<msg.plan[i].num_joints;j++)
        {
          msg.plan[i].joint_position[j] = (float) x[j+nonfloating_joint_start_ndx][i];
          msg.plan[i].joint_velocity[j] = (float) x[msg.plan[i].num_joints+j+nonfloating_joint_start_ndx][i];
          //System.out.format("The %d's joint of %d's state has joint angle %f, joint velocity %f\n",j+1,i+1,msg.plan[i].joint_position[j],msg.plan[i].joint_velocity[j]);
        }
      }
      return msg;
    }
    public void publish(double[] t,double[][] x)
    {
      if (t.length != msg.num_states) {
        allocate(t.length);  // reallocate on-demand
//        System.out.format("Error: Publish failed. RobotPlanPublisher was initialized with %d states, but trying to publish %d.\n",msg.num_states,t.length);
//        return;
      }
      if (x[0].length != t.length) {
        // todo: throw an exception here instead
        System.out.format("Error: x[0].length=%d and t.length=%d\n",x[0].length,t.length);
        return;
      }
      LCM lcm = LCM.getSingleton();
      lcm.publish(channel_name,encode(t,x));
    }
    
}
