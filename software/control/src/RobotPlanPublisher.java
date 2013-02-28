import java.io.*;
import lcm.lcm.*;

public class RobotPlanPublisher
{
    drc.robot_plan_t msg;
    String channel_name;
    String[] nonfloating_joint_name;
    boolean has_floating_base;
    int nonfloating_joint_start_ndx;

    public RobotPlanPublisher(String robot_name,String[] joint_name,boolean floating_base, String channel)
    {
      msg = new drc.robot_plan_t();
      msg.robot_name = robot_name;
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
    
    public RobotPlanPublisher(String robot_name, String[] joint_name,boolean has_floating_base, String channel, int num_states)
    {
      this(robot_name,joint_name,has_floating_base,channel);
      allocate(num_states);
    }

    public void allocate(int num_states)
    {
      msg.num_states = num_states;

      msg.plan = new drc.robot_state_t[msg.num_states];
      for (int i=0;i<msg.num_states;i++)
      {
        msg.plan[i] = new drc.robot_state_t();
        msg.plan[i].robot_name = msg.robot_name;
        msg.plan[i].num_joints = nonfloating_joint_name.length;
        msg.plan[i].joint_name = nonfloating_joint_name;
        msg.plan[i].origin_position = new drc.position_3d_t();
        msg.plan[i].origin_position.translation = new drc.vector_3d_t();
        msg.plan[i].origin_position.rotation = new drc.quaternion_t();
        msg.plan[i].origin_position.rotation.w = 1.0;
        msg.plan[i].origin_twist = new drc.twist_t();
        msg.plan[i].origin_twist.linear_velocity = new drc.vector_3d_t();
        msg.plan[i].origin_twist.angular_velocity = new drc.vector_3d_t();
        msg.plan[i].origin_cov = new drc.covariance_t();
        msg.plan[i].joint_position = new float[msg.plan[i].num_joints];
        msg.plan[i].joint_velocity = new float[msg.plan[i].num_joints];
        msg.plan[i].measured_effort = new float[msg.plan[i].num_joints];
        
        msg.plan[i].joint_cov = new drc.joint_covariance_t[msg.plan[i].num_joints];
        for(int j = 0;j<msg.plan[i].num_joints;j++)
        {
          msg.plan[i].joint_cov[j] = new drc.joint_covariance_t();
        }
        
        msg.plan[i].contacts = new drc.contact_state_t();
        msg.plan[i].contacts.num_contacts = 0;
      }
    }
    
    public void publish(double[] t,double[][] x)
    {
      LCM lcm = LCM.getSingleton();
      msg.utime = 0; //System.nanoTime()/1000;
      //System.out.format("Time is %d\n",msg.utime);
      
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
      for(int i = 0;i<msg.num_states;i++)
      {
        if (has_floating_base) {
          msg.plan[i].utime = (long) (t[i]*1000000+msg.utime);        
          msg.plan[i].origin_position.translation.x = (float) x[0][i];
          msg.plan[i].origin_position.translation.y = (float) x[1][i];
          msg.plan[i].origin_position.translation.z = (float) x[2][i];
          
          double roll = x[3][i];
          double pitch = x[4][i];
          double yaw = x[5][i];
          msg.plan[i].origin_position.rotation.x = Math.sin(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2)-Math.cos(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
          msg.plan[i].origin_position.rotation.y = Math.cos(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2)+Math.sin(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2);
          msg.plan[i].origin_position.rotation.z = Math.cos(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2)-Math.sin(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2);
          msg.plan[i].origin_position.rotation.w = Math.cos(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2)+Math.sin(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
          msg.plan[i].origin_twist.linear_velocity.x = x[6+msg.plan[i].num_joints][i];
          msg.plan[i].origin_twist.linear_velocity.y = x[6+msg.plan[i].num_joints+1][i];
          msg.plan[i].origin_twist.linear_velocity.z = x[6+msg.plan[i].num_joints+2][i];
          msg.plan[i].origin_twist.angular_velocity.x = x[6+msg.plan[i].num_joints+3][i];
          msg.plan[i].origin_twist.angular_velocity.y = x[6+msg.plan[i].num_joints+4][i];
          msg.plan[i].origin_twist.angular_velocity.z = x[6+msg.plan[i].num_joints+5][i];
        }
        
        //System.out.format("The %d's state is at time %d, origin's translation is %f, %f, %f, angles are %f,%f,%f; the velocity of origin is %f, %f, %f, %f, %f, %f\n",i+1,msg.plan[i].utime,msg.plan[i].origin_position.translation.x,msg.plan[i].origin_position.translation.y,msg.plan[i].origin_position.translation.z,msg.plan[i].origin_position.rotation.x,msg.plan[i].origin_position.rotation.y,msg.plan[i].origin_position.rotation.z,msg.plan[i].origin_twist.linear_velocity.x,msg.plan[i].origin_twist.linear_velocity.y,msg.plan[i].origin_twist.linear_velocity.z,msg.plan[i].origin_twist.angular_velocity.x,msg.plan[i].origin_twist.angular_velocity.y,msg.plan[i].origin_twist.angular_velocity.z);
        
        for(int j = 0;j<msg.plan[i].num_joints;j++)
        {
          msg.plan[i].joint_position[j] = (float) x[j+nonfloating_joint_start_ndx][i];
          msg.plan[i].joint_velocity[j] = (float) x[msg.plan[i].num_joints+j+nonfloating_joint_start_ndx][i];
          //System.out.format("The %d's joint of %d's state has joint angle %f, joint velocity %f\n",j+1,i+1,msg.plan[i].joint_position[j],msg.plan[i].joint_velocity[j]);
        }
      }
      lcm.publish(channel_name,msg);
    }
}
