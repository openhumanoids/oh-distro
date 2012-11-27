import java.io.*;
import lcm.lcm.*;

public class RobotPlanPublisher
{
    drc.robot_plan_t msg;
    String channel_name;

    public RobotPlanPublisher(String robot_name, int num_states,String[] joint_name,String channel)
    {
        msg = new drc.robot_plan_t();
        msg.robot_name = robot_name;
        msg.num_states = num_states;
        String[] nonFloating_joint_name;
        if(joint_name[0]=="base.x")
        {
            nonFloating_joint_name = new String[joint_name.length-6];
            for(int i = 0;i<nonFloating_joint_name.length;i++)
            {
                nonFloating_joint_name[i] = joint_name[i+6];
            }
        }
        else
        {
            nonFloating_joint_name = joint_name;
        }

        msg.plan = new drc.robot_state_t[msg.num_states]; 
        for (int i=0;i<msg.num_states;i++)
        {
            msg.plan[i] = new drc.robot_state_t();
            msg.plan[i].robot_name = robot_name;
            msg.plan[i].num_joints = nonFloating_joint_name.length;
            msg.plan[i].joint_name = nonFloating_joint_name;
            msg.plan[i].origin_position = new drc.position_3d_t();
            msg.plan[i].origin_position.translation = new drc.vector_3d_t();
            msg.plan[i].origin_position.rotation = new drc.quaternion_t();
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
        channel_name = channel;
    }
    public void publish(double[] t,double[] x)
    {
        LCM lcm = LCM.getSingleton();
        msg.utime = System.nanoTime()/1000;
        //System.out.format("Time is %d\n",msg.utime);
        int xind = 0;
        for(int i = 0;i<msg.num_states;i++)
        {
                msg.plan[i].utime = (long) (t[i]*1000000+msg.utime);
                msg.plan[i].origin_position.translation.x = (float) x[xind];
                msg.plan[i].origin_position.translation.y = (float) x[xind+1];
                msg.plan[i].origin_position.translation.z = (float) x[xind+2];
                double roll = x[xind+3];
                double pitch = x[xind+4];
                double yaw = x[xind+5];
                msg.plan[i].origin_position.rotation.x = Math.sin(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2)-Math.cos(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
                msg.plan[i].origin_position.rotation.y = Math.cos(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2)+Math.sin(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2);
                msg.plan[i].origin_position.rotation.z = Math.cos(roll/2)*Math.cos(pitch/2)*Math.sin(yaw/2)-Math.sin(roll/2)*Math.sin(pitch/2)*Math.cos(yaw/2);
                msg.plan[i].origin_position.rotation.w = Math.cos(roll/2)*Math.cos(pitch/2)*Math.cos(yaw/2)+Math.sin(roll/2)*Math.sin(pitch/2)*Math.sin(yaw/2);
                msg.plan[i].origin_twist.linear_velocity.x = x[xind+6+msg.plan[i].num_joints];
                msg.plan[i].origin_twist.linear_velocity.y = x[xind+6+msg.plan[i].num_joints+1];
                msg.plan[i].origin_twist.linear_velocity.z = x[xind+6+msg.plan[i].num_joints+2];
                msg.plan[i].origin_twist.angular_velocity.x = x[xind+6+msg.plan[i].num_joints+3];
                msg.plan[i].origin_twist.angular_velocity.y = x[xind+6+msg.plan[i].num_joints+4];
                msg.plan[i].origin_twist.angular_velocity.z = x[xind+6+msg.plan[i].num_joints+5];
                

                //System.out.format("The %d's state is at time %d, origin's translation is %f, %f, %f, angles are %f,%f,%f; the velocity of origin is %f, %f, %f, %f, %f, %f\n",i+1,msg.plan[i].utime,msg.plan[i].origin_position.translation.x,msg.plan[i].origin_position.translation.y,msg.plan[i].origin_position.translation.z,msg.plan[i].origin_position.rotation.x,msg.plan[i].origin_position.rotation.y,msg.plan[i].origin_position.rotation.z,msg.plan[i].origin_twist.linear_velocity.x,msg.plan[i].origin_twist.linear_velocity.y,msg.plan[i].origin_twist.linear_velocity.z,msg.plan[i].origin_twist.angular_velocity.x,msg.plan[i].origin_twist.angular_velocity.y,msg.plan[i].origin_twist.angular_velocity.z); 
            for(int j = 0;j<msg.plan[i].num_joints;j++)
            {
                msg.plan[i].joint_position[j] = (float) x[xind+j+6];                
                msg.plan[i].joint_velocity[j] = (float) x[xind+6+msg.plan[i].num_joints+6+j];
                //System.out.format("The %d's joint of %d's state has joint angle %f, joint velocity %f\n",j+1,i+1,msg.plan[i].joint_position[j],msg.plan[i].joint_velocity[j]);
            }
            xind += 2*(msg.plan[i].num_joints+6);
            
        }
        lcm.publish(channel_name,msg);
    }
}
