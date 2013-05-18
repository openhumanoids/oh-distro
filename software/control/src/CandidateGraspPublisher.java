import java.io.*;
import lcm.lcm.*;

public class CandidateGraspPublisher
{
    drc.desired_grasp_state_t msg;
    String channel_name;
    //const int16_t SANDIA_LEFT=0, SANDIA_RIGHT=1, SANDIA_BOTH=2, IROBOT_LEFT=3, IROBOT_RIGHT=4, IROBOT_BOTH=5;
    public CandidateGraspPublisher(String robot_name, String object_name, String geometry_name,int unique_id, short grasp_type, String[] l_joint_name, String[] r_joint_name, String channel)
    {
	    msg = new drc.desired_grasp_state_t();
	    msg.robot_name = robot_name;
	    msg.object_name=  object_name; 
	    msg.geometry_name =  geometry_name; 
	    msg.unique_id = unique_id; 
	    msg.power_grasp = false;
      msg.grasp_type = grasp_type; //msg.SANDIA_LEFT
      
      if ((msg.grasp_type == msg.SANDIA_LEFT)||(msg.grasp_type == msg.SANDIA_BOTH)||(msg.grasp_type == msg.IROBOT_LEFT)||(msg.grasp_type == msg.IROBOT_BOTH)) {
	    msg.num_l_joints = l_joint_name.length;
	    msg.l_joint_name = l_joint_name;
	    }
	    
	    if ((msg.grasp_type == msg.SANDIA_RIGHT)||(msg.grasp_type == msg.SANDIA_BOTH)||(msg.grasp_type == msg.IROBOT_RIGHT)||(msg.grasp_type == msg.IROBOT_BOTH)) {
	    msg.num_r_joints = r_joint_name.length;
	    msg.r_joint_name = r_joint_name;
	    }
	    msg.l_hand_pose = new drc.position_3d_t();
	    msg.l_hand_pose.translation = new drc.vector_3d_t();
      msg.l_hand_pose.rotation = new drc.quaternion_t();
      msg.l_hand_pose.rotation.w = 1.0;
      
      msg.r_hand_pose = new drc.position_3d_t();
	    msg.r_hand_pose.translation = new drc.vector_3d_t();
      msg.r_hand_pose.rotation = new drc.quaternion_t();
      msg.r_hand_pose.rotation.w = 1.0;
      
      msg.l_joint_position = new double[msg.num_l_joints];
      msg.r_joint_position = new double[msg.num_r_joints];
	    for (int i=0; i<msg.num_l_joints; i++)
	    {
	      msg.l_joint_position[i] = 0.0;  
	    }
	    for (int i=0; i<msg.num_r_joints; i++)
	    {
	      msg.r_joint_position[i] = 0.0;  
	    }
	    
	    msg.optimization_status =msg.RUNNING;//   const int16_t RUNNING=0, SUCCESS=1, FAILURE=2;

	    channel_name = channel;
    }

    public void publish(long utime, double[] hand_pose_trans, double[] hand_pose_rot, double[] joint_position) // use doubles here for compatibility w/ matlab
    {
	    LCM lcm = LCM.getSingleton();
	    msg.utime = utime; //System.nanoTime()/1000; // THIS SHOULD NOT BE SYSTEM TIME,  use the INIT_GRASP_SEED time
	    
	    if ((msg.grasp_type == msg.SANDIA_LEFT)||(msg.grasp_type == msg.IROBOT_LEFT)) {
	    msg.l_hand_pose.translation.x = hand_pose_trans[0];
	    msg.l_hand_pose.translation.y = hand_pose_trans[1];
	    msg.l_hand_pose.translation.z = hand_pose_trans[2];
	    msg.l_hand_pose.rotation.x = hand_pose_rot[0];
	    msg.l_hand_pose.rotation.y = hand_pose_rot[1];
	    msg.l_hand_pose.rotation.z = hand_pose_rot[2];
	    msg.l_hand_pose.rotation.w = hand_pose_rot[3];
	    msg.l_joint_position = joint_position;
	    }
	    else if((msg.grasp_type == msg.SANDIA_RIGHT)||(msg.grasp_type == msg.IROBOT_RIGHT)) {
	    msg.r_hand_pose.translation.x = hand_pose_trans[0];
	    msg.r_hand_pose.translation.y = hand_pose_trans[1];
	    msg.r_hand_pose.translation.z = hand_pose_trans[2];
	    msg.r_hand_pose.rotation.x = hand_pose_rot[0];
	    msg.r_hand_pose.rotation.y = hand_pose_rot[1];
	    msg.r_hand_pose.rotation.z = hand_pose_rot[2];
	    msg.r_hand_pose.rotation.w = hand_pose_rot[3];
	    msg.r_joint_position = joint_position;
	    }
	    lcm.publish(channel_name,msg);
    }
    
    public void publish(long utime, double[] l_hand_pose_trans, double[] l_hand_pose_rot, double[] l_joint_position, double[] r_hand_pose_trans, double[] r_hand_pose_rot, double[] r_joint_position) // use doubles here for compatibility w/ matlab
    {
	    LCM lcm = LCM.getSingleton();
	    msg.utime = utime; //System.nanoTime()/1000; // THIS SHOULD NOT BE SYSTEM TIME,  use the INIT_GRASP_SEED time
	    
	    if ((msg.grasp_type == msg.SANDIA_BOTH)||(msg.grasp_type == msg.IROBOT_BOTH)) {
	    msg.l_hand_pose.translation.x = l_hand_pose_trans[0];
	    msg.l_hand_pose.translation.y = l_hand_pose_trans[1];
	    msg.l_hand_pose.translation.z = l_hand_pose_trans[2];
	    msg.l_hand_pose.rotation.x = l_hand_pose_rot[0];
	    msg.l_hand_pose.rotation.y = l_hand_pose_rot[1];
	    msg.l_hand_pose.rotation.z = l_hand_pose_rot[2];
	    msg.l_hand_pose.rotation.w = l_hand_pose_rot[3];
	    msg.l_joint_position = l_joint_position;
	   
	    msg.r_hand_pose.translation.x = r_hand_pose_trans[0];
	    msg.r_hand_pose.translation.y = r_hand_pose_trans[1];
	    msg.r_hand_pose.translation.z = r_hand_pose_trans[2];
	    msg.r_hand_pose.rotation.x = r_hand_pose_rot[0];
	    msg.r_hand_pose.rotation.y = r_hand_pose_rot[1];
	    msg.r_hand_pose.rotation.z = r_hand_pose_rot[2];
	    msg.r_hand_pose.rotation.w = r_hand_pose_rot[3];
	    msg.r_joint_position = r_joint_position;
	    }
	    
	    lcm.publish(channel_name,msg);
    }

}
