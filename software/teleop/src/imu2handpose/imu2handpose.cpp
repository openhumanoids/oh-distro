#include <zlib.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include <lcmtypes/microstrain_ins_t.h>
#include <lcmtypes/drc_lcmtypes.h>
#include <ConciseArgs>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign; // bring 'operator+()' into scope
using namespace std;

////////////////////////////////////////////////////////////////
class ObstacleApp{
  public:
    ObstacleApp(lcm_t* publish_lcm,int mode);
    
    ~ObstacleApp(){
    }
    
    void PublishPoseToLCM(int id, Eigen::Isometry3d poseout,int64_t utime);
  private:
    lcm_t* lcm_;
    int mode;
};  


ObstacleApp::ObstacleApp(lcm_t* lcm_,int mode) : lcm_(lcm_) , mode(mode)
{
}


Eigen::Quaterniond imu2robotquat(const microstrain_ins_t *msg){
  Eigen::Quaterniond m(msg->quat[0],msg->quat[1],msg->quat[2],msg->quat[3]);
  
  Eigen::Isometry3d motion_estimate;
  motion_estimate.setIdentity();
  motion_estimate.translation() << 0,0,0;
  motion_estimate.rotate(m);  
  
  // rotate coordinate frame so that look vector is +X, and up is +Z
  Eigen::Matrix3d M;
    
  // Works for another configuration - keeping in case useful later:
  //  M <<  0,  1, 0,
  //	1, 0, 0,
  //	0, 0, -1;

  //convert imu on bocelli cane from:
  //x+ right, z+ down, y+ back
  //to x+ forward, y+ left, z+ up (robotics)
  //M <<  0,  -1, 0,
  //	-1, 0, 0,
  //		0, 0, -1;

  // Valid
  //  M <<  -1, 0, 0,
  //	0, -1, 0,
  //	0, 0, 1;

  M <<  -1, 0, 0,
	0, 1, 0,
	0, 0, -1;

	motion_estimate= M * motion_estimate;
	Eigen::Vector3d translation(motion_estimate.translation());
	Eigen::Quaterniond rotation(motion_estimate.rotation());
	rotation = rotation * M.transpose();

	Eigen::Isometry3d motion_estimate_out;
	motion_estimate_out.setIdentity();
	motion_estimate_out.translation() << translation[0],translation[1],translation[2];
	motion_estimate_out.rotate(rotation);

	Eigen::Quaterniond r_x(motion_estimate_out.rotation());

  return r_x;
}


void quat_to_euler(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


void ObstacleApp::PublishPoseToLCM(int id, Eigen::Isometry3d poseout,int64_t utime)
{
  bool send_pose = true;
  Eigen::Quaterniond r_x(poseout.rotation());

  if (send_pose){
    bot_core_pose_t pose_msg;
    memset(&pose_msg, 0, sizeof(pose_msg));
    pose_msg.utime =   utime;
    pose_msg.pos[0] =poseout.translation().x();
    pose_msg.pos[1] =poseout.translation().y();
    pose_msg.pos[2] =poseout.translation().z();  
    pose_msg.orientation[0] =  r_x.w();  
    pose_msg.orientation[1] =  r_x.x();  
    pose_msg.orientation[2] =  r_x.y();  
    pose_msg.orientation[3] =  r_x.z();  
    bot_core_pose_t_publish(lcm_, "POSE_HEAD", &pose_msg);      
  }

  double yaw, pitch, roll;
  quat_to_euler(r_x, yaw,pitch,roll);
  cout << pitch*180/M_PI << " | " << roll*180/M_PI << "\n";

/*   string joint_names[] ={ "back_lbz",
       "back_mby", "back_ubx", "neck_ay", "l_leg_uhz", 
       "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
       "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay",
       "r_leg_lax", "l_arm_elx", "l_arm_ely", "l_arm_mwx", "l_arm_shx",
       "l_arm_usy", "l_arm_uwy", "r_arm_elx", "r_arm_ely", "r_arm_mwx",
       "r_arm_shx", "r_arm_usy", "r_arm_uwy"};
   double joint_position[] ={ 0.0, 
       0,0,0,0,
       0,0,0,0,0,
       0,0,0,0,0,
       0,0,0,0,0,
       0,0,0,0,0,
       0,0,0  }; */
  
   string joint_names[] ={ "r_arm_mwx", "r_arm_uwy"};
   double joint_position[2];
       
  if (roll > 1.2){
    roll =1.2;
  }else if (roll < -1.2){
    roll =-1.2;
  }
  if (pitch > 1.2){
    pitch =1.2;
  }else if (pitch < -1.2){
    pitch =-1.2;
  }

  joint_position[0] = pitch;
  joint_position[1] = -roll;

  drc_joint_angles_t angles_msg;
  angles_msg.utime=utime;
  angles_msg.robot_name="atlas";
  angles_msg.num_joints=2;
  angles_msg.joint_name=(char**)joint_names;
  angles_msg.joint_position=joint_position;
  drc_joint_angles_t_publish(lcm_, "JOINT_POSITION_CMDS", &angles_msg);      
}


void on_imu(const lcm_recv_buf_t *rbuf, const char *channel,
    const microstrain_ins_t *msg, void *user_data){
  ObstacleApp* obsapp = (ObstacleApp*) user_data;

  Eigen::Quaterniond m = imu2robotquat(msg);
  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation() << 0,0,0;
  pose.rotate(m);
  obsapp->PublishPoseToLCM(1003,pose,msg->utime);
}

int main(int argc, char **argv)
{
  ConciseArgs parser(argc, argv, "kinect obstacle detection");
  int output_mode=1;
  parser.add(output_mode, "o", "output_mode", "1 18@20d, 2 36@10d, 3 15@24d | 4 8@45 | all 4 ranges 1m ][ 5 .125m 32@45 ");
  parser.parse();
  cout << output_mode << " is output_mode\n";
  
  lcm_t*  lcm = lcm_create(NULL);
  ObstacleApp obsapp= ObstacleApp(lcm,output_mode);
    
  microstrain_ins_t_subscribe(lcm, "MICROSTRAIN_INS", on_imu,&obsapp);

  // go!
  while(0 == lcm_handle(lcm));// && !shutdown_flag);
}
