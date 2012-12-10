// minimal two-way LCM gazebo plugin
// - listens to LCM for rotation rate of ROTATING_SCAN
// - publishes the angular position @ 100s of Hz
// mfallon nov 2012

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

Eigen::Quaterniond euler_to_quat(double yaw, double pitch, double roll) {
  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
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


namespace gazebo
{   
  
class HokuyoRightPlugin : public ModelPlugin
{
    
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;
    this->world = _parent->GetWorld();
    
    // Initial default rotation rate of laser:
    // TODO: read from config:
    this->target_velocity = M_PI/2;
      
    // LCM receive thread:
    this->callback_queue_thread_ = boost::thread(boost::bind(&HokuyoRightPlugin::QueueThread, this));
      
    
    
    
    // Load parameters for this plugin
    if (this->LoadParams(_sdf)){
      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&HokuyoRightPlugin::OnUpdate, this));
    }
    
  // assert that the body by link_name_ exists
    std::string link_name = "head_rightedhokuyo";
  this->link = boost::shared_dynamic_cast<physics::Link>(this->world->GetEntity(link_name));
  if (!this->link)
  {
    gzerr << "plugin error: "<<  link_name <<" does not exist\n";
    return;
  }
    

      
    if(!lcm_publish_.good()){
       gzerr <<"ERROR: lcm is not good()" <<std::endl;
      }
  }
    
  public: bool LoadParams(sdf::ElementPtr _sdf){
    if (this->FindJointByParam(_sdf, this->head_hokuyo_joint_,
                             "head_rightedhokuyo_joint")){
      return true;
    }else
        return false;
  }

  public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param){
    if (!_sdf->HasElement(_param)){
      gzerr << "param [" << _param << "] not found\n";
      return false;
    }else{
      _joint = this->model->GetJoint(_sdf->GetElement(_param)->GetValueString());

      if (!_joint){
        gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
        return false;
      }
    }
    return true;
  }

  // Called by the world update start event
  public: void OnUpdate(){
    //      this->head_hokuyo_joint_->SetForce(0, 0.02); // continually accelerating
    //this->head_hokuyo_joint_->SetVelocity(0,target_velocity );
    this->head_hokuyo_joint_->SetMaxForce(0, 5.0); // from sisir's diff drive plugin 

    math::Pose pose;
    pose = this->link->GetWorldPose(); // - this->link->GetCoMPose();
    math::Quaternion rot = pose.rot;
    // gzerr << " --------- rot " << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.w << std::endl;
    
    Eigen::Quaterniond rot_eigen= Eigen::Quaterniond(rot.w,rot.x,rot.y,rot.z);
    double ypr[3];
    quat_to_euler(rot_eigen, ypr[0], ypr[1], ypr[2]);
    //gzerr << " --------- ypr "  << ypr[0] << ", " << ypr[1] << ", "  << ypr[2]  << std::endl;
    
    double val = fabs(ypr[1]);
    //gzerr << val << " val\n";
    if ( fabs(ypr[1]) < 0.3 ){
      //gzerr << "CLOSE TO FLAT CLOSE =====================================\n";
      if ( ypr[1]  > 0.0 ){
         //gzerr << "POSITIVE =====================================\n";
         this->head_hokuyo_joint_->SetVelocity(0,1.4 );
      }else{
         //gzerr << "NEGATIVE =====================================\n";
         this->head_hokuyo_joint_->SetVelocity(0,-1.4 );
      }
    }else{
      this->head_hokuyo_joint_->SetVelocity(0,target_velocity );
    }
    
    
    /*
    double curr_ang, curr_vel;
    curr_ang = this->head_hokuyo_joint_->GetAngle(0).GetAsRadian();
    curr_vel=this->head_hokuyo_joint_->GetVelocity(0);
    gzerr << curr_time << " is curr time\n";
    gzerr << "curr: " << curr_ang << " | " << curr_vel << " right\n";
    Eigen::Quaterniond r= euler_to_quat(0,0, curr_ang); */

    bot_core::pose_t tf; 
    common::Time sim_time = this->world->GetSimTime();
    int64_t curr_time = (int64_t) round(sim_time.Double()*1E6);
    tf.utime = curr_time;

    // hard coded until inverse kin. is added:
    tf.pos[0] = pose.pos.x;
    tf.pos[1] = pose.pos.y;
    tf.pos[2] = pose.pos.z;
    
    tf.orientation[0] =pose.rot.w;
    tf.orientation[1] =pose.rot.x;
    tf.orientation[2] =pose.rot.y;
    tf.orientation[3] =pose.rot.z;
    lcm_publish_.publish("POSE_RIGHTED", &tf);
  }

  ////// All LCM receive thread work:
  public: void QueueThread(){
    lcm::LCM lcm_subscribe_ ;
    if(!lcm_subscribe_.good()){
      gzerr <<"ERROR: lcm_subscribe_ is not good()\n";
    }
  
    gzerr << "Launching Lidar LCM handler [Righting]\n";
    //lcm_subscribe_.subscribe("ROTATING_SCAN_RATE_CMD", &HokuyoRightPlugin::on_rotate_freq, this);
    //while (0 == lcm_subscribe_.handle());
  }    
    
    /*
  public: void on_rotate_freq(const lcm::ReceiveBuffer* buf,
                    const std::string& channel,
                    const drc::twist_timed_t* msg){
   
    gzerr << msg->angular_velocity.x << " is new rotation rate frequency\n";
    target_velocity = msg->angular_velocity.x;
  }*/  
  
  // Pointer to the model
  private: 
    physics::ModelPtr model;
    physics::WorldPtr world;
    lcm::LCM lcm_publish_ ;
    double target_velocity;
    boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: physics::JointPtr head_hokuyo_joint_;

    
    private: physics::LinkPtr link;    
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(HokuyoRightPlugin)
}
