#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core.hpp>


#include <Eigen/Dense>
#include <Eigen/StdVector>


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



namespace gazebo
{   
  class MobileBasePlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {

      // Store the pointer to the model
      this->model = _parent;
      this->world = _parent->GetWorld();

      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateStart(
            boost::bind(&MobileBasePlugin::OnUpdate, this));
      }

      
      if(!lcm_publish_.good()){
       gzerr <<"ERROR: lcm is not good()" <<std::endl;
      }


    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {
      if (this->FindJointByParam(_sdf, this->head_hokuyo_joint_,
                             "head_hokuyo_joint"))
        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
//      this->head_hokuyo_joint_->SetForce(0, 0.02); // continually accelerating
      this->head_hokuyo_joint_->SetVelocity(0, M_PI/2);
      this->head_hokuyo_joint_->SetMaxForce(0, 5.0); // from sisir's diff drive plugin 

      double curr_ang, curr_vel;
      curr_ang = this->head_hokuyo_joint_->GetAngle(0).GetAsRadian();
      curr_vel=this->head_hokuyo_joint_->GetVelocity(0);
      common::Time sim_time = this->world->GetSimTime();
      int64_t curr_time = (int64_t) round(sim_time.Double()*1E6);

      //gzerr << curr_time << " is curr time\n";
      //gzerr << "curr: " << curr_ang << " | " << curr_vel << "\n";

      Eigen::Quaterniond r= euler_to_quat(0,0, curr_ang);
      bot_core::rigid_transform_t tf;
      tf.utime = curr_time;

      // hard coded until inverse kin. is added:
      tf.trans[0] = -0.0015;
      tf.trans[1] = 0;
      tf.trans[2] = 0.68;
    
      tf.quat[0] =r.w();
      tf.quat[1] =r.x();
      tf.quat[2] =r.y();
      tf.quat[3] =r.z();
      lcm_publish_.publish("BODY_TO_ROTATING_SCAN", &tf);
   }

    // Pointer to the model
    private: 
      physics::ModelPtr model;
      physics::WorldPtr world;
      lcm::LCM lcm_publish_ ;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr head_hokuyo_joint_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
