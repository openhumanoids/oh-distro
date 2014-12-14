#include <leg-odometry/sharedUtilities.hpp>

using namespace std;


Eigen::Isometry3d KDLToEigen(KDL::Frame tf){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.p[0], tf.p[1], tf.p[2];
  Eigen::Quaterniond q;
  tf.M.GetQuaternion( q.x() , q.y(), q.z(), q.w());
  tf_out.rotate(q);
  return tf_out;
}

Eigen::Quaterniond KDLToEigenQuaterniond(KDL::Frame tf){
  Eigen::Quaterniond q;
  tf.M.GetQuaternion( q.x() , q.y(), q.z(), q.w());
  return q;
}

// updates left, right and body_to_head Isometry3d tranforms
void TwoLegs::getFKTransforms(TwoLegs::FK_Data &_fk_data, Eigen::Isometry3d &left, Eigen::Isometry3d &right, Eigen::Isometry3d &body_to_head) {

    bool kinematics_status;
    bool flatten_tree=true; // determines absolute transforms to robot origin, otherwise relative transforms between joints.

    // 1. Solve for Forward Kinematics:
    _fk_data._link_tfs.clear(); // TODO -- not sure why we have this, to investigate

    //map<string, drc::transform_t > cartpos_out;
    map<string, KDL::Frame > cartpos_out;

    kinematics_status = _fk_data.fksolver_->JntToCart(_fk_data.jointpos_in, cartpos_out, flatten_tree);

    //    map<string, drc::transform_t >::iterator transform_it_ll_l;
    //    map<string, drc::transform_t >::iterator transform_it_ll_r;
    map<string, KDL::Frame >::iterator transform_it_ll_l;
    map<string, KDL::Frame >::iterator transform_it_ll_r;

    // This should be depreciated
    transform_it_ll_l=cartpos_out.find("l_talus");
    transform_it_ll_r=cartpos_out.find("r_talus");

    // Find the pelvis to head transform
    // map<string, drc::transform_t >::iterator transform_it_ph;
    map<string, KDL::Frame >::iterator transform_it_ph;
    transform_it_ph=cartpos_out.find("head");

    Eigen::Quaterniond b2head_q = KDLToEigenQuaterniond(transform_it_ph->second);//.rotation.w, transform_it_ph->second.rotation.x,transform_it_ph->second.rotation.y,transform_it_ph->second.rotation.z);

    body_to_head.setIdentity();
    body_to_head.linear() = q2C(b2head_q);
    body_to_head.translation() <<  transform_it_ph->second.p[0] , transform_it_ph->second.p[1] , transform_it_ph->second.p[2];// transform_it_ph->second.translation.x, transform_it_ph->second.translation.y, transform_it_ph->second.translation.z;

    // get lcam to pelvis transform
    // map<string, drc::transform_t >::iterator transform_it_lcam;
    map<string, KDL::Frame >::iterator transform_it_lcam;
    transform_it_lcam=cartpos_out.find("left_camera_optical_frame");

    Eigen::Isometry3d p2lc;
    p2lc.setIdentity();
    p2lc.translation() << transform_it_lcam->second.p[0], transform_it_lcam->second.p[1], transform_it_lcam->second.p[2];
    p2lc.linear() = q2C(  KDLToEigenQuaterniond( transform_it_lcam->second) );
    _fk_data.bottransforms.setLCam2Pelvis(p2lc);

    //T_body_head = KDL::Frame::Identity();
    if(transform_it_ll_l!=cartpos_out.end()){// fk cart pos exists
      // This gives us the translation from body to left foot

    }else{
      std::cout<< "fk position does not exist" <<std::endl;
    }

    if(transform_it_ll_r!=cartpos_out.end()){// fk cart pos exists

    }else{
      std::cout<< "fk position does not exist" << std::endl;
    }



    // Get lower leg quaternions
    Eigen::Quaterniond q_ll_l(  KDLToEigenQuaterniond(transform_it_ll_l->second) );
    Eigen::Quaterniond q_ll_r( KDLToEigenQuaterniond(transform_it_ll_r->second) );

    //std::cout << "TwoLegs::getFKTransforms -- q is " << q_ll_l.w() << ", " << q_ll_l.x() << ", " << q_ll_l.y() << ", " << q_ll_l.z() << std::endl;


    Eigen::Isometry3d left_lleg;
    Eigen::Isometry3d right_lleg;

    left_lleg.translation() << transform_it_ll_l->second.p[0], transform_it_ll_l->second.p[1], transform_it_ll_l->second.p[2];
    right_lleg.translation() << transform_it_ll_r->second.p[0], transform_it_ll_r->second.p[1], transform_it_ll_r->second.p[2];

    //left_lleg.rotate(q_ll_l);// DO NOT TRUST THIS IN THE DRC CONTEXT
    //right_lleg.rotate(q_ll_r);// DO NOT TRUST THIS IN THE DRC CONTEXT

    left_lleg.linear() = q2C(q_ll_l);
    right_lleg.linear() = q2C(q_ll_r);


    // Now i need to imitate the ankle pitch and roll angles
    // this involves rotating the lower leg position with the IMU world angles. pay attention to the order in which these rotations are applied.

    //bot_core::rigid_transform_t tf;
    //KDL::Frame T_body_head;

    //map<string, drc::transform_t >::iterator transform_it_lf;
    //map<string, drc::transform_t >::iterator transform_it_rf;
    map<string, KDL::Frame >::iterator transform_it_lf;
    map<string, KDL::Frame >::iterator transform_it_rf;

    transform_it_lf=cartpos_out.find("l_foot");
    transform_it_rf=cartpos_out.find("r_foot");

    //T_body_head = KDL::Frame::Identity();
    if(transform_it_lf!=cartpos_out.end()){// fk cart pos exists
      // This gives us the translation from body to left foot

    }else{
      std::cout<< "fk position does not exist" <<std::endl;
    }

    if(transform_it_lf!=cartpos_out.end()){// fk cart pos exists

    }else{
      std::cout<< "fk position does not exist" << std::endl;
    }

    //Eigen::Vector3d E_;
    // quaternion scale and vector ordering seems to be correct
    Eigen::Quaterniond  leftq( KDLToEigenQuaterniond(transform_it_lf->second) );
    Eigen::Quaterniond rightq( KDLToEigenQuaterniond(transform_it_rf->second) );

    left.setIdentity();
    right.setIdentity();

    left.translation() << transform_it_lf->second.p[0], transform_it_lf->second.p[1], transform_it_lf->second.p[2];
    right.translation() << transform_it_rf->second.p[0], transform_it_rf->second.p[1], transform_it_rf->second.p[2];

    // We ignore rotations
    //left.linear() = q2C(leftq);
    //right.linear() = q2C(rightq);
//
//
//	#ifdef  MATTS_HELP
//
//      // level out foot position from IMU
//      Eigen::Isometry3d IMU_rp;
//      IMU_rp.setIdentity();
//      Eigen::Vector3d imu_E;
//
//      //std::cout << "Bangles: " <<_leg_odo->getLocalOrientation().w() << ", " <<_leg_odo->getLocalOrientation().x() << ", " <<_leg_odo->getLocalOrientation().y() << ", " <<_leg_odo->getLocalOrientation().z() << std::endl;
//      imu_E = q2e_new(_leg_odo->getLocalOrientation());
//      imu_E(2) = 0.;
//      //std::cout << "Aangles: " << imu_E.transpose() << std::endl << q2C(e2q(imu_E)) << std::endl;
//
//
//      IMU_rp.linear() = e2C(imu_E);
//
//      Eigen::Isometry3d temptransform;
//      Eigen::Isometry3d tempright, templeft;
//
//
//      temptransform.setIdentity();
//      temptransform = (IMU_rp)*left;
//      left = temptransform;
//      temptransform = (IMU_rp)*right;
//      right = temptransform;
//
//
//
//      // now we strip out the influence of the ankle joints.
//      // We do not need to know the slope of the terrain. Assuming all footsteps are flat at the contact point
//      Eigen::Vector3d stripRP;
//
//      //stripRP = q2e_new(InertialOdometry::QuaternionLib::C2q(left.linear()));
//      stripRP = q2e_new(C2q(left.linear()));
//      //std::cout << "Stripping left angles: " << stripRP.transpose() << std::endl;
//      stripRP(0) = 0.;
//      stripRP(1) = 0.;
//
//      templeft.setIdentity();
//      templeft.translation() = left.translation();
//      templeft.linear() = e2C(stripRP);
//
//
//      stripRP = q2e_new(C2q(right.linear()));
//      stripRP(0) = 0.;
//      stripRP(1) = 0.;
//
//      tempright.setIdentity();
//      tempright.translation() = right.translation();
//      tempright.linear() = e2C(stripRP);
//
//      left = templeft;
//      right = tempright;
//	#endif
//
//
//
//	#ifdef LOG_LEG_TRANSFORMS
//      // The idea here is to push all the required data to a single array [pelvis_to_feet_transform], which is to be logged in publish state method
//
//      Eigen::Vector3d tempvar;
//      int i;
//
//      tempvar = pelvis_to_feet_speed[0].diff(u_ts,left.translation());
//      // left vel, right vel, left rate, right rate
//      for (i=0;i<3;i++) {
//        pelvis_to_feet_transform[i] = tempvar(i); // left vel, right vel, left rate, right rate
//      }
//      tempvar = pelvis_to_feet_speed[1].diff(u_ts,right.translation());
//      // left vel, right vel, left rate, right rate
//      for (i=0;i<3;i++) {
//        pelvis_to_feet_transform[3+i] = tempvar(i); // left vel, right vel, left rate, right rate
//      }
//      tempvar = pelvis_to_feet_speed[2].diff(u_ts,C2e(left.rotation()));
//      // left vel, right vel, left rate, right rate
//      for (i=0;i<3;i++) {
//        pelvis_to_feet_transform[6+i] = tempvar(i); // left vel, right vel, left rate, right rate
//      }
//      tempvar = pelvis_to_feet_speed[3].diff(u_ts,C2e(right.rotation()));
//      // left vel, right vel, left rate, right rate
//      for (i=0;i<3;i++) {
//        pelvis_to_feet_transform[9+i] = tempvar(i); // left vel, right vel, left rate, right rate
//      }
//
//	#endif
	
}
 
