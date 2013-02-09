#ifndef KMCL_FOVISION_HPP_
#define KMCL_FOVISION_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.h>
#include <fovis/fovis.hpp>

#include <lcmtypes/fovis_stats_t.h>
#include <lcmtypes/fovis_update_t.h>

class FoVision
{
public:
    FoVision(boost::shared_ptr<lcm::LCM> &lcm_,
             boost::shared_ptr<fovis::StereoCalibration> kcal);
    //FoVision(boost::shared_ptr<lcm::LCM> &lcm_);
    
    
    ~FoVision();
    void doOdometry(uint8_t *rgb, uint8_t *depth);
    void fovis_stats();
    
    Eigen::Isometry3d getMotionEstimate(){ 
      Eigen::Isometry3d motion_estimate = odom_.getMotionEstimate();
      
      // rotate coordinate frame so that look vector is +X, and up is +Z
      Eigen::Matrix3d M;
      M <<  0,  0, 1,
	    -1,  0, 0,
	    0, -1, 0;

      motion_estimate= M * motion_estimate;
      Eigen::Vector3d translation(motion_estimate.translation());
      Eigen::Quaterniond rotation(motion_estimate.rotation());
      rotation = rotation * M.transpose();  
      
      Eigen::Isometry3d motion_estimate_out;
      motion_estimate_out.setIdentity();
      motion_estimate_out.translation() << translation[0],translation[1],translation[2];
      motion_estimate_out.rotate(rotation);
      return motion_estimate_out;
    }
      
    fovis::MotionEstimateStatusCode getEstimateStatus(){
      fovis::MotionEstimateStatusCode estim_status = odom_.getMotionEstimateStatus();
      /*std::cout << estim_status << "is status\n";
      if (estim_status == fovis::SUCCESS){
        std::cout << estim_status << "is valid status\n";
      }else if (estim_status = fovis::INSUFFICIENT_INLIERS){
        std::cout << estim_status << "is FOVIS_UPDATE_T_ESTIMATE_INSUFFICIENT_FEATURES\n";
      }*/

      return estim_status;
    }
    
    void setCurrentTimestamp(int64_t current_timestamp_in){
      current_timestamp_ = current_timestamp_in;
    }

    const fovis::FeatureMatch* getMatches(){ return odom_.getMotionEstimator()->getMatches(); }
    int getNumMatches(){ return odom_.getMotionEstimator()->getNumMatches(); }

    bool getChangeReferenceFrames(){ return odom_.getChangeReferenceFrames(); }

    void getMotion(Eigen::Isometry3d &delta, Eigen::MatrixXd &delta_cov, fovis::MotionEstimateStatusCode& delta_status ){
      delta=       odom_.getMotionEstimate();
      delta_cov =  odom_.getMotionEstimateCov();
      delta_status = odom_.getMotionEstimateStatus();
    }

private:
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<fovis::StereoCalibration> kcal_;
    fovis::VisualOdometry odom_;
    //fovis::PrimeSenseDepth depth_producer_;
    fovis::StereoDepth* depth_producer_;
    fovis::DepthImage* depth_image_;
    float* depth_data_;
    Eigen::Isometry3d pose_;
    fovis::StereoCalibration* default_config();
    static fovis::VisualOdometryOptions getDefaultOptions();
    int64_t current_timestamp_,prev_timestamp_;
};

#endif
