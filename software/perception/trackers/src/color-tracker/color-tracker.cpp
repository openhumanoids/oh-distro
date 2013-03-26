#include "color-tracker.hpp"
#include <iostream>

using namespace cv;
using namespace std;

ColorTracker::ColorTracker(boost::shared_ptr<lcm::LCM> &lcm_, 
                               int width_, int height_, 
                               double fx_, double fy_, double cx_, double cy_): 
                               lcm_(lcm_),
                               width_(width_),height_(height_),
                               fx_(fx_), fy_(fy_), cx_(cx_), cy_(cy_){

  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451002,"Tracker | NPose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451003,"Tracker | Plane"           ,3,1, 4451002,1, { 1.0, 0.0, 0.0} ));

  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4451004,"Tracker | Plane X"           ,1,1, 4451002,1, { 0.0, 0.0, 1.0} ));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4451005,"Tracker | Transform",5,1) );

  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), width_, height_ );
  
  // assumes we are tracking red object for now
  mode_ =0;
}


// Convert the image into an HSV image
IplImage* ColorTracker::GetThresholdedImage(IplImage* img){ 
  IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
  cvCvtColor(img, imgHSV, CV_RGB2HSV); // NB: conversion from typical LCM to HSV
  IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

  //GIMP        H = 0-360, S = 0-100 and V = 0-100. But 
  //OpenCV uses H: 0 - 180, S: 0 - 255, V: 0 - 255
  //OpenCV uses BGR format, not RGB
  // conversion from GIMP to OpenCV:  H/2  //S*2.55  //V*2.55  
  // Values 20,100,100 to 30,255,255 working perfect for yellow at around 6pm
  //cvInRangeS(imgHSV, cvScalar(112, 100, 100), cvScalar(124, 255, 255), imgThreshed);
  // Yellow:
  //cvInRangeS(imgHSV, cvScalar(0, 100, 100), cvScalar(30, 255, 255), imgThreshed);
  // Orange (tropicana:
  //cvInRangeS(imgHSV, cvScalar(10, 50, 50), cvScalar(15, 255, 255), imgThreshed);
  // red bowl:
  //  cvInRangeS(imgHSV, cvScalar(0, 50, 50), cvScalar(8, 255, 255), imgThreshed);
  // green (top of lemon juice):
  //cvInRangeS(imgHSV, cvScalar(55, 50, 50), cvScalar(65, 255, 255), imgThreshed);
  
  if(mode_==0){
    // red valve in VRC:
    cvInRangeS(imgHSV, cvScalar(0, 100, 1), cvScalar(5, 255, 60), imgThreshed);
  }else if (mode_==1){
    // green wheel in VRC:
    cvInRangeS(imgHSV, cvScalar(55, 50, 50), cvScalar(65, 255, 255), imgThreshed);
   cout << "green color\n"; 
  }

  cvReleaseImage(&imgHSV);
  return imgThreshed;
}



//std::vector<float> ColorTracker::ColorTracker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pts, uint8_t* img_data,
std::vector<float> ColorTracker::doColorTracker(std::vector< Eigen::Vector3d > & pts, uint8_t* img_data,
                                        Eigen::Isometry3d local_to_camera, int64_t current_utime){
  
  std::vector<float> loglikelihoods;
  loglikelihoods.assign ( pts.size() ,0);    

  Mat src= Mat::zeros( height_, width_  ,CV_8UC3);
  src.data = img_data;
  IplImage* frame = new IplImage(src);

  // 1. Threshold the image in HSV space (object = white, rest = black)
  IplImage* imgColorThresh = GetThresholdedImage(frame);
  // Calculate the moments to estimate the position of the ball
  CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
  cvMoments(imgColorThresh, moments, 1);
  // The actual moment values
  double moment10 = cvGetSpatialMoment(moments, 1, 0);
  double moment01 = cvGetSpatialMoment(moments, 0, 1);
  double area = cvGetCentralMoment(moments, 0, 0);
  // Holding the last and current ball positions
  int u_estimated = moment10/area;
  int v_estimated = moment01/area;

  // Print it out for debugging purposes
  cout << "est: " << u_estimated << " " << v_estimated << " | area: " << area << "\n";
  double min_area = 0; // 30 worked previous for real data
  if ((area> min_area) && (u_estimated>0 && v_estimated>0)){
    // Valid measurement
  }else{
   cout << "color not seen - returning ["<< area <<"]\n"; 
   return loglikelihoods;
  }
  

  // 2. Project particles into camera frame:
  Eigen::Affine3d transform;
  transform.setIdentity();
  Eigen::Translation3d translation(local_to_camera.translation());
  Eigen::Quaterniond quat(local_to_camera.rotation());
  transform = transform * translation * quat;
  for (size_t i = 0; i < pts.size (); ++i){
    pts[i] = transform*pts[i];
  }
  
  // 3. Determine Likelihood in Image space:
  for (size_t i=0; i< pts.size(); i++) {
    // u = pt.x fx/pt.z   ... project point to pixel
    Eigen::Vector3d pt1 = pts[i];
    int u = floor( ((pt1[0] * fx_)/pt1[2]) + cx_);
    int v = floor( ((pt1[1] * fy_)/pt1[2]) + cy_);
    int dist = sqrt( pow( u - u_estimated ,2) + pow( v - v_estimated ,2) );
    // Crude Binary Likelihood:
    if (dist < 13){
      loglikelihoods[i] =1; //was 1
    }
  }    
  
  
  // Debug Output:
  if (1==0){
    // Visualise projected points (in a camera frame)
    Eigen::Isometry3d null_pose;
    null_pose.setIdentity();
    Isometry3dTime null_poseT = Isometry3dTime(current_utime, null_pose);
    pc_vis_->pose_to_lcm_from_list(4451000, null_poseT);  
    //pc_vis_->ptcld_to_lcm_from_list(4451001, *pts, current_utime, current_utime);  
  }
    
  imgutils_->sendImage( (uint8_t*) imgColorThresh->imageData, current_utime, width_, 
                        height_, 1, "TRACKER_THRESH"  );
  cv::Mat imgMat(frame);
  for (size_t i=0; i< pts.size(); i++) {
    Eigen::Vector3d pt1 = pts[i];
    int u = floor( ((pt1[0] * fx_)/pt1[2])  +  512.5);
    int v = floor( ((pt1[1] * fy_)/pt1[2]) + 272.5);
    Point center( u, v );
    circle( imgMat, center, 3, Scalar(0,255,0), -1, 8, 0 );
  }  
  Point center( u_estimated, v_estimated );
  circle( imgMat, center, 3, Scalar(255,0,0), -1, 8, 0 );
  imgutils_->sendImage(imgMat.data, current_utime, width_, 
                        height_, 3, "TRACKER_TRACKING"  );
  
  return loglikelihoods;
}
