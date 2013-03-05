#include "vofeatures.hpp"

using namespace cv;

VoFeatures::VoFeatures(boost::shared_ptr<lcm::LCM> &lcm_, int image_width_, int image_height_):
  lcm_(lcm_), image_width_(image_width_), image_height_(image_height_), utime_(0), output_counter_(0)   {

  if(!lcm_->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }

  // Vis Config:
  // mfallon:
  pc_vis_ = new pointcloud_vis( lcm_->getUnderlyingLCM() );
  float colors_g[] ={0.0,1.0,0.0};
  vector <float> colors_v_g;
  colors_v_g.assign(colors_g,colors_g+4*sizeof(float));
  float colors_b[] ={0.0,0.0,1.0};
  vector <float> colors_v_b;
  colors_v_b.assign(colors_b,colors_b+4*sizeof(float));
  int reset =1;
  
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Reference Camera Pose",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3001,"Reference Features",1,1, 3000,1,colors_v_g));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3002,"Current Camera Pose",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3003,"Reference Camera Features",1,reset, 3002,1,colors_v_b));
}

// TODO: remove fovis dependency entirely:
void VoFeatures::setFeatures(const fovis::FeatureMatch* matches, int num_matches, int64_t utime){
  utime_ = utime;

//  ////////////////// Features: ////////////////////////////////////////////
//  const fovis::MotionEstimator* motion = _odom->getMotionEstimator();
//  int num_matches = motion->getNumMatches();
//  const fovis::FeatureMatch* matches = motion->getMatches();

  features_ref_.clear();
  features_ref_indices_.clear();
  features_cur_.clear();
  features_cur_indices_.clear();

  // This gets features that were matched but not necessarily inliers.
  // TODO also get features that were not even matched, if it helps.
  int feature_index = 0;

  //cout << "num_matches: " << num_matches << "\n";
  for (int i=0; i < num_matches; ++i) {
    const fovis::FeatureMatch & m(matches[i]);
    // @todo move the border removal outside, because it depends on the feature descriptor.
    //       these limits are for the BRIEF descriptor.
    int margin = 30;

    // current frame is  Ref frame and fa/dA
    //RawFrame::ConstPtr raw_ref_frame = ref_frame->raw_frame();

    if (   m.ref_keypoint->base_uv[0] <= margin
        || m.ref_keypoint->base_uv[0] >= (image_width_-margin)
        || m.ref_keypoint->base_uv[1] <= margin
        || m.ref_keypoint->base_uv[1] >= (image_height_-margin)) continue;

    //if (   m.target_keypoint->base_uv[0] <= margin
    //    || m.target_keypoint->base_uv[0] >= (width-margin)
    //    || m.target_keypoint->base_uv[1] <= margin
    //    || m.target_keypoint->base_uv[1] >= (height-margin)) continue;

    ImageFeature fA, fB;
    float dA =m.ref_keypoint->disparity; // compute_disparity_(m.ref_keypoint->disparity);
    float dB = m.target_keypoint->disparity;
    //float dB = compute_disparity_(m.target_keypoint->disparity);

    // Workaround for the fovis DepthImage depth source
    // was: if (isnan(dA)) dA = compute_disparity_(m.ref_keypoint->xyz(2));
    // was: if (isnan(dB)) dB = compute_disparity_(m.target_keypoint->xyz(2));
    if (isnan(dA)) dA = m.ref_keypoint->xyz(2); // stereo disparity is 1-to-1
    if (isnan(dB)) dB = m.target_keypoint->xyz(2);

    //if (1==1){//was
    if(m.inlier){ // if the feature is an inlier - very important distinciton, mfallon
      fA.track_id = fB.track_id = m.track_id;

      /*
      bool use_refined = false;
      if (use_refined && m.status == fovis::MATCH_OK)
      {
        fA.xyz = m.ref_keypoint->xyz;
        fB.xyz = m.refined_target_keypoint.xyz;

        fA.xyzw = m.ref_keypoint->xyzw;
        fB.xyzw = m.refined_target_keypoint.xyzw;

        fA.uv = Eigen::Vector2d(m.ref_keypoint->kp.u, m.ref_keypoint->kp.v);
        fB.uv = Eigen::Vector2d(m.refined_target_keypoint.kp.u, m.refined_target_keypoint.kp.v);

        fA.base_uv = m.ref_keypoint->base_uv;
        fB.base_uv = m.refined_target_keypoint.base_uv;

        fA.uvd = Eigen::Vector3d(m.ref_keypoint->rect_base_uv[0], m.ref_keypoint->rect_base_uv[1], dA);
        fB.uvd = Eigen::Vector3d(m.refined_target_keypoint.rect_base_uv[0], m.refined_target_keypoint.rect_base_uv[1], dB);
      }
      else
      {*/
        fA.xyz = m.ref_keypoint->xyz;
        fA.xyzw = m.ref_keypoint->xyzw;
        fA.uv = Eigen::Vector2d(m.ref_keypoint->kp.u, m.ref_keypoint->kp.v);
        fA.base_uv = m.ref_keypoint->base_uv;
        fA.uvd = Eigen::Vector3d(m.ref_keypoint->rect_base_uv[0], m.ref_keypoint->rect_base_uv[1], dA);

        fB.xyz = m.target_keypoint->xyz;
        fB.xyzw = m.target_keypoint->xyzw;
        fB.uv = Eigen::Vector2d(m.target_keypoint->kp.u, m.target_keypoint->kp.v);
        fB.base_uv = m.target_keypoint->base_uv;
        fB.uvd = Eigen::Vector3d(m.target_keypoint->rect_base_uv[0], m.target_keypoint->rect_base_uv[1], dB);
      //}
      features_ref_.push_back(fA);
      features_cur_.push_back(fB);

      if (m.inlier) {
        features_ref_indices_.push_back(1);
        features_cur_indices_.push_back(1);
      }else{
        features_ref_indices_.push_back(0);
        features_cur_indices_.push_back(0);
      }
      feature_index++;
    }
  }
}



// reference_or_current = 0 send reference (at the change of a key frame change)
// reference_or_current = 1 send current (otherwise)
void VoFeatures::doFeatureProcessing(int reference_or_current){
  if (1==0){ // this only works for old approach
    writeReferenceImages();
    writeFeatures(features_ref_);
    writePose();
  }

  
  if( reference_or_current==0){ // reference
    sendFeatures(features_ref_,features_ref_indices_, "FEATURES_REF");
    Isometry3dTime ref_camera_poseT = Isometry3dTime(utime_, ref_camera_pose_);
    sendImage("REF_LEFT",reference_or_current);
  
    // Send Features paired with the pose at the ref frame:
    pc_vis_->pose_to_lcm_from_list(3000, ref_camera_poseT);
    sendFeaturesAsCollection(features_ref_, features_ref_indices_, 3001); // red, most recent 

    //pc_vis_->pose_to_lcm_from_list(3002, ref_camera_poseT);
    //sendFeaturesAsCollection(features_ref_, features_ref_indices_, 3003); // blue, last 
  }else{ // current
    sendFeatures(features_cur_,features_cur_indices_,"FEATURES_CUR");
    Isometry3dTime cur_camera_poseT = Isometry3dTime(utime_, cur_camera_pose_);
    sendImage("CUR_LEFT",reference_or_current);

    // Send Features paired with the pose at the ref frame:
    //pc_vis_->pose_to_lcm_from_list(3000, cur_camera_poseT);
    //sendFeaturesAsCollection(features_cur_, features_cur_indices_, 3001); // red, most recent 

    pc_vis_->pose_to_lcm_from_list(3002, cur_camera_poseT);
    sendFeaturesAsCollection(features_cur_, features_cur_indices_, 3003); // blue, last 
  }

  
  output_counter_++;
}


void VoFeatures::drawFeaturesOnImage(cv::Mat &img, int which_image ){
  cv::Point p;
  p.x =3; p.y = 10;
  CvScalar color_out = CV_RGB(255,0,0);
  
  if (which_image==0){
    for (size_t j=0;j< features_ref_.size(); j++){
      if (features_ref_indices_[j]){
        cv::Point p0;
        p0.x = features_ref_[j].uv[0]; 
        p0.y = features_ref_[j].uv[1];
        cv::circle( img, p0, 5, color_out, 0 ); 
      }
    }  
  }else{
    for (size_t j=0;j< features_cur_.size(); j++){
      if (features_cur_indices_[j]){
        cv::Point p0;
        p0.x = features_cur_[j].uv[0]; 
        p0.y = features_cur_[j].uv[1];
        cv::circle( img, p0, 5, color_out, 0 ); 
      }
    }  
  }
}

void VoFeatures::sendImage(std::string channel, int which_image ){
  Mat img = Mat::zeros( image_height_, image_width_,CV_8UC1);
  if (which_image == 0){
    img.data = left_ref_buf_;
  }else{
    img.data = left_cur_buf_;
  }

  drawFeaturesOnImage(img, which_image);
  
  int n_colors=1;

  bot_core_image_t image;
  image.utime =0;
  image.width = img.cols;
  image.height= img.rows;
  image.row_stride =n_colors*img.cols;
  image.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  image.size =n_colors*img.cols*img.rows;
  image.data = img.data;
  image.nmetadata =0;
  image.metadata = NULL;
  bot_core_image_t_publish( lcm_->getUnderlyingLCM() , channel.c_str(), &image);    
}


void VoFeatures::writeReferenceImages(){
  //cout << "images written to file @ " << utime_ << "\n";
  stringstream ss;
  char buff[10];
  sprintf(buff,"%0.4d",output_counter_);
  ss << buff << "_" << utime_;
  Mat img = Mat::zeros( image_height_, image_width_,CV_8UC1);
  img.data = left_ref_buf_;
  imwrite( ( ss.str() + "_left.png"), img);

  //Mat imgR = Mat::zeros( image_height_, image_width_,CV_8UC1);
  //imgR.data = right_ref_buf_;
  //imwrite( (ss.str() + "_right.png"), imgR);  
}

void VoFeatures::writeFeatures(std::vector<ImageFeature> features){
  stringstream ss;
  char buff[10];
  sprintf(buff,"%0.4d",output_counter_);
  ss << buff << "_" << utime_;

  std::fstream feat_file;
  string fname = string(  ss.str() + ".feat");
  feat_file.open(  fname.c_str() , std::fstream::out);
//  cout << "nmatches written to file: "<< features.size() << " @ " << utime_ << "\n";
  feat_file << "#i,track_id,uv,base_uv,uvd,xyz,xyzw,color\n";
  for (size_t i = 0; i < features.size(); i++) {
    ImageFeature f = features[i];
    ostringstream temp2;
    temp2 << i << ","
        << f.track_id << ","
        << f.uv[0] << "," << f.uv[1] << "," // actual pixel locations of features
        << f.base_uv[0] << "," << f.base_uv[1] << ","
        << f.uvd[0] << "," << f.uvd[1] << "," << f.uvd[2] << ","
        << f.xyz[0] << "," << f.xyz[1] << "," << f.xyz[2] << ","
        << f.xyzw[0] << "," << f.xyzw[1] << "," << f.xyzw[2] << "," << f.xyzw[3] << ","
        << (int) f.color[0] << "," << (int) f.color[1] << "," << (int) f.color[2] << "\n";
    feat_file << temp2.str() ;
  }
  feat_file.close();
}


void VoFeatures::writePose(){
  stringstream ss;
  char buff[10];
  sprintf(buff,"%0.4d",output_counter_);
  ss << buff << "_" << utime_;

  std::fstream feat_file;
  string fname = string(  ss.str() + ".pose");
  feat_file.open(  fname.c_str() , std::fstream::out);
  feat_file << "#utime,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z\n";
  
  Eigen::Vector3d t(ref_camera_pose_.translation());
  Eigen::Quaterniond r(ref_camera_pose_.rotation());
  feat_file <<utime_<< "," << t[0]<<","<<t[1]<<","<<t[2]<<"," 
       <<r.w()<<","<<r.x()<<","<<r.y()<<","<<r.z() ;
  
  
  
  feat_file.close();
}

void VoFeatures::sendFeaturesAsCollection(std::vector<ImageFeature> features, 
                                          std::vector<int> features_indices,
                                          int vs_id){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for (size_t i = 0; i < features.size(); i++) {
    if (features_indices[i]){
      ImageFeature f = features[i];
      pcl::PointXYZRGB pt;
      pt.x = f.xyz[0];
      pt.y = f.xyz[1];
      pt.z = f.xyz[2];
      pt.r =0; pt.g =0; pt.b =255.0; // blue
      cloud->points.push_back(pt);
    }
  }
  pc_vis_->ptcld_to_lcm_from_list(vs_id, *cloud, utime_, utime_);
}

void VoFeatures::sendFeatures(std::vector<ImageFeature> features, 
                              std::vector<int> features_indices, 
                              std::string channel){
  
  reg::features_t msg;
  msg.utime = utime_;
  for (size_t i = 0; i < features.size(); i++) {
    if (features_indices[i]){
      reg::feature_t* feat(new reg::feature_t());
      ImageFeature f = features[i];
      feat->track_id = f.track_id;
      feat->uv[0] = f.uv[0];
      feat->uv[1] = f.uv[1];
      feat->base_uv[0] = f.base_uv[0];
      feat->base_uv[1] = f.base_uv[1];
      feat->uvd[0] = f.uvd[0];
      feat->uvd[1] = f.uvd[1];
      feat->uvd[2] = f.uvd[2];
      feat->xyz[0] = f.xyz[0];
      feat->xyz[1] = f.xyz[1];
      feat->xyz[2] = f.xyz[2];
      feat->xyzw[0] = f.xyzw[0];
      feat->xyzw[1] = f.xyzw[1];
      feat->xyzw[2] = f.xyzw[2];
      feat->xyzw[3] = f.xyzw[3];
      // color left out for now
      msg.features.push_back(*feat);
    }
  }
  msg.nfeatures= msg.features.size();

  Eigen::Vector3d t(ref_camera_pose_.translation());
  Eigen::Quaterniond r(ref_camera_pose_.rotation());
  msg.pos[0] = t[0];
  msg.pos[1] = t[1];
  msg.pos[2] = t[2];
  msg.quat[0] = r.w();
  msg.quat[1] = r.x();
  msg.quat[2] = r.y();
  msg.quat[3] = r.z();
  lcm_->publish(channel.c_str(), &msg);
}

