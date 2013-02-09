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
  float colors_r[] ={1.0,0.0,0.0};
  vector <float> colors_v_r;
  colors_v_r.assign(colors_r,colors_r+4*sizeof(float));
  float colors_b[] ={0.0,0.0,1.0};
  vector <float> colors_v_b;
  colors_v_b.assign(colors_b,colors_b+4*sizeof(float));
  int reset =0;
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Pose - Feats",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3001,"Feats - Current",1,1, 3000,1,colors_v_r));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3002,"Feats - All",1,reset, 3000,1,colors_v_b));
  
  pc_vis_->obj_cfg_list.push_back( obj_cfg(60000,"Pose - All",5,0) );
  //pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(60001,"Cloud - Laser"         ,1,0, 60000,0, colors_v ));

}

// TODO: remove fovis dependency entirely:
void VoFeatures::setFeatures(const fovis::FeatureMatch* matches, int num_matches, int64_t utime){
  utime_ = utime;

//  ////////////////// Features: ////////////////////////////////////////////
//  const fovis::MotionEstimator* motion = _odom->getMotionEstimator();
//  int num_matches = motion->getNumMatches();
//  const fovis::FeatureMatch* matches = motion->getMatches();

  featuresA.clear();
  featuresA_indices.clear();

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
    //float dB = compute_disparity_(m.target_keypoint->disparity);

    // Workaround for the fovis DepthImage depth source
    // was: if (isnan(dA)) dA = compute_disparity_(m.ref_keypoint->xyz(2));
    if (isnan(dA)) dA = m.ref_keypoint->xyz(2); // stereo disparity is 1-to-1
    //if (isnan(dB)) dB = compute_disparity_(m.target_keypoint->xyz(2));

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
        //fB.xyz = m.target_keypoint->xyz;

        fA.xyzw = m.ref_keypoint->xyzw;
        //fB.xyzw = m.target_keypoint->xyzw;

        fA.uv = Eigen::Vector2d(m.ref_keypoint->kp.u, m.ref_keypoint->kp.v);
        //fB.uv = Eigen::Vector2d(m.target_keypoint->kp.u, m.target_keypoint->kp.v);

        fA.base_uv = m.ref_keypoint->base_uv;
        //fB.base_uv = m.target_keypoint->base_uv;

        fA.uvd = Eigen::Vector3d(m.ref_keypoint->rect_base_uv[0], m.ref_keypoint->rect_base_uv[1], dA);
        //fB.uvd = Eigen::Vector3d(m.target_keypoint->rect_base_uv[0], m.target_keypoint->rect_base_uv[1], dB);
      //}
      featuresA.push_back(fA);
      //featuresB.push_back(fB);

      if (m.inlier) {
        featuresA_indices.push_back(feature_index);
        //frame_match_.featuresB_indices.push_back(feature_index);
      }
      feature_index++;
    }
  }
}

void VoFeatures::sendFeatures(){

        //write_ref_images();
     //   writeFeatures(featuresA);
        sendFeatures(featuresA);
        
        // Send Features paired with the pose at the ref frame:
      Eigen::Isometry3d temp_pose;
      temp_pose.setIdentity();


        Isometry3dTime ref_camera_poseT = Isometry3dTime(utime_, ref_camera_pose_);
        pc_vis_->pose_to_lcm_from_list(3000, ref_camera_poseT);
        sendFeaturesAsCollection(featuresA, 3001); // blue, last 
        sendFeaturesAsCollection(featuresA, 3002); // red, most recent
  output_counter_++;
}

void VoFeatures::write_ref_images(){
  //cout << "images written to file @ " << utime_ << "\n";
  stringstream ss;
  char buff[10];
  sprintf(buff,"%0.4d",output_counter_);
  ss << buff << "_" << utime_;
  Mat img = Mat::zeros( image_height_, image_width_,CV_8UC1);
  img.data = left_buf_;
  imwrite( ( ss.str() + "_left.png"), img);

  Mat imgR = Mat::zeros( image_height_, image_width_,CV_8UC1);
  imgR.data = right_buf_;
  imwrite( (ss.str() + "_right.png"), imgR);  
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

void VoFeatures::sendFeaturesAsCollection(std::vector<ImageFeature> features, int vs_id){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for (size_t i = 0; i < features.size(); i++) {
    ImageFeature f = features[i];
    pcl::PointXYZRGB pt;
    pt.x = f.xyz[0];
    pt.y = f.xyz[1];
    pt.z = f.xyz[2];
    pt.r =0; pt.g =0; pt.b =255.0; // blue
    cloud->points.push_back(pt);
  }
  pc_vis_->ptcld_to_lcm_from_list(vs_id, *cloud, utime_, utime_);
}

void VoFeatures::sendFeatures(std::vector<ImageFeature> features){
  
  reg::features_t msg;
  msg.utime = utime_;
  msg.nfeatures=features.size();
  for (size_t i = 0; i < features.size(); i++) {
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

  lcm_->publish("FEATURES", &msg);
}

