//drc-descript-brief 1225720333142477 1225720333192471

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

#include <GL/gl.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include <bot_lcmgl_client/lcmgl.h>

#include <estimate-pose/pose_estimator.hpp>

#include <pointcloud_tools/pointcloud_vis.hpp>

using namespace std;

bot_lcmgl_t* _lcmgl;
lcm_t* _bot_param_lcm;
pointcloud_vis* pc_vis_;

struct ImageFeature{
  int track_id;
  Eigen::Vector2d uv; ///< unrectified, distorted, orig. coords
  Eigen::Vector2d base_uv; ///< unrectified, distorted, base level
  Eigen::Vector3d uvd; ///< rectified, undistorted, base level
  Eigen::Vector3d xyz;
  Eigen::Vector4d xyzw;
  uint8_t color[3];

  // @todo what more is needed?
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FrameMatch{
  std::vector<int> featuresA_indices;
  std::vector<int> featuresB_indices;

  std::vector<ImageFeature> featuresA;
  std::vector<ImageFeature> featuresB;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef boost::shared_ptr<FrameMatch> FrameMatchPtr;


void draw_reg(std::vector<ImageFeature> features,    int status, Eigen::Isometry3d pose)
{
  // reorder as:
  // z, -x, -y (or 2,-0,-1)

  bot_lcmgl_push_matrix(_lcmgl);
  bot_lcmgl_rotated(_lcmgl, -90, 0, 0, 1);
  bot_lcmgl_rotated(_lcmgl, -90, 1, 0, 0);


//  bot_lcmgl_translated(_lcmgl, 0, pose.translation().y(), 0);


  bot_lcmgl_color3f(_lcmgl, 0, 1, 1);
  bot_lcmgl_point_size(_lcmgl, 1.5f);
  bot_lcmgl_begin(_lcmgl, GL_POINTS);

  if (status==0){
    bot_lcmgl_color3f(_lcmgl, 0, 0, 1);
  }else{ // status 1 when changing key frames:
    bot_lcmgl_color3f(_lcmgl, 1, 0, 0);
  }
  for (size_t i=0; i < features.size(); ++i) {
    ImageFeature f = features[i];

    bot_lcmgl_vertex3f(_lcmgl, f.xyz[0],  f.xyz[1], f.xyz[2]);
//    bot_lcmgl_vertex3f(_lcmgl, f.xyz[2],  -f.xyz[0],-f.xyz[1]);
  }
  bot_lcmgl_end(_lcmgl);
  bot_lcmgl_pop_matrix(_lcmgl);

  // disabling this allows draw_reg to daisy chain onto draw()
  //bot_lcmgl_switch_buffer(_lcmgl);
}

void draw_both_reg(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
    Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1){
  draw_reg(features0,    0,  pose0);
  draw_reg(features1,    1, pose1);
  bot_lcmgl_switch_buffer(_lcmgl);
}


void features2cloud(std::vector<ImageFeature> features, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  cloud->width   = features.size();
  cloud->height   = 1;
  cloud->points.resize (cloud->width  *cloud->height);
  for (size_t i=0;i<features.size(); i++){
    cloud->points[i].x = features[i].xyz[0];
    cloud->points[i].y = features[i].xyz[1];
    cloud->points[i].z = features[i].xyz[2];
    cloud->points[i].r = 1;
    cloud->points[i].g = 1;
    cloud->points[i].b = 1;
  }
}

// Send a subset of the feature set:
void features2cloud(std::vector<ImageFeature> features, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    std::vector<int> features_indices){
  cloud->width   = features_indices.size();
  cloud->height   = 1;
  cloud->points.resize (cloud->width  *cloud->height);
  for (size_t j=0;j<features_indices.size(); j++){
    int i = features_indices[j];
    cloud->points[j].x = features[i].xyz[0];
    cloud->points[j].y = features[i].xyz[1];
    cloud->points[j].z = features[i].xyz[2];
    cloud->points[j].r = 1;
    cloud->points[j].g = 1;
    cloud->points[j].b = 1;
  }
}



void send_both_reg(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
    Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1){

  Eigen::Matrix3d M;
  M <<  0,  0, 1,
       -1,  0, 0,
        0, -1, 0;
  pose0 = M * pose0;
  pose1 = M * pose1;

  int64_t pose0_id = 1;
  Isometry3dTime pose0T = Isometry3dTime(pose0_id, pose0);
  pc_vis_->pose_to_lcm_from_list(1000, pose0T);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  features2cloud(features0,cloud0);
  pc_vis_->ptcld_to_lcm_from_list(1001, *cloud0, pose0_id, pose0_id);


  int64_t pose1_id = 1;
  Isometry3dTime pose1T = Isometry3dTime(pose1_id, pose1);
  pc_vis_->pose_to_lcm_from_list(2000, pose1T);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  features2cloud(features1,cloud1);
  pc_vis_->ptcld_to_lcm_from_list(2001, *cloud1, pose1_id, pose1_id);

}


void send_both_reg_inliers(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
    Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1,
    std::vector<int> feature_inliers0,    std::vector<int> feature_inliers1 ){

  Eigen::Matrix3d M;
  M <<  0,  0, 1,
       -1,  0, 0,
        0, -1, 0;
  pose0 = M * pose0;
  pose1 = M * pose1;

  int64_t pose0_id = 1;
  Isometry3dTime pose0T = Isometry3dTime(pose0_id, pose0);
  pc_vis_->pose_to_lcm_from_list(1000, pose0T);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  features2cloud(features0,cloud0,feature_inliers0);
  pc_vis_->ptcld_to_lcm_from_list(1002, *cloud0, pose0_id, pose0_id);


  int64_t pose1_id = 1;
  Isometry3dTime pose1T = Isometry3dTime(pose1_id, pose1);
  pc_vis_->pose_to_lcm_from_list(2000, pose1T);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  features2cloud(features1,cloud1,feature_inliers1);
  pc_vis_->ptcld_to_lcm_from_list(2002, *cloud1, pose1_id, pose1_id);

}








// 'lexicographic' comparison
static bool DMatch_lt(const cv::DMatch& a, const cv::DMatch& b) {
  //return ( (a.trainIdx < b.trainIdx) || (a.queryIdx < b.queryIdx) );
  if (a.trainIdx != b.trainIdx) { return a.trainIdx < b.trainIdx; }
  return a.queryIdx < b.queryIdx;
}

void features_to_keypoints(const std::vector<ImageFeature> & features, std::vector<cv::KeyPoint> & kpts) {
  kpts.clear();
  for(std::vector<ImageFeature>::const_iterator it=features.begin(); it != features.end(); ++it) {
    const ImageFeature & f = *it;
    // @todo use Mei's true scale here
    //    kpts.push_back(cv::KeyPoint(f.uvd(0), f.uvd(1), 1.0));
    kpts.push_back(cv::KeyPoint(f.base_uv(0), f.base_uv(1), 20.0)); // was in hordur's code
  }
}

void compute_descriptors(cv::Mat &image, vector<ImageFeature> & features, std::string & name, cv::DescriptorExtractor & extractor,
    cv::Mat &descriptors, std::vector<cv::KeyPoint>& keypoints){
  features_to_keypoints(features, keypoints);
  extractor.compute(image, keypoints, descriptors);
}


void read_log(std::string fname,
    std::vector<ImageFeature>& features ){

  printf( "About to read\n%s\n",fname.c_str());
  int counter=0;
  string line0;
  ifstream myfile (fname.c_str());
  if (myfile.is_open()){

    getline (myfile,line0);
    //cout << line0 << " is first line\n";

    counter =0;
    while ( myfile.good() ){
      string line;
      getline (myfile,line);
      if (line.size() > 4){
        ImageFeature f;
        int i,track_id;
        double v[15];
        int d[3];
        int res = sscanf(line.c_str(), "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d",&i,
            &(v[0]), &(v[1]), &(v[2]), &(v[3]), &(v[4]), // id, uv, base_uv
            &(v[5]), &(v[6]), &(v[7]),  &(v[8]), &(v[9]), &(v[10]),// uvd xyz
            &(v[11]), &(v[12]), &(v[13]), &(v[14]), // xyzd
            &(d[0]), &(d[1]), &(d[2])        );

        f.track_id=v[0]; f.uv[0]=v[1];f.uv[1]=v[2];
        f.base_uv[0]=v[3];f.base_uv[1]=v[4];
        f.uvd[0]=v[5];f.uvd[1]=v[6];f.uvd[2]=v[7];
        f.xyz[0]=v[8];f.xyz[1]=v[9];f.xyz[2]=v[10];
        f.xyzw[0]=v[11];f.xyzw[1]=v[12];f.xyzw[2]=v[13];f.xyzw[3]=v[14];
        f.color[0] = d[0];f.color[1] = d[1];f.color[2] = d[2];

        /*

        cout << line << " is line\n";
        cout << "i: " << i <<"\n";
        cout << "f.track_id: " << f.track_id <<"\n";
        cout << "f.uv: " << f.uv[0] << " "<< f.uv[1] <<"\n";
        cout << "f.base_uv: " << f.base_uv[0] << " "<< f.base_uv[1] <<"\n";
        cout << "f.uvd: " << f.uvd[0] << " "<< f.uvd[1]<< " "<< f.uvd[2]<<"\n";
        cout << "f.xyz: " << f.xyz[0] << " "<< f.xyz[1]<< " "<< f.xyz[2]<<"\n";
        cout << "f.xyzw: " << f.xyzw[0] << " "<< f.xyzw[1]<< " "<< f.xyzw[2]<< " "<< f.xyzw[3]<<"\n";
        cout << "f.color: " << (int)f.color[0] << " "<< (int)f.color[1] << " "<< (int)f.color[2] <<"\n";
         */
        features.push_back(f);
      }
    }
    myfile.close();
  } else{
    printf( "Unable to open poses file\n%s",fname.c_str());
    return;
  }
  cout << "read " << features.size() << " features\n";
}


Eigen::Isometry3d pose_estimate(FrameMatchPtr match,
    std::vector<char> & inliers,
    Eigen::Isometry3d & motion,
    Eigen::MatrixXd & motion_covariance,
    Eigen::Matrix<double, 3, 4> & proj_matrix) {
  using namespace pose_estimator;

  cout << "pt0\n";
  PoseEstimator pe(proj_matrix);
  cout << "pt1\n";

  if ((match->featuresA_indices.size()!=match->featuresB_indices.size()))
    cout <<    "Number of features doesn't match\n";

  size_t num_matches = match->featuresA_indices.size();

  if(num_matches < 3)
    cout << "Need at least three matches to estimate pose";

  cout << "pt2\n";

  motion.setIdentity();
  motion_covariance.setIdentity();

  Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::ColMajor> src_xyzw(4, num_matches);
  Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::ColMajor>dst_xyzw(4, num_matches);
  for (size_t i=0; i < num_matches; ++i) {

    //    const ImageFeature& featureA(int i) const {
    //      assert (frameA);
    //    int ix = featuresA_indices.at(i);
    //      return frameA->features().at(ix);
    //    }
    //src_xyzw.col(i) = match->featureA(i).xyzw;
    //dst_xyzw.col(i) = match->featureB(i).xyzw;

    int ixA = match->featuresA_indices.at(i);
    int ixB = match->featuresB_indices.at(i);
    //cout << ixA << " | " << ixB << "\n";
    //cout << match->featuresA.size() << " fA size\n";
    //cout <<  match->featuresA[ixA].xyzw[0] << "\n";
    //cout <<  match->featuresA[ixA].xyzw[0] << "\n";

    src_xyzw.col(i) = match->featuresA[ixA].xyzw;
    dst_xyzw.col(i) = match->featuresB[ixB].xyzw;
  }
  cout << "pt3\n";


  // PoseEstimateStatus status = pe.estimate(src_xyzw, dst_xyzw, &inliers, &motion, &motion_covariance);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> motion_covariance_col_major;

  cout << "pt4\n";
  PoseEstimateStatus status = pe.estimate(src_xyzw, dst_xyzw, &inliers, &motion, &motion_covariance_col_major); //&motion_covariance);
  motion_covariance = motion_covariance_col_major;

  /*
  int num_inliers = std::accumulate(inliers.begin(), inliers.end(), 0);
  const char* str_status = PoseEstimateStatusStrings[status];
  std::cerr << "Motion: " << str_status << " feats: " << match->featuresA_indices.size()
            << " inliers: " << num_inliers
            << " Pose: " << motion
            << " Delta: " << match->delta
            //<< " Cov:\n" << motion_covariance << "\n"
            << " " << match->timeA << " " << match->timeB << std::endl;
   */

  return motion;
}



// main
int main( int argc, char** argv ) {
  _bot_param_lcm = lcm_create(NULL);
  _lcmgl = bot_lcmgl_init(_bot_param_lcm, "descript-brief");

  // Vis Config:
  pc_vis_ = new pointcloud_vis(_bot_param_lcm);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  vector <float> colors_v;
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose - 0",5,0) );
//  pc_vis_->obj_cfg_list.push_back( obj_cfg(800,"Pose - Laser Map Init",5,0) );
  float colors_b[] ={0.0,0.0,1.0};
  colors_v.assign(colors_b,colors_b+4*sizeof(float));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud - 0"     ,1,1, 1000,1,colors_v));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1002,"Cloud - 0 - inliers"     ,1,1, 1000,1,colors_v));


  float colors_r[] ={1.0,0.0,0.0};
  colors_v.assign(colors_r,colors_r+4*sizeof(float));
  pc_vis_->obj_cfg_list.push_back( obj_cfg(2000,"Pose - 1",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2001,"Cloud - 1"     ,1,1, 1000,1,colors_v));
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2002,"Cloud - 1 - inliers"     ,1,1, 1000,1,colors_v));


  stringstream ifile0, ifile1, featfile0, featfile1;
  ifile0 << argv[1] << "_left.png";
  featfile0 << argv[1] << ".feat";
  ifile1 << argv[2] << "_left.png";
  featfile1 << argv[2] << ".feat";

  // 1. Read in imgs and featues:
  cv::Mat img0 = cv::imread( ifile0.str(), CV_LOAD_IMAGE_GRAYSCALE );
  cv::Mat img1 = cv::imread( ifile1.str(), CV_LOAD_IMAGE_GRAYSCALE );
  std::vector<ImageFeature> features0,features1;
  std::vector<cv::KeyPoint> keypoints0, keypoints1;
  cv::Mat descriptors0, descriptors1;
  read_log(featfile0.str(), features0);
  read_log(featfile1.str(), features1);

  Eigen::Isometry3d dummypose0;
  dummypose0.setIdentity();
  Eigen::Isometry3d dummypose1;
  dummypose1.setIdentity();
  dummypose1.translation().y() = 10;
  //draw_both_reg(features0, features1,dummypose0,dummypose1);

  //2. Extract Image Descriptors:
  cv::BriefDescriptorExtractor extractor(32); // size of descriptor in bytes
  std::string desc_name= "brief";
  compute_descriptors(img0, features0, desc_name, extractor,descriptors0,keypoints0);
  cout << descriptors0.rows << "descripters in 0\n";
  compute_descriptors(img1, features1, desc_name, extractor,descriptors1,keypoints1);
  cout << descriptors1.rows << "descripters in 1\n";


  //3: Matching descriptor vectors with a brute force matcher
  cv::BFMatcher matcher(cv::NORM_HAMMING); // used by hordur
  std::vector< cv::DMatch > matches0in1,matches1in0;
  matcher.match(descriptors0, descriptors1, matches0in1); // each feature in 0 found in 1
  matcher.match(descriptors1, descriptors0, matches1in0); // each feature in 1 found in 0
  BOOST_FOREACH (cv::DMatch& match, matches1in0) {
    std::swap(match.trainIdx, match.queryIdx);
  }

  // 3b keep intersection, aka the mutual best matches.
  std::sort(matches0in1.begin(), matches0in1.end(), DMatch_lt);
  std::sort(matches1in0.begin(), matches1in0.end(), DMatch_lt);
  std::vector<cv::DMatch> matches;
  std::set_intersection(matches0in1.begin(), matches0in1.end(),
      matches1in0.begin(), matches1in0.end(),
      std::back_inserter(matches),
      DMatch_lt);
  std::cout << matches0in1.size() << " matches0in1 found\n";
  std::cout << matches1in0.size() << " matches1in0 found\n";
  std::cout << matches.size() << " intersect matches found\n";

  // 3c Draw descriptor matches
  cv::Mat img_matches0in1, img_matches1in0;
  std::cout << "blah4\n";
  cv::drawMatches( img0, keypoints0, img1, keypoints1, matches0in1, img_matches0in1 );
  imshow("Matches0in1", img_matches0in1 );

  cv::drawMatches( img0, keypoints0, img1, keypoints1, matches1in0, img_matches1in0  );
  imshow("Matches1in0 [matches1in0 reordered]", img_matches1in0 );

  cv::Mat img_matches_inter;
  cv::drawMatches( img0, keypoints0, img1, keypoints1, matches, img_matches_inter );
  imshow("Matches Intersection", img_matches_inter );



  // 4 Get delta pose estimate:
  std::vector<int> idxA;
  std::vector<int> idxB;
  for (size_t i = 0; i < descriptors0.rows; ++i)
  {
    //if (1==1){//(desc_A->valid[i]) {
    //descriptors0.row(i).copyTo(descriptors0.row(validA));
    idxA.push_back(i);
    //validA++;
    //}
  }
  for (size_t i = 0; i < descriptors1.rows; ++i)
  {
    //if (desc_B->valid[i]) {
    //descriptorsB.row(validB) = desc_B->descriptors.row(i);
    //desc_B->descriptors.row(i).copyTo(descriptorsB.row(validB));
    idxB.push_back(i);
    //validB++;
    //}
  }

  std::vector<char> inliers;
  int num_inliers = 0;
  Eigen::Isometry3d motion;
  FrameMatchPtr match(new FrameMatch());
  BOOST_FOREACH (const cv::DMatch& dmatch, matches) {
    //match->featuresA_indices.push_back(idxA[dmatch.queryIdx]);
    //match->featuresB_indices.push_back(idxB[dmatch.trainIdx]);

    match->featuresA_indices.push_back(dmatch.queryIdx);
    match->featuresB_indices.push_back(dmatch.trainIdx);
  }


  BOOST_FOREACH (const ImageFeature& feature, features0) {
    match->featuresA.push_back(feature);
  }
  BOOST_FOREACH (const ImageFeature& feature, features1) {
    match->featuresB.push_back(feature);
  }



  BOOST_FOREACH (cv::DMatch& dmatch, matches) {
    //dmatch.queryIdx = idxA[dmatch.queryIdx];
    //dmatch.trainIdx = idxB[dmatch.trainIdx];
  }
  if (matches.size() >= 3) {
    Eigen::Isometry3d delta;
    delta.setIdentity();
    Eigen::MatrixXd covariance;

    Eigen::Matrix<double, 3, 4> projection_matrix;
    // from newcollege_stereo config:
    projection_matrix << 389.956085,  0, 254.903519, 0,
        0, 389.956085,  201.899490, 0,
        0,   0,   1, 0;

    cout << "about to pose_estimate\n";
    delta = pose_estimate(match, inliers, motion, covariance,
        projection_matrix);

    cout << delta.translation().x() << " "
        << delta.translation().y() << " "
        << delta.translation().z() << " \n";


    Eigen::Isometry3d nullpose;
    nullpose.setIdentity();
//    draw_both_reg(features0, features1,nullpose0,delta);
    draw_both_reg(features0, features1,nullpose,delta);

    send_both_reg(features0, features1,delta,nullpose);

    size_t j = 0;
    for (size_t i = 0; i < inliers.size(); ++i)
    {
      if (inliers[i]) {
        match->featuresA_indices[j] = match->featuresA_indices[i];
        match->featuresB_indices[j] = match->featuresB_indices[i];
        j++;
      }
    }
    match->featuresA_indices.resize(j);
    match->featuresB_indices.resize(j);
    cout << j << " inliers found\n";

    send_both_reg_inliers(features0, features1,delta,nullpose, match->featuresA_indices, match->featuresB_indices);
// drc-descript-brief   1225720333942439 1225720585781353


    //std::vector<cv::KeyPoint> keypoints0_in, keypoints1_in;
//    cv::Mat img_matches_inter_inlier;
//    cv::drawMatches( img0, keypoints0, img1, keypoints1_in, matches, img_matches_inter_inlier );
//    imshow("Matches Intersection", img_matches_inter_inlier );


  } else {
    //match->estimation_status = fovis::INSUFFICIENT_INLIERS;
    num_inliers = 0;
  }


  cv::waitKey(0);

  return 0;
}
