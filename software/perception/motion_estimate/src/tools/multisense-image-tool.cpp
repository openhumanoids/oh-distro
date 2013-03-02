// Tool to take a multisense stereo image (images_t)
// - republish images on seperate channels
// - use disparity image and left and create a point cloud
//
// Various features incomplete as of feb 2013
// e.g. support for rgb, masking, checking if images are actually left or disparity
//      getting the camera calibration from the param server
// mfallon 2013
//
// Example usage:
// drc-multisense-image-tool -p -s -d 4

#include <stdio.h>
#include <lcm/lcm.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ConciseArgs>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#include <bot_core/bot_core.h>
//#include <lcmtypes/multisense.h>
#include <lcmtypes/bot_core.hpp>
#include "lcmtypes/multisense.hpp"


#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pointcloud_tools/pointcloud_lcm.hpp> // create point clouds
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

using namespace cv;
using namespace std;

class image_tool{
  public:
    image_tool(boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_in_, 
                 std::string camera_out_, 
                 bool output_pointcloud_, bool output_images_, int decimate_);
    
    ~image_tool(){
    }
    
  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    std::string camera_in_, camera_out_;
    std::string mask_channel_;
    bool output_pointcloud_, output_images_;
    
    void disparityHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg);   
    void maskHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg);   
    
    
    int counter_;

    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;
    
    mutable std::vector<float> disparity_buff_;
    mutable std::vector<cv::Vec3f> points_buff_;
    mutable sensor_msgs::PointCloud2 cloud_out;
    
    cv::Mat_<double> Q_;
    int decimate_;
    
    pointcloud_vis* pc_vis_;  
    pointcloud_lcm* pc_lcm_;     
    image_io_utils*  imgutils_;

    
    bot_core::image_t last_mask_;    
};    


// static bool isValidPoint(const cv::Vec3f& pt)
// {
//   // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
//   // and zero disparities (point mapped to infinity).
//   return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
//   return pt[2];// != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
// }


image_tool::image_tool(boost::shared_ptr<lcm::LCM> &lcm_, std::string camera_in_,
                           std::string camera_out_, 
                           bool output_pointcloud_, bool output_images_, int decimate_):
      lcm_(lcm_), camera_in_(camera_in_), camera_out_(camera_out_), 
      output_pointcloud_(output_pointcloud_), output_images_(output_images_), decimate_(decimate_), Q_(4,4,0.0){
        
  lcm_->subscribe( camera_in_.c_str(),&image_tool::disparityHandler,this);
  mask_channel_="CAMERALEFT_MASKZIPPED";
  lcm_->subscribe( mask_channel_ ,&image_tool::maskHandler,this);

  
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);  
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);  
        
  pc_vis_ = new pointcloud_vis(lcm_->getUnderlyingLCM());
  float colors_r[] ={1.0,0.0,0.0};
  vector <float> colors_v_r;
  colors_v_r.assign(colors_r,colors_r+4*sizeof(float));
  int reset =1;
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb        
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3001,"Pts",1,1, 3000,0,colors_v_r));
        
  pc_lcm_ = new pointcloud_lcm(lcm_->getUnderlyingLCM());
  pc_lcm_->set_decimate( decimate_ );  
        
  
  // TODO: get from botparam server..
  if (1==0){ // real device with letterboxes
    Q_(0,0) = Q_(1,1) = 1.0;  
    //double Tx = baseline();
    Q_(3,2) = 14.2914745276283;//1.0 / Tx;
    Q_(0,3) = -512;//-right_.cx();
    Q_(1,3) = -544;//-right_.cy();
    Q_(2,3) = 606.0344848632812;//right_.fx();
    Q_(3,3) = 0;//(right_.cx() - left_.cx()) / Tx;  
  }else if (1==0){   // real device without letterboxes
    Q_(0,0) = Q_(1,1) = 1.0;  
    //double Tx = baseline();
    Q_(3,2) = 14.2914745276283;//1.0 / Tx;
    Q_(0,3) = -512.0;//-right_.cx();
    Q_(1,3) = -272.0;//-right_.cy();
    Q_(2,3) = 606.0344848632812;//right_.fx();
    Q_(3,3) = 0;// (512.0 - 272.0)/0.07;//(right_.cx() - left_.cx()) / Tx;  
  }else{ // simulated
    Q_(0,0) = Q_(1,1) = 1.0;  
    //double Tx = baseline();
    Q_(3,2) = 14.2914745276283;//1.0 / Tx;
    Q_(0,3) = -512.5;//-right_.cx();
    Q_(1,3) = -272.5;//-right_.cy();
    Q_(2,3) = 610.1778;//right_.fx();
    Q_(3,3) = 0;// (512.0 - 272.0)/0.07;//(right_.cx() - left_.cx()) / Tx;      
  }
  std::cout << Q_ << " is reprojectionMatrix\n";  
  
  imgutils_ = new image_io_utils( lcm_->getUnderlyingLCM(), 1024, 544);
  last_mask_.utime =0; // use this number to determine initial image
  counter_=0;
  
}



// left , disparity
void image_tool::disparityHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  multisense::images_t* msg){
  int w = msg->images[0].width;
  int h = msg->images[0].height; 

  // 1. Output Images
  if (output_images_){
    // cout << (int) msg->image_types[0] << " and " << (int) msg->image_types[1] << "\n";
    lcm_->publish(camera_out_.c_str(), &msg->images[0]);
    lcm_->publish( "SECOND_IMAGE" , &msg->images[1]); // TODO add paramater for name
  }
  
  // Only process the point cloud occasionally:
  counter_++;
  if (counter_ % 5 !=0){ 
    return;
  }
  cout << counter_ << " @ "<< msg->utime << " | "<< msg->images[0].width <<" x "<< msg->images[0].height <<"\n";
  
  // Extract a point cloud:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pc_lcm_->unpack_multisense(msg,Q_,cloud);  
  if (!output_pointcloud_)
    return;
  
  /// 2. Colorize point cloud using the image mask
  // TODO: add proper time checks ... or change change incoming messages
  if(last_mask_.utime!=0){
    // cout <<"got mask and depth\n";
    uint8_t* mask_buf = imgutils_->unzipImage(   &last_mask_ );
    //imgutils_->sendImage(mask_buf, msg->utime, 1024, 544, 1, string("UNZIPPED")  );

    // Colorise the depth points using the mask
    int j2=0;
    for(int v=0; v<h; v=v+ decimate_) { // t2b
      for(int u=0; u<w; u=u+decimate_ ) {  //l2r
          if (mask_buf[v*w + u] > 0){ // if the mask is not black, apply it as red
            cloud->points[j2].r = mask_buf[v*w + u];
            cloud->points[j2].g = cloud->points[j2].g/4;
            cloud->points[j2].b = cloud->points[j2].b/4; // reduce other color for emphaise
          }
          j2++;
      }
    } 
      
    if (1==1){ // sanity checks
      cv::Mat mask(h, w, CV_8UC1);
      mask.data = mask_buf;
      cv::imwrite("mask_image.png", mask);

      cv::Mat img(h, w, CV_8UC1);
      std::copy(msg->images[0].data.data(), msg->images[0].data.data() + (msg->images[0].size) , img.data);
      cv::imwrite("gray_image.png", img);

      cv::Mat mask_combined;
      mask_combined = 0.3*img + 0.7*mask;
      cv::imwrite("mask_image_applied_to_gray.png", mask_combined);
      
      // cout << "size ptcd: " << cloud->points.size() << " " << cloud->width << " " << cloud->height << "\n";
    }
  }
  
  /// 3 Publish the point cloud on the camera pose:
  Eigen::Isometry3d ref_pose;
  if (1==0){ 
    // Apply in initial local-to-camera pose transform
    ref_pose.setIdentity();
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd ( 90*M_PI/180 , Eigen::Vector3d::UnitZ ()) // was 0
      * Eigen::AngleAxisd ( 180*M_PI/180 , Eigen::Vector3d::UnitY ())
      * Eigen::AngleAxisd ( 90*M_PI/180  , Eigen::Vector3d::UnitX ());
    ref_pose *= m;  
  }else{
    botframes_cpp_->get_trans_with_utime( botframes_ ,  "CAMERA", "local", msg->utime, ref_pose);  
  }
  Isometry3dTime ref_poseT = Isometry3dTime(msg->utime, ref_pose);
  pc_vis_->pose_to_lcm_from_list(3000, ref_poseT);    
  pc_vis_->ptcld_to_lcm_from_list(3001, *cloud, msg->utime, msg->utime);
  
  //pcl::PCDWriter writer;
  //std::stringstream pcd_fname;
  //pcd_fname << "/home/mfallon/Desktop/depth/pcd_lcm_" << counter_ << ".pcd";
  //writer.write (pcd_fname.str() , *cloud, false);  
}


void image_tool::maskHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::image_t* msg){
  last_mask_= *msg;  
  //cout << "got mask\n";  
}

int main(int argc, char ** argv) {
  ConciseArgs parser(argc, argv, "registeration-app");
  string camera_in="MULTISENSE_LD";
  string camera_out="CAMERALEFT";
  bool output_pointcloud=false;
  bool output_images=false;  
  int decimate = 8;
  parser.add(camera_in, "i", "in", "Incoming Multisense channel");
  parser.add(camera_out, "o", "out", "Outgoing Mono Camera channel");
  parser.add(output_pointcloud, "p", "output_pointcloud", "Output PointCloud");
  parser.add(output_images, "s", "output_images", "Output the images split");
  parser.add(decimate, "d", "decimate", "Decimation of images");
  parser.parse();
  cout << camera_in << " is camera_in\n"; 
  cout << camera_out << " is camera_out\n"; 
  cout << decimate << " is decimate\n"; 
  cout << output_pointcloud << " is output_pointcloud\n"; 
  cout << output_images << " is output_images (split\n"; 

  
  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }
  
  image_tool app(lcm,camera_in, camera_out, output_pointcloud, output_images, decimate);
  cout << "Ready image tool" << endl << "============================" << endl;
  while(0 == lcm->handle());
  return 0;
}
