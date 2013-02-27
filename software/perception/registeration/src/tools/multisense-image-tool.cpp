// Tool to take a multisense stereo image (images_t)
// - republish images on seperate channels
// - use disparity image and left and create a point cloud
//
// Various features incomplete as of feb 2013
// e.g. support for rgb, masking, checking if images are actually left or disparity
//      getting the camera calibration from the param server
// mfallon 2013

#include <stdio.h>
#include <lcm/lcm.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ConciseArgs>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <bot_core/bot_core.h>
#include <lcmtypes/multisense.h>


#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <pointcloud_tools/pointcloud_lcm.hpp> // create point clouds
#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

using namespace cv;
using namespace std;

class image_writer{
  public:
    image_writer(lcm_t* publish_lcm, std::string camera_, 
                 std::string camera_out_, 
                 bool output_pointcloud_, bool output_images_, int decimate_);
    
    ~image_writer(){
    }
    
  private:
    lcm_t* publish_lcm_;
    std::string camera_, camera_out_;
    bool output_pointcloud_, output_images_;
    
    static void images_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel,
                                const multisense_images_t* msg, void* user_data){
      ((image_writer *) user_data)->images_handler(msg, channel);
    }
    void images_handler(const multisense_images_t *msg, const char* channel);
    
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
};    


// static bool isValidPoint(const cv::Vec3f& pt)
// {
//   // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
//   // and zero disparities (point mapped to infinity).
//   return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
//   return pt[2];// != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
// }


image_writer::image_writer(lcm_t* publish_lcm_, std::string camera_,
                           std::string camera_out_, 
                           bool output_pointcloud_, bool output_images_, int decimate_):
      publish_lcm_(publish_lcm_), camera_(camera_), camera_out_(camera_out_), 
      output_pointcloud_(output_pointcloud_), output_images_(output_images_), decimate_(decimate_), Q_(4,4,0.0){
        
        
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  float colors_r[] ={1.0,0.0,0.0};
  vector <float> colors_v_r;
  colors_v_r.assign(colors_r,colors_r+4*sizeof(float));
  int reset =1;
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb        
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3001,"Pts",1,1, 3000,0,colors_v_r));
        
  pc_lcm_ = new pointcloud_lcm(publish_lcm_);
  pc_lcm_->set_decimate( decimate_ );  
        
  counter_=0;
  multisense_images_t_subscribe(publish_lcm_, camera_.c_str(),
      image_writer::images_handler_aux, this);
  
  if (1==0){

    Q_(0,0) = Q_(1,1) = 1.0;  
    //double Tx = baseline();
    Q_(3,2) = 14.2914745276283;//1.0 / Tx;
    Q_(0,3) = -512;//-right_.cx();
    Q_(1,3) = -544;//-right_.cy();
    Q_(2,3) = 606.0344848632812;//right_.fx();
    Q_(3,3) = 0;//(right_.cx() - left_.cx()) / Tx;  
  }else if (1==0){   // real letterboxed
    Q_(0,0) = Q_(1,1) = 1.0;  
    //double Tx = baseline();
    Q_(3,2) = 14.2914745276283;//1.0 / Tx;
    Q_(0,3) = -512.0;//-right_.cx();
    Q_(1,3) = -272.0;//-right_.cy();
    Q_(2,3) = 606.0344848632812;//right_.fx();
    Q_(3,3) = 0;// (512.0 - 272.0)/0.07;//(right_.cx() - left_.cx()) / Tx;  
  }else{
    Q_(0,0) = Q_(1,1) = 1.0;  
    //double Tx = baseline();
    Q_(3,2) = 14.2914745276283;//1.0 / Tx;
    Q_(0,3) = -512.5;//-right_.cx();
    Q_(1,3) = -272.5;//-right_.cy();
    Q_(2,3) = 610.1778;//right_.fx();
    Q_(3,3) = 0;// (512.0 - 272.0)/0.07;//(right_.cx() - left_.cx()) / Tx;      
  }
  std::cout << Q_ << " is reprojectionMatrix\n";  
  
  botparam_ = bot_param_new_from_server(publish_lcm_, 0);  
  botframes_= bot_frames_get_global(publish_lcm_, botparam_);  
}


// left , disparity
void image_writer::images_handler(const multisense_images_t *msg, const char* channel){
  cout << msg->utime << " | "<< msg->images[0].width <<" | "<< msg->images[0].height <<"\n";
  // cout << (int) msg->image_types[0] << " and " << (int) msg->image_types[1] << "\n";
  
  if (output_images_){
    bot_core_image_t_publish(publish_lcm_, camera_out_.c_str() , &msg->images[0]);    
    bot_core_image_t_publish(publish_lcm_, "SECOND_IMAGE" , &msg->images[1]);     // TODO fix me
  }

  if (!output_pointcloud_)
    return;
    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pc_lcm_->unpack_multisense(msg,Q_,cloud);  
  
  
  Eigen::Isometry3d init_pose;
  if (1==0){ 
    // Apply in initial local-to-camera pose transform
    init_pose.setIdentity();
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd ( 90*M_PI/180 , Eigen::Vector3d::UnitZ ()) // was 0
      * Eigen::AngleAxisd ( 180*M_PI/180 , Eigen::Vector3d::UnitY ())
      * Eigen::AngleAxisd ( 90*M_PI/180  , Eigen::Vector3d::UnitX ());
    init_pose *= m;  
  }else{
    botframes_cpp_->get_trans_with_utime( botframes_ ,  "CAMERA", "local", msg->utime, init_pose);  
  }
  Isometry3dTime ref_poseT = Isometry3dTime(msg->utime, init_pose);
  
  pc_vis_->pose_to_lcm_from_list(3000, ref_poseT);    
  pc_vis_->ptcld_to_lcm_from_list(3001, *cloud, msg->utime, msg->utime);
  
  
  
  //pcl::PCDWriter writer;
  //std::stringstream pcd_fname;
  //pcd_fname << "/home/mfallon/Desktop/depth/pcd_lcm_" << counter_ << ".pcd";
  //writer.write (pcd_fname.str() , *cloud, false);  
  counter_++;
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

  
  lcm_t * lcm;
  lcm = lcm_create(NULL);//"udpm://239.255.76.67:7667?recv_buf_size=100000");

  image_writer app(lcm,camera_in, camera_out, output_pointcloud, output_images, decimate);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
