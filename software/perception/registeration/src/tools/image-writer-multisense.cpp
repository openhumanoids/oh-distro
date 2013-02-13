// mfallon sept 2012

#include <stdio.h>
#include <lcm/lcm.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ConciseArgs>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <bot_core/bot_core.h>
#include <lcmtypes/multisense.h>

#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

using namespace cv;
using namespace std;

class image_writer{
  public:
    image_writer(lcm_t* publish_lcm, std::string camera_, 
                 std::string camera_out_, 
                 bool output_pointcloud_, int decimate_);
    
    ~image_writer(){
    }
    
  private:
    lcm_t* publish_lcm_;
    std::string camera_, camera_out_;
    bool output_pointcloud_;
    static void images_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel,
                                const multisense_images_t* msg, void* user_data){
      ((image_writer *) user_data)->images_handler(msg, channel);
    }
    void images_handler(const multisense_images_t *msg, const char* channel);
    
    int counter_;

    
    mutable std::vector<float> disparity_buff_;
    mutable std::vector<cv::Vec3f> points_buff_;
    mutable sensor_msgs::PointCloud2 cloud_out;
    
    cv::Mat_<double> Q_;
    int decimate_;
    
  pointcloud_vis* pc_vis_;    
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
                           bool output_pointcloud_, int decimate_):
      publish_lcm_(publish_lcm_), camera_(camera_), camera_out_(camera_out_), 
      output_pointcloud_(output_pointcloud_), decimate_(decimate_), Q_(4,4,0.0){
        
        
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  float colors_r[] ={1.0,0.0,0.0};
  vector <float> colors_v_r;
  colors_v_r.assign(colors_r,colors_r+4*sizeof(float));
  int reset =1;
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb        
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3001,"Pts",1,1, 3000,0,colors_v_r));
        
        
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
  }else{  
  Q_(0,0) = Q_(1,1) = 1.0;  
  //double Tx = baseline();
  Q_(3,2) = 14.2914745276283;//1.0 / Tx;
  Q_(0,3) = -512.0;//-right_.cx();
  Q_(1,3) = -272.0;//-right_.cy();
  Q_(2,3) = 606.0344848632812;//right_.fx();
  Q_(3,3) = 0;// (512.0 - 272.0)/0.07;//(right_.cx() - left_.cx()) / Tx;  
  }
  std::cout << Q_ << " is reprojectionMatrix\n";  
  
  
}


void image_writer::images_handler(const multisense_images_t *msg, const char* channel){
  cout << msg->utime << " | "<< msg->images[0].width <<" | "<< msg->images[0].height <<"\n";
  cout << (int) msg->image_types[0] << " and " << (int) msg->image_types[1] << "\n";
  bot_core_image_t_publish(publish_lcm_, camera_out_.c_str() , &msg->images[0]);    

  if (!output_pointcloud_)
    return;
  // left , disparity
  int h = msg->images[0].height;
  int w = msg->images[0].width;
  
  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
  disparity_orig_temp.data = msg->images[1].data;  
  cv::Mat_<uint16_t> disparity_orig(h, w);
  disparity_orig = disparity_orig_temp;
  
  //std::stringstream disparity_fname;
  //disparity_fname << "/home/mfallon/Desktop/depth/disparity_lcm_" << msg->utime << ".png";
  //imwrite(disparity_fname.str(),disparity_orig_temp); 
  
  
  disparity_buff_.resize(h * w);
  cv::Mat_<float> disparity(h, w, &(disparity_buff_[0]));
  disparity = disparity_orig / 16.0;
  
  // Allocate buffer for reprojection output
  points_buff_.resize(h * w);
  cv::Mat_<cv::Vec3f> points(h, w, &(points_buff_[0]));

  // Do the reprojection in open space
  static const bool handle_missing_values = true;
  cv::reprojectImageTo3D(disparity, points, Q_, handle_missing_values);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->width    =(int) (w/ (double) decimate_) ;
  cloud->height   =(int) (h/ (double) decimate_);
  cloud->is_dense = true;
  cloud->points.resize (cloud->width * cloud->height);  
  int j2=0;
  for(int v=0; v<h; v=v+ decimate_) { // t2b
    for(int u=0; u<w; u=u+decimate_ ) {  //l2r
        //cout <<  points(v,u)[0] << " " <<  points(v,u)[1] << " " <<  points(v,u)[1] << "\n";
        cloud->points[j2].x = points(v,u)[0];
        cloud->points[j2].y = points(v,u)[1];
        cloud->points[j2].z = points(v,u)[2];
        
        cloud->points[j2].r = msg->images[0].data[v*w + u];
        cloud->points[j2].g = msg->images[0].data[v*w + u];// + 1];
        cloud->points[j2].b = msg->images[0].data[v*w + u];// + 2];
        j2++;
    }
  }

  Eigen::Isometry3d init_pose;
// Apply in initial local-to-camera pose transform
  init_pose.setIdentity();
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd ( 90*M_PI/180 , Eigen::Vector3d::UnitZ ()) // was 0
    * Eigen::AngleAxisd ( 180*M_PI/180 , Eigen::Vector3d::UnitY ())
    * Eigen::AngleAxisd ( 90*M_PI/180  , Eigen::Vector3d::UnitX ());
  init_pose *= m;  
  Isometry3dTime ref_poseT = Isometry3dTime(msg->utime, init_pose);
  pc_vis_->pose_to_lcm_from_list(3000, ref_poseT);    
  pc_vis_->ptcld_to_lcm_from_list(3001, *cloud, msg->utime, msg->utime);
  
  
  
  //pcl::PCDWriter writer;
  //std::stringstream pcd_fname;
  //pcd_fname << "/home/mfallon/Desktop/depth/pcd_lcm_" << counter_ << ".pcd";
  //writer.write (pcd_fname.str() , *cloud, false);  
  
  cout << "no. points: "<< cloud->points.size() <<"\n";
  counter_++;
}


int main(int argc, char ** argv) {
  ConciseArgs parser(argc, argv, "registeration-app");
  string camera_in="MULTISENSE_LD";
  string camera_out="CAMERALEFT";
  bool output_pointcloud=false;
  int decimate = 8;
  parser.add(camera_in, "i", "in", "Incoming Multisense channel");
  parser.add(camera_out, "o", "out", "Outgoing Mono Camera channel");
  parser.add(output_pointcloud, "p", "output_pointcloud", "Output PointCloud");
  parser.add(decimate, "d", "decimate", "Decimation of images");
  parser.parse();
  cout << camera_in << " is camera_in\n"; 
  cout << camera_out << " is camera_out\n"; 
  cout << decimate << " is decimate\n"; 
  cout << output_pointcloud << " is output_pointcloud\n"; 

  
  lcm_t * lcm;
  lcm = lcm_create(NULL);//"udpm://239.255.76.67:7667?recv_buf_size=100000");

  image_writer app(lcm,camera_in, camera_out, output_pointcloud, decimate);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
