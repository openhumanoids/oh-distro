// mfallon sept 2012

#include <stdio.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ConciseArgs>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


#include <pointcloud_tools/pointcloud_vis.hpp> // visualize pt clds

using namespace cv;
using namespace std;

class image_writer{
  public:
    image_writer(lcm_t* publish_lcm, std::string camera_, 
                 bool output_color_, int decimate_);
    
    ~image_writer(){
    }
    
  private:
    lcm_t* publish_lcm_;
    std::string camera_;
    bool output_color_;
    static void image_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel,
                                const bot_core_image_t* msg, void* user_data){
      ((image_writer *) user_data)->image_handler(msg, channel);
    }
    void image_handler(const bot_core_image_t *msg, const char* channel);
    
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
                           bool output_color_, int decimate_):
      publish_lcm_(publish_lcm_), camera_(camera_), 
      output_color_(output_color_), decimate_(decimate_), Q_(4,4,0.0){
        
        
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  float colors_r[] ={1.0,0.0,0.0};
  vector <float> colors_v_r;
  colors_v_r.assign(colors_r,colors_r+4*sizeof(float));
  int reset =1;
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb        
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Pose",5,1) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3001,"Pts",1,1, 3000,1,colors_v_r));
        
        
  counter_;
  bot_core_image_t_subscribe(publish_lcm_, camera_.c_str(),
      image_writer::image_handler_aux, this);
  

  Q_(0,0) = Q_(1,1) = 1.0;  
  //double Tx = baseline();
  Q_(3,2) = 14.2914745276283;//1.0 / Tx;
  Q_(0,3) = -512;//-right_.cx();
  Q_(1,3) = -544;//-right_.cy();
  Q_(2,3) = 606.0344848632812;//right_.fx();
  Q_(3,3) = 0;//(right_.cx() - left_.cx()) / Tx;  
  std::cout << Q_ << " is reprojectionMatrix\n";  
  
}


void image_writer::image_handler(const bot_core_image_t *msg, const char* channel){
  cout << msg->utime << "\n";
  
  int h = msg->height;
  int w = msg->width;

  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  Mat disparity_orig_temp = Mat::zeros(msg->height,msg->width,CV_16UC1); // h,w
  disparity_orig_temp.data = msg->data;  
  cv::Mat_<uint16_t> disparity_orig(h, w);
  disparity_orig = disparity_orig_temp;
  
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
        j2++;
    }
  }

  Eigen::Isometry3d init_pose;
// Apply in initial local-to-camera pose transform
  init_pose.setIdentity();
  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd ( 0*M_PI/180 , Eigen::Vector3d::UnitZ ())
    * Eigen::AngleAxisd ( 180*M_PI/180 , Eigen::Vector3d::UnitY ())
    * Eigen::AngleAxisd ( 90*M_PI/180  , Eigen::Vector3d::UnitX ());
  init_pose *= m;  
  Isometry3dTime ref_poseT = Isometry3dTime(msg->utime, init_pose);
  pc_vis_->pose_to_lcm_from_list(3000, ref_poseT);    
  pc_vis_->ptcld_to_lcm_from_list(3001, *cloud, msg->utime, msg->utime);
  
  
  //std::stringstream disparity_fname;
  //disparity_fname << "/home/mfallon/Desktop/depth/disparity_lcm_" << counter_ << ".png";
  //imwrite(disparity_fname.str(),img_lcm); 
  
  //pcl::PCDWriter writer;
  //std::stringstream pcd_fname;
  //pcd_fname << "/home/mfallon/Desktop/depth/pcd_lcm_" << counter_ << ".pcd";
  //writer.write (pcd_fname.str() , *cloud, false);  
  
  cout << "no. points: "<< cloud->points.size() <<"\n";
  counter_++;
}


int main(int argc, char ** argv) {
  ConciseArgs parser(argc, argv, "registeration-app");
  string camera="CAMLCM_IMAGE_GRAY";
  bool output_color=false;
  int decimate = 8;
  parser.add(camera, "c", "camera", "Incoming Camera channel");
  parser.add(output_color, "o", "output_color", "Output Color [t=color, f=grey]");
  parser.add(decimate, "d", "decimate", "Decimation of images");
  parser.parse();
  cout << camera << " is camera\n"; 
  cout << decimate << " is decimate\n"; 
  cout << output_color << " is output_color\n"; 

  
  lcm_t * lcm;
  lcm = lcm_create(NULL);//"udpm://239.255.76.67:7667?recv_buf_size=100000");

  image_writer app(lcm,camera, output_color, decimate);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
