// mfallon sept 2012

#include <stdio.h>
#include <bits/algorithmfwd.h>
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
    image_writer(lcm_t* publish_lcm, std::string object_, std::string pose_out_);
    
    ~image_writer(){
    }
    
  private:
    lcm_t* publish_lcm_;
    std::string object_;
    std::string pose_out_;
    static void images_handler_aux(const lcm_recv_buf_t* rbuf, const char* channel,
                                const bot_core_rigid_transform_t* msg, void* user_data){
      ((image_writer *) user_data)->images_handler(msg, channel);
    }
    void images_handler(const bot_core_rigid_transform_t *msg, const char* channel);
    
    int counter_;
    
  pointcloud_vis* pc_vis_;    
};    


// static bool isValidPoint(const cv::Vec3f& pt)
// {
//   // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
//   // and zero disparities (point mapped to infinity).
//   return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
//   return pt[2];// != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
// }


image_writer::image_writer(lcm_t* publish_lcm_, std::string object_, std::string pose_out_):
      publish_lcm_(publish_lcm_), object_(object_), pose_out_(pose_out_){
        
  pc_vis_ = new pointcloud_vis(publish_lcm_);
  int reset =1;
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb        
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Pose",5,1) );
        
        
  counter_=0;
  bot_core_rigid_transform_t_subscribe(publish_lcm_, object_.c_str(),
      image_writer::images_handler_aux, this);
  
}


void image_writer::images_handler(const bot_core_rigid_transform_t *msg, const char* channel){
/*
  bot_core_pose_t p;
  p.utime =msg->utime;
  p.pos[0] = msg->trans[0];
  p.pos[1] = msg->trans[1];
  p.pos[2] = msg->trans[2];
  p.orientation[0] = msg->quat[0];
  p.orientation[1] = msg->quat[1];
  p.orientation[2] = msg->quat[2];
  p.orientation[3] = msg->quat[3];
*/  
  
  Eigen::Isometry3d init_pose;
// Apply in initial local-to-object pose transform
  init_pose.setIdentity();
  init_pose.translation()  << msg->trans[0], msg->trans[1], msg->trans[2];
  Eigen::Quaterniond m1 = Eigen::Quaterniond( msg->quat[0],  msg->quat[1],  msg->quat[2],  msg->quat[3] );
  init_pose.rotate(m1);
  
  Eigen::Quaterniond fix_r = euler_to_quat(-90.0*M_PI/180.0, 0.0*M_PI/180.0 , 0.0*M_PI/180.0);
  init_pose.rotate(fix_r);  

  Eigen::Quaterniond align_r = euler_to_quat(0.0*M_PI/180.0, 0.0*M_PI/180.0 , 0.0*M_PI/180.0);
  init_pose.rotate(align_r);  
  
  
  Isometry3dTime ref_poseT = Isometry3dTime(msg->utime, init_pose);
  pc_vis_->pose_to_lcm_from_list(3000, ref_poseT);    
  
  
  bot_core_pose_t p;
  p.utime =msg->utime;
  p.pos[0] = init_pose.translation().x();
  p.pos[1] = init_pose.translation().y();
  p.pos[2] = init_pose.translation().z();
  
  Eigen::Quaterniond r_out (  init_pose.rotation() );
  
  p.orientation[0] = r_out.w();
  p.orientation[1] = r_out.x();
  p.orientation[2] = r_out.y();
  p.orientation[3] = r_out.z();
  
  
  
  bot_core_pose_t_publish(publish_lcm_, pose_out_.c_str(), &p);
  
  
  //pcl::PCDWriter writer;
  //std::stringstream pcd_fname;
  //pcd_fname << "/home/mfallon/Desktop/depth/pcd_lcm_" << counter_ << ".pcd";
  //writer.write (pcd_fname.str() , *cloud, false);  
  
  counter_++;
  if (counter_>100){
    cout << msg->utime <<"\n";
    counter_=0; 
  }
}


int main(int argc, char ** argv) {
  ConciseArgs parser(argc, argv, "registeration-app");
  string object="VICON_multisense";
  string pose_out = "POSE_HEAD_VICON";
  parser.add(object, "c", "object", "Incoming Camera channel");
  parser.add(pose_out, "p", "pose_out", "Pose Out Chan");
  parser.parse();
  cout << object << " is object\n"; 
  cout << pose_out << " is pose_out\n"; 

  
  lcm_t * lcm;
  lcm = lcm_create(NULL);//"udpm://239.255.76.67:7667?recv_buf_size=100000");

  image_writer app(lcm,object,pose_out);

  while(1)
    lcm_handle(lcm);

  lcm_destroy(lcm);
  return 0;
}
