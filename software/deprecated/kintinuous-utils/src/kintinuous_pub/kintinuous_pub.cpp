#include "kintinuous_pub.hpp"

using namespace std;


kintinuous_pub::kintinuous_pub (){
  boost::shared_ptr<lcm::LCM> theLCM(new lcm::LCM);
  lcm_ = theLCM;

  width_ = 1024;
  height_ = 544;
  npixels_ = width_*height_;
  ncolors_ =3;
  ndepthbytes_ = 2;

  lcm_left_.width = width_;
  lcm_left_.height = height_;
  lcm_left_.row_stride = width_*ncolors_;
  lcm_left_.size= width_*height_ * ncolors_;
  lcm_left_.data.resize( lcm_left_.size);
  lcm_left_.nmetadata =0;
  lcm_left_.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;

  lcm_depth_.width = width_;
  lcm_depth_.height = height_;
  lcm_depth_.row_stride = width_*ndepthbytes_;
  lcm_depth_.size= width_*height_ * ndepthbytes_;
  lcm_depth_.data.resize( lcm_depth_.size);
  lcm_depth_.nmetadata =0;
  lcm_depth_.pixelformat = bot_core::image_t::PIXEL_FORMAT_BE_GRAY16; // i'm not sure this is the right type

  lcm_fused_.n_images =2;
  lcm_fused_.image_types.push_back(0);//multisense::images_t::LEFT );
  lcm_fused_.image_types.push_back(4);//multisense::images_t::DEPTH_MM );
  lcm_fused_.images.push_back(lcm_left_);
  lcm_fused_.images.push_back(lcm_depth_);
}

// colorData is rgb in typical LCM ordering
void kintinuous_pub::sendImages(uint8_t * colorData, uint16_t * depthData){
  std::cout << "Publishing fused range images\n";

  // 1. Publish to LCM:
  lcm_left_.utime = current_utime;
  lcm_depth_.utime = current_utime;
  lcm_fused_.utime = current_utime;   
  memcpy(&lcm_left_.data[0], colorData, npixels_* ncolors_ );
  memcpy(&lcm_depth_.data[0], depthData, npixels_* ndepthbytes_ );
  // lcm_->publish("CAMERA_FUSED_COLOR", &lcm_left_);
  // lcm_->publish("CAMERA_FUSED_DEPTH", &lcm_depth_);

  lcm_fused_.images[0]= lcm_left_; // is this necessary?
  lcm_fused_.images[1]= lcm_depth_;
  lcm_->publish("CAMERA_FUSED", &lcm_fused_);

  if (1==0){
    writeToFile(depthData);
  }
}

// Output Raw data formats
void kintinuous_pub::writeToFile(uint16_t * depthData){
  // Save mm depth values to text file:
  ofstream myfile;
  myfile.open ("/tmp/kintinuous_fused_depth.txt");
  std::cout << "Writing depth values to /tmp/kintinuous_fused_depth.txt\n";
  for(int i = 0; i < width_; i++){
    for(int j = 0; j < height_; j++){
      myfile << depthData[ j*width_ + i ] << ", ";
    }
    myfile << "\n";
  }
  myfile.close();  

  // Scale and save as grayscale PNG or push to LCM
  float max = 0;
  for(int i = 0; i < npixels_; i++){
    if( depthData[i] > max ){
      max = depthData[i];
    }
  }
  std::cout << "Max depth: " << max << "mm.\n"; // typically 

  IplImage * depthImg;
  depthImg = cvCreateImage( cvSize( width_, height_ ) , IPL_DEPTH_8U, 1);
  for(int i = 0; i < npixels_; i++){
    depthImg->imageData[i] = ((float)depthData[i] / max) * 255.0f;
  }
  cvSaveImage("/tmp/kintinuous_fused_depths.png",depthImg);

  if (1==0){
    bot_core_image_t msg;
    msg.utime =123;
    msg.width = width_;
    msg.height = height_;
    msg.row_stride = width_;
    msg.size= npixels_;
    msg.data= (uint8_t*) depthImg->imageData;
    msg.nmetadata =0;
    msg.metadata =NULL;
    msg.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
    bot_core_image_t_publish( lcm_->getUnderlyingLCM(), "CAMERA_FUSED_DEPTH_GRAY", &msg);
  }
}

void kintinuous_pub::sendPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
    std::cout << "Just fetched: " << cloud->points.size() << " from GPU\n";

    /// Raw Cloud: ~5MB
    /// 1cm voxel grid: 376K
    pcl::PCDWriter writer;
    if (1==0){    
      // Write the cloud to disk [mfallon, temp]
      writer.write<pcl::PointXYZRGB> ("/tmp/kintinuous_cloud.pcd", *cloud, true);  
    }
    
    // Sparsify using VoxelGrid:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vox (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud ( cloud );
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_vox);    
    std::cout << "voxel grid: " << cloud_vox->points.size() << " points\n";

    writer.write<pcl::PointXYZRGB> ("/tmp/kintinuous_cloud_voxel.pcd", *cloud_vox, true);  
}
