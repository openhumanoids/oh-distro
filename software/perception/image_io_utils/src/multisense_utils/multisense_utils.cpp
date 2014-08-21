#include <iostream>
#include <zlib.h>

#include <bot_core/bot_core.h>

#include "multisense_utils.hpp"
#define PCL_VERBOSITY_LEVEL L_ERROR

using namespace std;
using namespace cv;

multisense_utils::multisense_utils (){

  // Data buffers
  //rgb_buf_ = (uint8_t*) malloc(10*1024 * 1024 * sizeof(uint8_t)); // wasn't large enough for 1024x1024 decompression
  rgb_buf_ = (uint8_t*) malloc(10*1024 * 1024 * sizeof(uint8_t)); 
  depth_buf_ = (uint8_t*) malloc( 4*1024*1024*sizeof(uint8_t));  // arbitary size chosen..

  decimate_ =32.0;
}

void multisense_utils::unpack_multisense(const uint8_t* depth_data, const uint8_t* color_data, int h, int w, cv::Mat_<double> repro_matrix,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool is_rgb, bool is_disparity){

  if (is_disparity){
    // Convert Carnegie disparity format into floating point disparity. Store in local buffer
    Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
    //  const uint8_t* raw_data= stereob_->getDisparity();//= 3;//msg->images[1].data.data();
    disparity_orig_temp.data = (uchar*) depth_data;   // ... is a simple assignment possible?

    //std::copy(msg->images[1].data.data()             , msg->images[1].data.data() + (msg->images[1].size) ,
    //          disparity_orig_temp.data);

    // disparity_orig_temp.data = msg->images[1].data.data();   // ... is a simple assignment possible?
    cv::Mat_<float> disparity_orig(h, w);
    disparity_orig = disparity_orig_temp;

    disparity_buf_.resize(h * w);
    cv::Mat_<float> disparity(h, w, &(disparity_buf_[0]));
    disparity = disparity_orig / 16.0;

    // Allocate buffer for reprojection output
    points_buf_.resize(h * w);
    cv::Mat_<cv::Vec3f> points(h, w, &(points_buf_[0]));

    // Do the reprojection in open space
    static const bool handle_missing_values = true;
    cv::reprojectImageTo3D(disparity, points, repro_matrix, handle_missing_values);

    cloud->width    =(int) (w/ (double) decimate_) ;
    cloud->height   =(int) (h/ (double) decimate_);
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);
    int j2=0;
    for(int v=0; v<h; v=v+ decimate_) { // t2b
      for(int u=0; u<w; u=u+decimate_ ) {  //l2r
          // cout <<j2 << " " << v << " " << u << " | " <<  points(v,u)[0] << " " <<  points(v,u)[1] << " " <<  points(v,u)[1] << "\n";
          cloud->points[j2].x = points(v,u)[0];
          cloud->points[j2].y = points(v,u)[1];
          cloud->points[j2].z = points(v,u)[2];

          int pixel =v*w + u;
          if (1==0){//color_provided){ // Assumed gray:
            cloud->points[j2].r =color_data[pixel];
            cloud->points[j2].g =color_data[pixel];
            cloud->points[j2].b =color_data[pixel];
          }else{ // RGB:
            cloud->points[j2].r =color_data[pixel*3];
            cloud->points[j2].g =color_data[pixel*3 +1];
            cloud->points[j2].b =color_data[pixel*3 +2];
          }
          j2++;
      }
    }
  }else{
    uint16_t* depths = (uint16_t*) depth_data;

    // Recover focal lengths from repro_matrix - this assumes fx=fy
    // and the reprojection formula. Yuck!
    double fx = repro_matrix(2,3);
    double fy = repro_matrix(2,3);
    double cx = -repro_matrix(0,3);
    double cy = -repro_matrix(1,3);

    cloud->width    =(int) (w/ (double) decimate_) ;
    cloud->height   =(int) (h/ (double) decimate_);
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);
    int j2=0;
    for(int v=0; v<h; v=v+ decimate_) { // t2b
      for(int u=0; u<w; u=u+decimate_ ) {  //l2r

          int pixel = v*w +u;
          float z = (float) depths[pixel] /1000.0;

          cloud->points[j2].x =( z * (u  - cx))/ fx ;
          cloud->points[j2].y =( z * (v  - cy))/ fy ;
          cloud->points[j2].z =z;

          if (1==0){//color_provided){ // Assumed gray:
            cloud->points[j2].r =color_data[pixel];
            cloud->points[j2].g =color_data[pixel];
            cloud->points[j2].b =color_data[pixel];
          }else{ // RGB:
            cloud->points[j2].r =color_data[pixel*3];
            cloud->points[j2].g =color_data[pixel*3 +1];
            cloud->points[j2].b =color_data[pixel*3 +2];
          }
          j2++;
      }
    }


  }

//  cout << "points: " << cloud->points.size() << "\n";
//  pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
}


// msg - raw input data
// repro_matrix is reprojection matrix e.g. this was the loan unit in feb 2013:
//   [1, 0, 0, -512.5;
//    0, 1, 0, -272.5;
//    0, 0, 0, 606.034;
//    0, 0, 14.2914745276283, 0]
// cloud - output pcl cloud
void multisense_utils::unpack_multisense(const multisense_images_t *msg, cv::Mat_<double> repro_matrix, 
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  bool is_rgb=false;
  if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB ){
    rgb_buf_ = msg->images[0].data;
    is_rgb = true;
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY ){
    rgb_buf_ = msg->images[0].data;
    is_rgb = false;
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG ){
    jpeg_decompress_8u_rgb (msg->images[0].data, msg->images[0].size,
        rgb_buf_, msg->images[0].width, msg->images[0].height, msg->images[0].width* 3);
    //jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
    //        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
    is_rgb = true;
  }else{
    std::cout << "multisense_utils::unpack_multisense | type not understood\n";
    exit(-1);
  }
  
  // TODO: support other modes (as in the renderer)
  if (msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY_ZIPPED ) {
    unsigned long dlen = msg->images[0].width*msg->images[0].height*2 ;//msg->depth.uncompressed_size;
    uncompress(depth_buf_ , &dlen, msg->images[1].data, msg->images[1].size);
  } else{
    std::cout << "multisense_utils::unpack_multisense | depth type not understood\n";
    exit(-1);
  }  
  
  unpack_multisense(depth_buf_, rgb_buf_, msg->images[0].height, msg->images[0].width, repro_matrix, 
                                       cloud, is_rgb);
}


void multisense_utils::unpack_multisense(const multisense::images_t *msg, cv::Mat_<double> repro_matrix, 
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  bool is_rgb=true;
  if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB ){
    rgb_buf_ = (uint8_t*) msg->images[0].data.data();
    is_rgb = true;
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY ){
    rgb_buf_ = (uint8_t*) msg->images[0].data.data();
    is_rgb = false;
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG ){
    jpeg_decompress_8u_rgb ( msg->images[0].data.data(), msg->images[0].size,
        rgb_buf_, msg->images[0].width, msg->images[0].height, msg->images[0].width* 3);
    //jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
    //        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
    is_rgb = true;
  }else{
    std::cout << "multisense_utils::unpack_multisense | image type not understood\n";
    exit(-1);
  }
  
  // TODO: support non-zipped modes (as in the renderer)
  bool is_disparity=true;
  if (msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY_ZIPPED ) {
    unsigned long dlen = msg->images[0].width*msg->images[0].height*2 ;//msg->depth.uncompressed_size;
    uncompress(depth_buf_ , &dlen, msg->images[1].data.data(), msg->images[1].size);
    is_disparity=true;
  }else if (msg->image_types[1] == MULTISENSE_IMAGES_T_DEPTH_MM_ZIPPED ) {
        unsigned long dlen = msg->images[0].width*msg->images[0].height*2 ;//msg->depth.uncompressed_size;
    uncompress(depth_buf_ , &dlen, msg->images[1].data.data(), msg->images[1].size);
    is_disparity=false;
  } else{
    std::cout << "multisense_utils::unpack_multisense | depth type not understood\n";
    exit(-1);
  }
  
  unpack_multisense(depth_buf_, rgb_buf_, msg->images[0].height, msg->images[0].width, repro_matrix, 
                                       cloud, is_rgb, is_disparity);
  // unpack_multisense(msg->images[1].data.data(), msg->images[0].data.data(), msg->images[0].height, msg->images[0].width, repro_matrix, 
  //                                     cloud, is_rgb);
}
