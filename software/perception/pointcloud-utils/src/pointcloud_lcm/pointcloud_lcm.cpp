#include <iostream>
#include <zlib.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include "visualization/collections.hpp"

#include "pointcloud_lcm.hpp"
#define PCL_VERBOSITY_LEVEL L_ERROR

using namespace std;
using namespace cv;

pointcloud_lcm::pointcloud_lcm (lcm_t* publish_lcm):
        publish_lcm_(publish_lcm){

  kcal = kinect_calib_new();
  kcal->intrinsics_depth.fx = 576.09757860;
  kcal->intrinsics_depth.cx = 321.06398107;
  kcal->intrinsics_depth.cy = 242.97676897;
  kcal->intrinsics_rgb.fx = 576.09757860;
  kcal->intrinsics_rgb.cx = 321.06398107;
  kcal->intrinsics_rgb.cy = 242.97676897;
  kcal->intrinsics_rgb.k1 = 0; // none given so far
  kcal->intrinsics_rgb.k2 = 0; // none given so far
  kcal->shift_offset = 1079.4753;
  kcal->projector_depth_baseline = 0.07214;
  //double rotation[9];
  double rotation[]={0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970};
  double depth_to_rgb_translation[] ={ -0.015756, -0.000923, 0.002316};
  memcpy(kcal->depth_to_rgb_rot, rotation, 9*sizeof(double)); 
  memcpy(kcal->depth_to_rgb_translation, depth_to_rgb_translation  , 3*sizeof(double));  


  // Data buffers
  //rgb_buf_ = (uint8_t*) malloc(10*1024 * 1024 * sizeof(uint8_t)); // wasn't large enough for 1024x1024 decompression
  rgb_buf_ = (uint8_t*) malloc(10*1024 * 1024 * sizeof(uint8_t)); 
  depth_buf_ = (uint8_t*) malloc( 4*1024*1204*sizeof(uint8_t));  // arbitary size chosen..

  decimate_ =32.0;
}


// Copied from kinect-lcm
static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
    double result[3])
{
  result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
  result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
  result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}


void pointcloud_lcm::unpack_pointcloud2(const ptools_pointcloud2_t *msg,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

  // 1. Copy fields - this duplicates /pcl/ros/conversions.h for "fromROSmsg"
  cloud->width   = msg->width;
  cloud->height   = msg->height;
  uint32_t num_points = msg->width * msg->height;
  cloud->points.resize (num_points);
  cloud->is_dense = false;//msg->is_dense;
  uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud->points[0]);
  const uint8_t* msg_data = &msg->data[0];
  memcpy (cloud_data, msg_data, msg->data_nbytes );

  // 2. HACK/Workaround
  // for some reason in pcl1.5/6, this callback results
  // in RGB data whose offset is not correctly understood
  // Instead of an offset of 12bytes, its offset is set to be 16
  // this fix corrects for the issue:
  sensor_msgs::PointCloud2 msg_cld;
  pcl::toROSMsg(*cloud, msg_cld);
  msg_cld.fields[3].offset = 12;
  pcl::fromROSMsg (msg_cld, *cloud);
}


void pointcloud_lcm::unpack_pointcloud2(const ptools::pointcloud2_t *msg,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

  // 1. Copy fields - this duplicates /pcl/ros/conversions.h for "fromROSmsg"
  cloud->width   = msg->width;
  cloud->height   = msg->height;
  uint32_t num_points = msg->width * msg->height;
  cloud->points.resize (num_points);
  cloud->is_dense = false;//msg->is_dense;
  uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud->points[0]);
 const uint8_t* msg_data = &msg->data[0];
  memcpy (cloud_data, msg_data, msg->data_nbytes );

  // 2. HACK/Workaround
  // for some reason in pcl1.5/6, this callback results
  // in RGB data whose offset is not correctly understood
  // Instead of an offset of 12bytes, its offset is set to be 16
  // this fix corrects for the issue:
  sensor_msgs::PointCloud2 msg_cld;
  pcl::toROSMsg(*cloud, msg_cld);
  msg_cld.fields[3].offset = 16; // was 12 // 16 works for PCL Streaming
  pcl::fromROSMsg (msg_cld, *cloud);
}


void pointcloud_lcm::unpack_kinect_frame(const kinect_frame_msg_t *msg, uint8_t* rgb_data,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){

  /////////////////////////////////////////////////////////////////////
  /// 1.1 RGB:
  // TODO check width, height
  if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
    memcpy(rgb_data, msg->image.image_data,
        msg->depth.width * msg->depth.height * 3);
  } else if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
    jpeg_decompress_8u_rgb (msg->image.image_data, msg->image.image_data_nbytes,
        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
    //jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
    //        rgb_data, msg->image.width, msg->image.height, msg->image.width* 3);
  }

  /////////////////////////////////////////////////////////////////////
  /// 1.2. DEPTH:
  uint8_t* uncompress_buffer = NULL;
  int uncompress_buffer_size = 0;
  const uint8_t* depth_data =NULL; //= msg->depth.depth_data;
  // 1.2.1 De-compress if necessary:
  if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
    //std:: cout << "compression \n ";
    if(msg->depth.uncompressed_size > uncompress_buffer_size) {
      uncompress_buffer_size = msg->depth.uncompressed_size;
      uncompress_buffer = (uint8_t*) realloc(uncompress_buffer, uncompress_buffer_size);
    }
    unsigned long dlen = msg->depth.uncompressed_size;
    int status = uncompress(uncompress_buffer, &dlen, 
        msg->depth.depth_data, msg->depth.depth_data_nbytes);
    if(status != Z_OK) {
      return;
    }
    depth_data =(uint8_t*) uncompress_buffer;
  }else{
    depth_data = (uint8_t*) msg->depth.depth_data;
  }

  int npixels = msg->depth.width * msg->depth.height;
  if (msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_11BIT){ 
    /////////////////////////////////////////////////////////////////////
    // Freenect Data
    uint16_t* disparity_array = (uint16_t*) malloc(msg->depth.width * msg->depth.height * sizeof(uint16_t)); 
    if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
      int16_t* rdd = (int16_t*) depth_data;
      int i;
      for(i=0; i<npixels; i++) {
        int d = rdd[i];
        disparity_array[i] = d;
      }
    } else {
      fprintf(stderr, "Big endian systems not supported\n");
    }

    /// 2 Calculate transformation matrices:
    double depth_to_rgb_uvd[12];
    double depth_to_depth_xyz[16];
    kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(kcal, depth_to_rgb_uvd);
    kinect_calib_get_depth_uvd_to_depth_xyz_4x4(kcal, depth_to_depth_xyz);
    double depth_to_depth_xyz_trans[16];
    //_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);
    bot_matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);

    // 3 for each depth point find the corresponding xyz and then RGB
    //   then put into PCL structure
    cloud->width    =(int) (msg->depth.width/ (double) decimate_) ;
    cloud->height   =(int) (msg->depth.height/ (double) decimate_);
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    double xyzw2[4];
    int j2=0;
    // NB: the order of these loop was changed... aug 2011. important
    for(int v=0; v<msg->depth.height; v=v+ decimate_) { // t2b state->height 480
      for(int u=0; u<msg->depth.width; u=u+decimate_ ) {  //l2r state->width 640
        // 3.4.1 compute distorted pixel coordinates
        uint16_t disparity = disparity_array[v*msg->depth.width+u];
        double uvd_depth[4] = { u, v, disparity, 1 };
        double uvd_rgb[3];
        _matrix_vector_multiply_3x4_4d(depth_to_rgb_uvd, uvd_depth, uvd_rgb);
        double uv_rect[2] = {
            uvd_rgb[0] / uvd_rgb[2],
            uvd_rgb[1] / uvd_rgb[2]
        };
        double uv_dist[2];
        kinect_calib_distort_rgb_uv(kcal, uv_rect, uv_dist);
        int u_rgb = uv_dist[0] + 0.5;
        int v_rgb = uv_dist[1] + 0.5;
        uint8_t r, g, b;
        if(u_rgb >= msg->depth.width || u_rgb < 0 || v_rgb >= msg->depth.height || v_rgb < 0) {
          r = g = b = 0;
        } else {
          r = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 0];
          g = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 1];
          b = rgb_data[v_rgb*msg->depth.width*3 + u_rgb*3 + 2];
        }
        // 3.4.2 find the xyz location of the points:
        bot_matrix_multiply(depth_to_depth_xyz, 4, 4,
            uvd_depth, 4, 1, xyzw2);

        cloud->points[j2].y = -xyzw2[0]/xyzw2[3];//y right+ (check)
        cloud->points[j2].z = -xyzw2[1]/xyzw2[3];//z up+
        cloud->points[j2].x = xyzw2[2]/xyzw2[3]; //x forward+
        // was bgr...
        cloud->points[j2].b =b;
        cloud->points[j2].r =r;
        cloud->points[j2].g =g;
        j2++;
      }
    }
    free(disparity_array);

  }else if(msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_MM  ){ 
    /////////////////////////////////////////////////////////////////////
    // Openni Data
    // 1.2.2 unpack raw byte data into float values in mm

    // NB: no depth return is given 0 range - and becomes 0,0,0 here
    const uint16_t* val = reinterpret_cast<const uint16_t*>( depth_data );

    cloud->width    =(int) (msg->depth.width/ (double) decimate_) ;
    cloud->height   =(int) (msg->depth.height/ (double) decimate_); 
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    int j2=0;
    for(int v=0; v<msg->depth.height; v=v+ decimate_) { // t2b self->height 480
      for(int u=0; u<msg->depth.width; u=u+decimate_ ) {  //l2r self->width 640
        uint8_t r = rgb_data[v*msg->depth.width*3 + u*3 + 0];
        uint8_t g = rgb_data[v*msg->depth.width*3 + u*3 + 1];
        uint8_t b = rgb_data[v*msg->depth.width*3 + u*3 + 2];
        double constant = 1.0f / kcal->intrinsics_rgb.fx ;
        double disparity_d = val[v*msg->depth.width+u]  / 1000.0; // convert to m
        cloud->points[j2].y =  - (((double) u)- 319.50)*disparity_d*constant; //y right+ (check)
        cloud->points[j2].z = - (((double) v)- 239.50)*disparity_d*constant;  //z up+
        cloud->points[j2].x = disparity_d;  //x forward+
        if (disparity_d==0){ // place null points at negative range... arbitarty decision
          double disparity_d = -0.1; // convert to m
          cloud->points[j2].y =  - (((double) u)- 319.50)*disparity_d*constant; //y right+ (check)
          cloud->points[j2].z =  -(((double) v)- 239.50)*disparity_d*constant;  //z up+
          cloud->points[j2].x = disparity_d;  //x forward+
        }

        cloud->points[j2].b =b;
        cloud->points[j2].r =r;
        cloud->points[j2].g =g;
        j2++;
      }
    }         
  }

  if(msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
    free(uncompress_buffer); // memory leak bug fixed
  }
}



void pointcloud_lcm::unpack_multisense(const uint8_t* depth_data, const uint8_t* color_data, int height, int width, cv::Mat_<double> repro_matrix, 
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool is_rgb){
  // cout << width <<" | "<< height <<" in unpack routine\n";

  int h = height;//msg->images[0].height;
  int w = width;//msg->images[0].width;
  
  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
//  const uint8_t* raw_data= stereob_->getDisparity();//= 3;//msg->images[1].data.data();
  disparity_orig_temp.data = (uchar*) depth_data;   // ... is a simple assignment possible?

  //std::copy(msg->images[1].data.data()             , msg->images[1].data.data() + (msg->images[1].size) ,
  //          disparity_orig_temp.data);
  
  // disparity_orig_temp.data = msg->images[1].data.data();   // ... is a simple assignment possible?
  cv::Mat_<float> disparity_orig(h, w);
  disparity_orig = disparity_orig_temp;
  
  //std::stringstream disparity_fname;
  //disparity_fname << "crl_disparity_lcm_" << msg->utime << ".png";
  //imwrite(disparity_fname.str(),disparity_orig_temp); 
  
  disparity_buf_.resize(h * w);
  cv::Mat_<float> disparity(h, w, &(disparity_buf_[0]));
  disparity = disparity_orig / 16.0;

  // Allocate buffer for reprojection output
  points_buf_.resize(h * w);
  cv::Mat_<cv::Vec3f> points(h, w, &(points_buf_[0]));

  // Do the reprojection in open space
  static const bool handle_missing_values = true;
  cv::reprojectImageTo3D(disparity, points, repro_matrix, handle_missing_values);
  
  
/*  int vv =400; //l2r
  int uu =512; //t2b
  cout << vv <<" " << uu << " | " << disparity( vv, uu) << " | " << points(vv,uu)[0]
              << " " << points(vv,uu)[1]
              << " " << points(vv,uu)[2]
              << "\n";
*/  
  
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->width    =(int) (w/ (double) decimate_) ;
  cloud->height   =(int) (h/ (double) decimate_);
  cloud->is_dense = true;
  cloud->points.resize (cloud->width * cloud->height);  
  // std::cout << cloud->points.size() << " cldsize\n";
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
void pointcloud_lcm::unpack_multisense(const multisense_images_t *msg, cv::Mat_<double> repro_matrix, 
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
    std::cout << "pointcloud_lcm::unpack_multisense | type not understood\n";
    exit(-1);
  }
  
  // TODO: support other modes (as in the renderer)
  if (msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY_ZIPPED ) {
    unsigned long dlen = msg->images[0].width*msg->images[0].height*2 ;//msg->depth.uncompressed_size;
    uncompress(depth_buf_ , &dlen, msg->images[1].data, msg->images[1].size);
  } else{
    std::cout << "pointcloud_lcm::unpack_multisense | depth type not understood\n";
    exit(-1);
  }  
  
  unpack_multisense(depth_buf_, rgb_buf_, msg->images[0].height, msg->images[0].width, repro_matrix, 
                                       cloud, is_rgb);
}


void pointcloud_lcm::unpack_multisense(const multisense::images_t *msg, cv::Mat_<double> repro_matrix, 
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
    std::cout << "pointcloud_lcm::unpack_multisense | image type not understood\n";
    exit(-1);
  }
  
  // TODO: support other modes (as in the renderer)
  if (msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY_ZIPPED ) {
    unsigned long dlen = msg->images[0].width*msg->images[0].height*2 ;//msg->depth.uncompressed_size;
    uncompress(depth_buf_ , &dlen, msg->images[1].data.data(), msg->images[1].size);
  } else{
    std::cout << "pointcloud_lcm::unpack_multisense | depth type not understood\n";
    exit(-1);
  }
  
  unpack_multisense(depth_buf_, rgb_buf_, msg->images[0].height, msg->images[0].width, repro_matrix, 
                                       cloud, is_rgb);
  // unpack_multisense(msg->images[1].data.data(), msg->images[0].data.data(), msg->images[0].height, msg->images[0].width, repro_matrix, 
  //                                     cloud, is_rgb);
}
