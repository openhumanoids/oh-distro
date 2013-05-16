#include <iostream>
#include "image_io_utils.hpp"



//#include "jpeg-utils.h"
//#include "jpeg-utils-ijg.c"
//#include "jpeg-utils-ijg.h"

#include <zlib.h>


using namespace std;


image_io_utils::image_io_utils (lcm_t* publish_lcm_):
        publish_lcm_(publish_lcm_){

  // Maximum size of reserved buffer. This will support and images smaller than this also:
  img_buffer_size_ = 800 * 800 * sizeof(int16_t) * 4;
  img_buffer_= new uint8_t[img_buffer_size_];  // x4 was used for zlib in kinect_lcm
  // is x10 necessary for  jpeg? thats waht kinect_lcm assumed
}




void image_io_utils::jpegImageThenSend(uint8_t* buffer, int64_t utime, int width, int height, int jpeg_quality, std::string channel){

  /*
  /// factor of 4: 800x800 --> 200x200
  Mat src= Mat::zeros( msg->height,msg->width  ,CV_8UC3);
  src.data = msg->data;
  int resize_height = msg->height/self->resize;
  int resize_width  = msg->width/self->resize;
  Mat img = Mat::zeros( resize_height , resize_width ,CV_8UC3);
  cv::resize(src, img, img.size());  // Resize src to img size
  */
  
  int compressed_size =  width*height*3;//image_buf_size;
  int compression_status = jpeg_compress_8u_rgb (buffer, width, height, width*3,
                                                     img_buffer_, &compressed_size, jpeg_quality);
  
  bot_core_image_t msgout_small;
  msgout_small.utime = utime;
  msgout_small.width = width;
  msgout_small.height = height;
  msgout_small.row_stride = 3*width;
  msgout_small.size = compressed_size;
  msgout_small.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
  msgout_small.data = img_buffer_;
  msgout_small.nmetadata =0;
  msgout_small.metadata = NULL;
  string channel_out_small = string(channel) + "_COMPRESSED";
  bot_core_image_t_publish(publish_lcm_, channel_out_small.c_str(), &msgout_small);
  
}

// assumes a zipped gray scale image:
void image_io_utils::unzipImageThenSend(const bot_core_image_t *msg, std::string channel){
  unsigned long dlen = msg->width*msg->height;
  uncompress(img_buffer_, &dlen, msg->data, msg->size);
  sendImage(img_buffer_, msg->utime, msg->width, msg->height, 1, string(channel + "_UNZIPPED")  );
}

// assumes a zipped gray scale image:
void image_io_utils::unzipImageThenSend(const bot_core::image_t *msg, std::string channel){
  unsigned long dlen = msg->width*msg->height;
  uncompress(img_buffer_, &dlen, msg->data.data(), msg->size);
  sendImage(img_buffer_, msg->utime, msg->width, msg->height, 1, string(channel + "_UNZIPPED")  );
}

// assumes a zipped gray scale image: 
uint8_t* image_io_utils::unzipImage(const bot_core_image_t *msg){
  unsigned long dlen = msg->width*msg->height; // msg->depth.uncompressed_size;
  uncompress(img_buffer_, &dlen, msg->data, msg->size);
  return img_buffer_;
}

// assumes a zipped gray scale image: CPP CPP CPP CPP
uint8_t* image_io_utils::unzipImage(const bot_core::image_t *msg){
  unsigned long dlen = msg->width*msg->height; // msg->depth.uncompressed_size;
  uncompress(img_buffer_, &dlen, msg->data.data(), msg->size);
  return img_buffer_;
}


void image_io_utils::sendImageZipped(uint8_t* buffer, int64_t utime, 
  int width, int height, int n_colors, std::string channel){
  int isize= width* height;

  int uncompressed_size = isize*n_colors;
  unsigned long compressed_size = img_buffer_size_;
  compress2(img_buffer_, &compressed_size, (const Bytef*) buffer, uncompressed_size,
            Z_BEST_SPEED);
  
  bot_core_image_t image;
  image.utime =utime;
  image.width =width;
  image.height=height;
  image.row_stride =n_colors*width; // this is useless if doing zip

  // This label will be invalid...
  if (n_colors==1){
    image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  }else{
    image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
  }

  image.size = (int) compressed_size;
  image.data = img_buffer_;
  image.nmetadata =0;
  image.metadata = NULL;
  bot_core_image_t_publish( publish_lcm_, channel.c_str(), &image);  
}


void image_io_utils::sendImage(uint8_t* buffer, int64_t utime, int width, int height, int n_colors, std::string channel)
{
  int isize= width* height;

  bot_core_image_t image;
  image.utime =utime;
  image.width =width;
  image.height=height;
  image.row_stride =n_colors*width;


  if (n_colors==1){
    image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  }else{
    image.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
  }

  image.size =n_colors*isize;
  image.data = buffer;

  image.nmetadata =0;
  image.metadata = NULL;

  bot_core_image_t_publish( publish_lcm_, channel.c_str(), &image);  
}
