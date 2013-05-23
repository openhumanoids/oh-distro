#include <iostream>
#include "image_io_utils.hpp"



//#include "jpeg-utils.h"
//#include "jpeg-utils-ijg.c"
//#include "jpeg-utils-ijg.h"

#include <zlib.h>


using namespace std;


image_io_utils::image_io_utils (lcm_t* publish_lcm_, int width_, int height_):
        publish_lcm_(publish_lcm_){

  // Maximum size of reserved buffer. This will support and images smaller than this also:
  img_buffer_size_ = width_ * height_ * sizeof(int16_t) * 4;
  img_buffer_= new uint8_t[img_buffer_size_];  // x4 was used for zlib in kinect_lcm
  // is x10 necessary for  jpeg? thats waht kinect_lcm assumed
}

/// Added for RGB-to-Gray:
int pixel_convert_8u_rgb_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride)
{
  int i, j;
  for (i=0; i<height; i++) {
    uint8_t *drow = dest + i * dstride;
    const uint8_t *srow = src + i * sstride;
    for (j=0; j<width; j++) {
      drow[j] = 0.2125 * srow[j*3+0] +
        0.7154 * srow[j*3+1] +
        0.0721 * srow[j*3+2];
    }
  }
  return 0;
}

void image_io_utils::decodeStereoImage(const  bot_core::image_t* msg, uint8_t* left_buf, uint8_t* right_buf){
  int h = msg->height/2; /// stacked stereo
  int w = msg->width;
  
  int buf_size = w*h;
  
  switch (msg->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
      memcpy(left_buf,  msg->data.data() , msg->size/2);
      memcpy(right_buf,  msg->data.data() + msg->size/2 , msg->size/2);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
      pixel_convert_8u_rgb_to_8u_gray(  left_buf, w, w, h, msg->data.data(),  w*3);
      pixel_convert_8u_rgb_to_8u_gray(  right_buf, w,  w, h, msg->data.data() + msg->size/2,  w*3);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // This assumes Gray Scale MJPEG
      jpeg_decompress_8u_gray(msg->data.data(), msg->size, img_buffer_,
                              msg->width, msg->height, msg->width);
      std::copy(img_buffer_          , img_buffer_+buf_size   , left_buf);
      std::copy(img_buffer_+buf_size , img_buffer_+2*buf_size , right_buf);
      break;
    default:
      std::cout << "Unrecognized image format\n";
      exit(-1);
      break;
  }  
}




void image_io_utils::jpegImageThenSend(uint8_t* buffer, int64_t utime, int width, int height, int jpeg_quality, std::string channel, int n_colors){

  /*
  /// factor of 4: 800x800 --> 200x200
  Mat src= Mat::zeros( msg->height,msg->width  ,CV_8UC3);
  src.data = msg->data;
  int resize_height = msg->height/self->resize;
  int resize_width  = msg->width/self->resize;
  Mat img = Mat::zeros( resize_height , resize_width ,CV_8UC3);
  cv::resize(src, img, img.size());  // Resize src to img size
  */
  
  int compressed_size =  width*height*n_colors;//image_buf_size;
  int compression_status  = -1;
  if (n_colors == 1){
    int compression_status = jpeg_compress_8u_gray(buffer, width, height, width*n_colors,
                                                     img_buffer_, &compressed_size, jpeg_quality);
  }else if( n_colors ==3) {
    int compression_status = jpeg_compress_8u_rgb  (buffer, width, height, width*n_colors,
                                                     img_buffer_, &compressed_size, jpeg_quality);
  }else {
    std::cout << "number if colors is no correct " << n_colors << "\n";
    
    exit(-1);
  }
  bot_core_image_t msgout_small;
  msgout_small.utime = utime;
  msgout_small.width = width;
  msgout_small.height = height;
  msgout_small.row_stride = n_colors*width;
  msgout_small.size = compressed_size;
  msgout_small.pixelformat = BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG;
  msgout_small.data = img_buffer_;
  msgout_small.nmetadata =0;
  msgout_small.metadata = NULL;
  string channel_out_small = string(channel);// dont appent channel  // + "_COMPRESSED";
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
