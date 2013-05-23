#ifndef image_io_utils_HPP_
#define image_io_utils_HPP_

#include <iostream>
#include <vector>
#include <algorithm>

#include <image_utils/jpeg.h>
#include <lcm/lcm.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/bot_core.hpp>

class image_io_utils {
  public:
    image_io_utils (lcm_t* publish_lcm_, int width_, int height_);
    
    void decodeStereoImage(const  bot_core::image_t* msg, uint8_t* left_buf, uint8_t* right_buf);
    
    void unzipImageThenSend(const bot_core_image_t *msg, std::string channel);
    void unzipImageThenSend(const bot_core::image_t *msg, std::string channel);
    uint8_t* unzipImage(const bot_core_image_t *msg);
    uint8_t* unzipImage(const bot_core::image_t *msg);
    
    void jpegImageThenSend(uint8_t* buffer, int64_t utime, int width, int height, 
			   int jpeg_quality, std::string channel, int n_colors);

    
    void sendImageZipped(uint8_t* buffer, int64_t utime, 
                         int width, int height, int n_colors, 
                         std::string channel);

    void sendImage(uint8_t* buffer, int64_t utime, 
                   int width, int height, 
                   int n_colors, std::string channel);
  private:
    lcm_t *publish_lcm_; 
    int width_, height_;

    int img_buffer_size_;
    uint8_t* img_buffer_;
};






#endif
