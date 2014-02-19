/**
 * @file camera.cpp
 *
 * Copyright 2013
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#include "camera.h"

#include <arpa/inet.h>

using namespace crl::multisense;




namespace multisense_ros {

namespace { // anonymous

//
// All of the data sources that we control here

const DataSource allImageSources = (Source_Luma_Left            |
                                    Source_Luma_Right           |
                                    Source_Luma_Rectified_Left  |
                                    Source_Luma_Rectified_Right |
                                    Source_Chroma_Left          |
                                    Source_Chroma_Right         |
                                    Source_Raw_Right            |
                                    Source_Disparity);
//
// Shims for C-style driver callbacks 
void rectCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rectCallback(header); }
void depthCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->depthCallback(header); }
void colorCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->colorImageCallback(header); }
void rightRectCB(const image::Header& header, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rightRectCallback(header); }

}; // anonymous

struct Camera::ColorData {
  Camera *cam_;
  uint8_t *rgbP_;
  uint8_t *lumaP_;
  uint8_t *rgbP_rect_;
  uint8_t *grayP_;
  bool got_luma_;
  int64_t luma_frame_id_;
  CvMat *calibration_map_1_;
  CvMat *calibration_map_2_;
  IplImage *destImageP_;
  bot_core::image_t msg_;
  int64_t lcm_frame_id_;
  std::list<std::shared_ptr<bot_core::image_t> > img_queue_;
  std::string name_;

  static const int width_ = 1024;
  static const int height_ = 544;

  ColorData(Camera* cam, const int npixels, const std::string& name) {
    cam_ = cam;
    name_ = name;
    rgbP_ = (uint8_t*) malloc(npixels*3);
    lumaP_ = (uint8_t*) malloc(npixels*3);
    grayP_ = (uint8_t*) malloc(npixels);
    rgbP_rect_ = (uint8_t*) malloc(npixels*3);
    got_luma_ = false;
    luma_frame_id_ = 0;
    calibration_map_1_ = calibration_map_2_ = NULL;
    destImageP_ = NULL;
  }

  void createCalibMaps(crl::multisense::image::Calibration::Data& in_cal_data) {
    calibration_map_1_ = cvCreateMat(cam_->image_config_.height(),
                                     cam_->image_config_.width(), CV_32F);
    calibration_map_2_ = cvCreateMat(cam_->image_config_.height(),
                                     cam_->image_config_.width(), CV_32F);

    // Calibration from sensor is for 2 Mpix, must adjust here (TODO: fix on firmware side, this
    // will get messy when we support arbitrary resolutions.)
    
    auto cal_data = in_cal_data;
    if (width_ == cam_->image_config_.width()) {
      cal_data.M[0][0] /= 2.0;
      cal_data.M[0][2] /= 2.0;
      cal_data.M[1][1] /= 2.0;
      cal_data.M[1][2] /= 2.0;
      cal_data.P[0][0] /= 2.0;
      cal_data.P[0][2] /= 2.0;
      cal_data.P[0][3] /= 2.0;
      cal_data.P[1][1] /= 2.0;
      cal_data.P[1][2] /= 2.0;
    }

    CvMat M1 = cvMat(3, 3, CV_32F, &cal_data.M);
    CvMat D1 = cvMat(1, 8, CV_32F, &cal_data.D);
    CvMat R1 = cvMat(3, 3, CV_32F, &cal_data.R);
    CvMat P1 = cvMat(3, 4, CV_32F, &cal_data.P);
    
    cvInitUndistortRectifyMap(&M1, &D1, &R1, &P1,
                              calibration_map_1_, calibration_map_2_);
  }

  //
  // Convert YCbCr 4:2:0 to RGB
  // TODO: speed this up
  void yuvToRgb(const void* imageDataP) {

    const uint8_t *chromaP = reinterpret_cast<const uint8_t*>(imageDataP);
    const uint32_t rgbStride = width_ * 3;

    for(uint32_t y=0; y<height_; y++) {
      for(uint32_t x=0; x<width_; x++) {

        const uint32_t lumaOffset   = (y * width_) + x;
        const uint32_t chromaOffset = 2 * (((y/2) * (width_/2)) + (x/2));
                    
        const float px_y  = static_cast<float>(lumaP_[lumaOffset]);
        const float px_cb = static_cast<float>(chromaP[chromaOffset+0]) - 128.0f;
        const float px_cr = static_cast<float>(chromaP[chromaOffset+1]) - 128.0f;

        float px_r  = px_y +                    1.402f   * px_cr;
        float px_g  = px_y - 0.34414f * px_cb - 0.71414f * px_cr;
        float px_b  = px_y + 1.772f   * px_cb;

        if (px_r < 0.0f)        px_r = 0.0f;
        else if (px_r > 255.0f) px_r = 255.0f;
        if (px_g < 0.0f)        px_g = 0.0f;
        else if (px_g > 255.0f) px_g = 255.0f;
        if (px_b < 0.0f)        px_b = 0.0f;
        else if (px_b > 255.0f) px_b = 255.0f;

        const uint32_t rgbOffset = (y * rgbStride) + (3 * x);

        rgbP_[rgbOffset + 0] = static_cast<uint8_t>(px_r);
        rgbP_[rgbOffset + 1] = static_cast<uint8_t>(px_g);
        rgbP_[rgbOffset + 2] = static_cast<uint8_t>(px_b);
      }
    }
  }

  void rectify() {
    std::unique_lock<std::mutex> lock(cam_->cal_lock_);
    if (width_  != cam_->image_config_.width() ||
        height_ != cam_->image_config_.height())
      printf("calibration/image size mismatch: image=%dx%d, calibration=%dx%d\n",
             width_, height_, cam_->image_config_.width(), cam_->image_config_.height());
    else if (NULL == calibration_map_1_ || NULL == calibration_map_2_)
      printf("undistort maps not initialized\n");
    else {

      const CvScalar outlierColor = {{0.0}};
      IplImage *sourceImageP  = cvCreateImageHeader(cvSize(width_, height_), IPL_DEPTH_8U, 3);
      sourceImageP->imageData = reinterpret_cast<char*>( rgbP_ );
      destImageP_ = cvCreateImageHeader(cvSize(width_, height_), IPL_DEPTH_8U, 3);
      destImageP_->imageData   = reinterpret_cast<char*>( rgbP_rect_ );
      
      cvRemap(sourceImageP, destImageP_, calibration_map_1_, calibration_map_2_,
              CV_INTER_LINEAR, outlierColor);
      cvReleaseImageHeader(&sourceImageP);
    }
  }

  void setRectImage(const image::Header header,
                    const void* imageDataP) {
    destImageP_ = cvCreateImageHeader(cvSize(header.width, header.height), IPL_DEPTH_8U, 1);
    memcpy(grayP_, imageDataP, header.width*header.height);
    destImageP_->imageData = (char*)grayP_;
  }

  void pushMessage(const int64_t data_utime, const int64_t frame_id) {
    if (cam_->config_.do_jpeg_compress_){
      std::vector<int> params;
      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(cam_->config_.jpeg_quality_);
      if (destImageP_->nChannels > 1) {
        cvCvtColor( destImageP_, destImageP_, CV_BGR2RGB);
      }
      cv::Mat img = cv::cvarrToMat(destImageP_);
      if (!cv::imencode(".jpg", img, msg_.data, params)) {
        printf("Error encoding jpeg image\n");
      }
      msg_.size = msg_.data.size();
      msg_.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG; 
    }else{
      int isize = width_*height_*destImageP_->nChannels;
      msg_.data.resize(isize);
      memcpy(&msg_.data[0], destImageP_->imageData, isize);
      // destImageP->imageData   = reinterpret_cast<char*>(&(msg.data[0]));
      msg_.size = isize;
      msg_.pixelformat = destImageP_->nChannels==1 ?
        bot_core::image_t::PIXEL_FORMAT_RGB : bot_core::image_t::PIXEL_FORMAT_GRAY;
    }
    
    msg_.utime = data_utime;
    msg_.width = width_;
    msg_.height = height_;
    msg_.nmetadata = 0;
    msg_.row_stride= destImageP_->nChannels*width_;
    //printf("left luma frame id: %lld", left_luma_frame_id_);
    //printf("leftchromaframe id: %lld", header.frameId);
    lcm_frame_id_ = frame_id;

    // enqueue and notify; publish is handled by publisher class
    std::shared_ptr<bot_core::image_t> img;
    img.reset(new bot_core::image_t(msg_));
    std::unique_lock<std::mutex> lock(cam_->queue_mutex_);
    img_queue_.push_back(img);
    while (img_queue_.size() > cam_->max_queue_len_) {
      img_queue_.pop_front();
      if (cam_->warn_queue_) {
        printf("exceeded buffer size; dropped %s image\n", name_.c_str());
      }
    }
    cam_->queue_condition_.notify_one();
    
    cvReleaseImageHeader(&destImageP_);
  }

};

struct Camera::Publisher {
  Camera* camera_;
  bool running_;
  std::thread thread_;

  void operator()() {
    running_ = true;
    bool use_left(false), use_right(false), use_disp(false);
    int mode = camera_->config_.output_mode_;
    switch(mode) {
    case 0:   use_left = use_disp = true;              break;
    case 3:   use_left = use_right = use_disp = true;  break;
    default:  running_ = false;                        break;
    }

    auto& msg = camera_->multisense_msg_out_;
    msg.n_images = use_left + use_right + use_disp;
    msg.images.resize(msg.n_images);
    msg.image_types.clear();
    int disp_type = camera_->config_.do_zlib_compress_ ?
      multisense::images_t::DISPARITY_ZIPPED : multisense::images_t::DISPARITY;
    if (use_left) msg.image_types.push_back((int)multisense::images_t::LEFT);
    if (use_disp) msg.image_types.push_back(disp_type);
    if (use_right) msg.image_types.push_back((int)multisense::images_t::RIGHT);

    while (running_) {
      // lock queues
      std::unique_lock<std::mutex> lock(camera_->queue_mutex_);
      camera_->queue_condition_.wait_for(lock, std::chrono::milliseconds(100));

      // for each left image, find corresponding depth image and publish
      auto leftIter = camera_->left_data_->img_queue_.begin();
      for (; leftIter != camera_->left_data_->img_queue_.end();) {
        bool published = false;
        auto dispIter = camera_->disp_img_queue_.begin();
        for (; dispIter != camera_->disp_img_queue_.end(); ++dispIter) {
          if ((*leftIter)->utime == (*dispIter)->utime) {
            msg.utime = (*leftIter)->utime;
            msg.images[0] = **leftIter;
            msg.images[1] = **dispIter;
            bool found = true;
            if (mode == 3) {
              found = false;
              auto rightIter = camera_->right_data_->img_queue_.begin();
              for (; rightIter != camera_->right_data_->img_queue_.end();
                   ++rightIter) {
                if ((*rightIter)->utime == (*leftIter)->utime) {
                  msg.images[2] = **rightIter;
                  camera_->right_data_->img_queue_.erase(rightIter);
                  found = true;
                  break;
                }
              }
            }
            if (found) {
              camera_->lcm_publish_.publish("CAMERA", &msg);
              camera_->disp_img_queue_.erase(dispIter);
              published = true;
              printf("published %d images at %ld\n", msg.n_images, (*leftIter)->utime);
              break;
            }
          }
        }
        if (published) {
          leftIter = camera_->left_data_->img_queue_.erase(leftIter);
        }
        else {
          ++leftIter;
        }
      }
    }
  }
};

Camera::Camera(Channel* driver, CameraConfig& config_) :
    driver_(driver),
    max_queue_len_(30),
    warn_queue_(false),
    capture_fps_(0.0f),
    lcm_disp_type_(multisense::images_t::DISPARITY),
    config_(config_)
{

    if(!lcm_publish_.good()){
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }
    // allocate space for zlib compressing depth data
    depth_compress_buf_size_ = 1024 * 544 * sizeof(int16_t) * 4;
    depth_compress_buf_ = (uint8_t*) malloc(depth_compress_buf_size_);
    
    int npixels = 1024*544;

    left_data_.reset(new ColorData(this, npixels, "LEFT"));
    right_data_.reset(new ColorData(this, npixels, "RIGHT"));
    lcm_disp_frame_id_ = -1;
    left_data_->lcm_frame_id_ = -2;
    right_data_->lcm_frame_id_ = -3;
    
    //
    // All image streams off

    stop();

    //
    // Publish image calibration

    if (Status_Ok != driver_->getImageCalibration(image_calibration_))
        printf("Failed to query image calibration\n");


    //
    // Get current sensor configuration

    {
        std::unique_lock<std::mutex> lock(cal_lock_);
        
        // Get the camera config (TODO: clean this up to better handle multiple resolutions)
        if (Status_Ok != driver_->getImageConfig(image_config_)) {
            printf("failed to query sensor configuration\n");
            return;
        }

        // For local rectification of color images
        left_data_->createCalibMaps(image_calibration_.left);
        right_data_->createCalibMaps(image_calibration_.right);
    }

    // start publish thread
    publisher_.reset(new Publisher());
    publisher_->camera_ = this;
    publisher_->thread_ = std::thread(std::ref(*publisher_));

    //
    // Add driver-level callbacks.
    //
    //    -Driver creates individual background thread for each callback.
    //    -Images are queued (depth=5) per callback, with oldest silently dropped if not keeping up.
    //    -All images presented are backed by a referenced buffer system (no copying of image data is done.)
    driver_->addIsolatedCallback(rectCB,  Source_Luma_Rectified_Left | Source_Luma_Rectified_Right, this);
    driver_->addIsolatedCallback(depthCB, Source_Disparity, this);
    driver_->addIsolatedCallback(colorCB, Source_Luma_Left | Source_Chroma_Left, this);
    driver_->addIsolatedCallback(colorCB, Source_Luma_Right | Source_Chroma_Right, this);
    driver_->addIsolatedCallback(rightRectCB, Source_Luma_Rectified_Right, this);
    

    if ( config_.output_mode_ ==0){
      connectStream(Source_Disparity);
      connectStream(Source_Luma_Left);
      connectStream(Source_Chroma_Left);
      warn_queue_ = true;
    } else if ( config_.output_mode_ ==1){
      connectStream(Source_Luma_Rectified_Left);
      connectStream(Source_Luma_Rectified_Right);
    } else if ( config_.output_mode_ ==2){
      connectStream(Source_Luma_Left);
      connectStream(Source_Chroma_Left);
    } else if ( config_.output_mode_ ==3){
      connectStream(Source_Disparity);
      connectStream(Source_Luma_Left);
      connectStream(Source_Chroma_Left);
      connectStream(Source_Luma_Rectified_Right);
      warn_queue_ = true;
    }

    // Apply input config:
    applyConfig(config_);
    

    verbose_=false;
}

Camera::~Camera( )
{
    stop();
}




void Camera::applyConfig(CameraConfig& config){
  std::cout << "Applying new config:\n";  
  std::cout << "do_jpeg_compress_ " << config.do_jpeg_compress_ << "\n"; 
  std::cout << "do_zlib_compress_ " << config.do_zlib_compress_ << "\n";
  std::cout << "agc_ " << config.agc_ << "\n";
  std::cout << "leds_flash_ " << config.leds_flash_ << "\n";
  std::cout << "leds_duty_cycle_ " << config.leds_duty_cycle_ << "\n";
  
  //const float radiansPerSecondToRpm = 9.54929659643;
  if (fabs(config.spindle_rpm_) <= 25) {
    driver_->setMotorSpeed(config.spindle_rpm_);
  }
  
  image::Config cfg;

  Status status = driver_->getImageConfig(cfg);
  if (Status_Ok != status) {
      printf("Failed to query image config. Error code %d", status);
      return;
  }
  if (config.fps_ >= 0) cfg.setFps(config.fps_);
  if (config.gain_ >= 0) cfg.setGain(config.gain_);
  if (config.agc_ >= 0) {
    bool automatic = config.agc_ > 0;
    cfg.setAutoExposure(automatic);
    cfg.setAutoWhiteBalance(automatic);
  }
    
  status = driver_->setImageConfig(cfg);
  if (Status_Ok != status)
      printf("Failed to set image config. Error code %d", status);

    
    lighting::Config leds;
    leds.setFlash(config.leds_flash_);
    leds.setDutyCycle(config.leds_duty_cycle_);

    status = driver_->setLightingConfig(leds);
    if (Status_Ok != status)
        printf("Failed to set lighting config. Error code %d\n", status);    
  
}




void Camera::rectCallback(const image::Header& header)
{    
    /*
    if (Source_Luma_Rectified_Left  == header.source){
        printf("Got L\n");
    }else if (Source_Luma_Rectified_Right  == header.source){
        printf("Got R\n");
    }
    */

    if (Source_Luma_Rectified_Left  != header.source &&
        Source_Luma_Rectified_Right != header.source) {
     
        printf("Unexpected image source: 0x%x\n", header.source);
        return;
    }

    int64_t data_utime = header.timeSeconds*1E6 + header.timeMicroSeconds;
    const uint32_t imageSize = header.width * header.height;
    
    // TODO: add compression to left/right camera images

    switch(header.source) {
    case Source_Luma_Rectified_Left:

        // LCM Left:        
        left_data_->msg_.data.resize( imageSize);
        memcpy(&left_data_->msg_.data[0], header.imageDataP, imageSize);
        left_data_->msg_.size =imageSize;
        left_data_->msg_.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY; 
        left_data_->msg_.utime = data_utime;
        left_data_->msg_.width = header.width;
        left_data_->msg_.height = header.height;
        left_data_->msg_.nmetadata =0;
        left_data_->msg_.row_stride=header.width;
        left_data_->lcm_frame_id_ = header.frameId;
        //lcm_publish_.publish("CAMERALEFT", &left_data_->msg_);
        break;
    case Source_Luma_Rectified_Right:

        // LCM Right:        
        right_data_->msg_.data.resize( imageSize);
        memcpy(&right_data_->msg_.data[0], header.imageDataP, imageSize);
        right_data_->msg_.size =imageSize;
        right_data_->msg_.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY; 
        right_data_->msg_.utime = data_utime;
        right_data_->msg_.width = header.width;
        right_data_->msg_.height = header.height;
        right_data_->msg_.nmetadata =0;
        right_data_->msg_.row_stride=header.width;
        right_data_->lcm_frame_id_ = header.frameId;
        //lcm_publish_.publish("CAMERARIGHT", &right_data_->msg_);        
        break;
    }
   
    if ( left_data_->lcm_frame_id_ == right_data_->lcm_frame_id_ ){
        // publish sync'ed left/right images:
        multisense_msg_out_.utime = data_utime;
        multisense_msg_out_.n_images =2;
        multisense_msg_out_.image_types.resize(2);
        multisense_msg_out_.image_types[0] = multisense::images_t::LEFT;
        multisense_msg_out_.image_types[1] = multisense::images_t::RIGHT;
        multisense_msg_out_.images.resize(2);
        multisense_msg_out_.images[0]= left_data_->msg_;
        multisense_msg_out_.images[1]= right_data_->msg_;
        //printf("Syncd frames. publish pair [id %lld]", header.frameId);
        lcm_publish_.publish("CAMERA", &multisense_msg_out_);
    }else{
      // This error will happen for about 50% of frames
      //printf("Left [%lld] and Right [%lld]: Frame Ids dont match", left_data_->msg_frame_id_, right_data_->msg_frame_id_);
    }
    
}

void Camera::depthCallback(const image::Header& header)
{
    if (verbose_){
      printf("depth\n");
    }

    if (Source_Disparity != header.source) {
        printf("Unexpected image source: 0x%x\n", header.source);
        return;
    }

    /*    
    const float    bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint32_t depthSize = header.height * header.width * sizeof(float);
    const uint32_t imageSize = header.width * header.height;
    */

    if (16 == header.bitsPerPixel) {
        // LCM Disparity:
        int64_t data_utime = header.timeSeconds*1E6 + header.timeMicroSeconds;
        int n_bytes=2; // 2 bytes per value
        int isize = n_bytes*header.width*header.height;
        lcm_disp_.utime = data_utime;
        lcm_disp_.width = header.width;
        lcm_disp_.height = header.height;
        lcm_disp_.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY;
        lcm_disp_.nmetadata = 0;
        lcm_disp_.row_stride = n_bytes*header.width;
        if (config_.do_zlib_compress_){
          int uncompressed_size = isize;
          unsigned long compressed_size = depth_compress_buf_size_;
          compress2( depth_compress_buf_, &compressed_size, (const Bytef*) header.imageDataP, uncompressed_size,
                  Z_BEST_SPEED);
          lcm_disp_.data.resize(compressed_size);
          memcpy(&lcm_disp_.data[ 0 ], depth_compress_buf_, compressed_size ); 
          lcm_disp_.size = compressed_size;
          lcm_disp_type_ = multisense::images_t::DISPARITY_ZIPPED;;
        }else{
          lcm_disp_.data.resize(isize);
          memcpy(&lcm_disp_.data[ 0 ], header.imageDataP, isize);
          lcm_disp_.size = isize;
          lcm_disp_type_ = multisense::images_t::DISPARITY;
        }
        // printf("disp      frame id: %lld", header.frameId);
        lcm_disp_frame_id_ = header.frameId;
        //////////////////////////////////////////////// 

        {
          std::shared_ptr<bot_core::image_t> img;
          img.reset(new bot_core::image_t(lcm_disp_));
          std::unique_lock<std::mutex> lock(queue_mutex_);
          disp_img_queue_.push_back(img);
          while (disp_img_queue_.size() > max_queue_len_) {
            disp_img_queue_.pop_front();
            printf("dropped disp image\n");
          }
          queue_condition_.notify_one();
        }
          
    } else {
        printf("unsupported disparity bpp: %d\n", header.bitsPerPixel);
        return;
    }

}



void Camera::rightRectCallback(const image::Header& header)
{
  if (header.source != Source_Luma_Rectified_Right) return;
  right_data_->setRectImage(header, header.imageDataP);
  int64_t data_utime = header.timeSeconds*1E6 + header.timeMicroSeconds;
  right_data_->pushMessage(data_utime, header.frameId);
}


void Camera::colorImageCallback(const image::Header& header)
{
    if (verbose_) printf("colorImageCallback\n");

    // The luma image is currently published before
    // the matching chroma image. 

    int64_t data_utime = header.timeSeconds*1E6 + header.timeMicroSeconds;
    if (Source_Luma_Left == header.source) {
        if (false == left_data_->got_luma_) {
            const uint32_t imageSize = header.width * header.height;
            memcpy(left_data_->lumaP_, header.imageDataP, imageSize);
            left_data_->luma_frame_id_ = header.frameId;
            left_data_->got_luma_      = true;
        }
    } else if (Source_Luma_Right == header.source) {
        if (false == right_data_->got_luma_) {
            const uint32_t imageSize = header.width * header.height;
            memcpy(right_data_->lumaP_, header.imageDataP, imageSize);
            right_data_->luma_frame_id_ = header.frameId;
            right_data_->got_luma_      = true;
        }
    }

    else if (Source_Chroma_Left == header.source) {
        if (header.frameId == left_data_->luma_frame_id_) {
            left_data_->yuvToRgb(header.imageDataP);
            left_data_->rectify();
            left_data_->pushMessage(data_utime, header.frameId);
            if (config_.output_mode_ == 2){
                lcm_publish_.publish("CAMERA_LEFT", &left_data_->msg_);
            }
        }
        left_data_->got_luma_ = false;
    } else if (Source_Chroma_Right == header.source) {
        if (header.frameId == right_data_->luma_frame_id_) {
            right_data_->yuvToRgb(header.imageDataP);
            right_data_->rectify();
            right_data_->pushMessage(data_utime, header.frameId);
        }
        right_data_->got_luma_ = false;
    }
}

void Camera::stop()
{
    if (publisher_ != NULL) {
      publisher_->running_ = false;
      if (publisher_->thread_.joinable()) publisher_->thread_.join();
    }

    {
      std::unique_lock<std::mutex> lock(stream_lock_);

      Status status;
      lighting::Config leds;
    
      leds.setFlash(false);
      leds.setDutyCycle( 0.00 * 100.0);
      status = driver_->setLightingConfig(leds);
      if (Status_Ok != status)
        printf("Failed to set lighting config. Error code %d\n", status);    
    
      status = driver_->setMotorSpeed( 0);    
    

      stream_map_.clear();

      status = driver_->stopStreams(allImageSources);
      if (Status_Ok != status)
        printf("Failed to stop all streams: Error code %d\n",
               status);
    }

    // wait for pending messages to shake out
    // TODO: can probably wait for a condition instead
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void Camera::updateDiagnostics()
{
    DataSource streamsEnabled = 0;

    if (Status_Ok != driver_->getEnabledStreams(streamsEnabled)) {
        printf("failed to query enabled streams\n");
        return;
    }

    streamsEnabled &= allImageSources;

    if (Source_Disparity & streamsEnabled) {
        streamsEnabled &= ~Source_Disparity;
    }
    float aggregateImageRate = 0;

    for(uint32_t i=0; i<32; i++) 
        if ((1<<i) & streamsEnabled)
            aggregateImageRate += capture_fps_;
}

void Camera::connectStream(DataSource enableMask)
{    
    std::unique_lock<std::mutex> lock(stream_lock_);

    DataSource notStarted = 0;

    for(uint32_t i=0; i<32; i++) 
        if ((1<<i) & enableMask && 0 == stream_map_[(1<<i)]++)
            notStarted |= (1<<i);

    if (0 != notStarted) {
        Status status = driver_->startStreams(notStarted);
        if (Status_Ok != status)
            printf("Failed to start streams 0x%x. Error code %d\n", 
                      notStarted, status);
    }

    updateDiagnostics();
}

void Camera::disconnectStream(DataSource disableMask)
{
    std::unique_lock<std::mutex> lock(stream_lock_);

    DataSource notStopped = 0;

    for(uint32_t i=0; i<32; i++) 
        if ((1<<i) & disableMask && 0 == --stream_map_[(1<<i)])
            notStopped |= (1<<i);

    if (0 != notStopped) {
        Status status = driver_->stopStreams(notStopped);
        if (Status_Ok != status)
            printf("Failed to stop streams 0x%x. Error code %d\n", 
                      notStopped, status);
    }

    updateDiagnostics();
}


} // namespace
