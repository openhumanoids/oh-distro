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
                                    Source_Disparity);
//
// Shims for C-style driver callbacks 
void rectCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rectCallback(header, imageDataP); }
void depthCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->depthCallback(header, imageDataP); }
void colorCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->colorImageCallback(header, imageDataP); }


}; // anonymous

struct Camera::Publisher {
  Camera* camera_;
  bool running_;
  std::thread thread_;

  void operator()() {
    running_ = (camera_->config_.output_mode_ == 0);
    while (running_) {
      // lock queues
      std::unique_lock<std::mutex> lock(camera_->queue_mutex_);
      camera_->queue_condition_.wait_for(lock, std::chrono::milliseconds(100));

      // for each left image, find corresponding depth image and publish
      bool any_published = false;
      auto leftIter = camera_->left_img_queue_.begin();
      for (; leftIter != camera_->left_img_queue_.end();) {
        bool published = false;
        auto rightIter = camera_->right_img_queue_.begin();
        for (; rightIter != camera_->right_img_queue_.end(); ++rightIter) {
          if ((*leftIter)->utime == (*rightIter)->utime) {
            camera_->multisense_msg_out_.utime = (*leftIter)->utime;
            camera_->multisense_msg_out_.n_images =2;
            camera_->multisense_msg_out_.images[0]= **leftIter;
            camera_->multisense_msg_out_.images[1]= **rightIter;
            camera_->lcm_publish_.publish("CAMERA", &camera_->multisense_msg_out_);
            published = true;
            any_published = true;
            printf("published left/disp pair at %ld\n", (*leftIter)->utime);
            break;
          }
        }
        if (published) {
          leftIter = camera_->left_img_queue_.erase(leftIter);
          camera_->right_img_queue_.erase(rightIter);
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
    got_left_luma_(false),
    left_luma_frame_id_(0),
    left_rect_frame_id_(0),
    calibration_map_left_1_(NULL),
    calibration_map_left_2_(NULL),
    max_queue_len_(30),
    capture_fps_(0.0f),
    config_(config_)
{

    if(!lcm_publish_.good()){
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }
    lcm_disp_frame_id_ = -1;
    lcm_left_frame_id_ = -2;
    lcm_right_frame_id_ = -3;
    multisense_msg_out_.image_types.push_back(0);// multisense::images_t::LEFT );
    multisense_msg_out_.image_types.push_back(2);// multisense::images_t::DISPARITY );
    multisense_msg_out_.images.push_back(lcm_left_);
    multisense_msg_out_.images.push_back(lcm_disp_);
    // allocate space for zlib compressing depth data
    depth_compress_buf_size_ = 1024 * 544 * sizeof(int16_t) * 4;
    depth_compress_buf_ = (uint8_t*) malloc(depth_compress_buf_size_);
    
    int npixels = 1024*544;
    rgbP = (uint8_t*) malloc(npixels*3);
    lumaP = (uint8_t*) malloc(npixels*3);
    rgbP_rect = (uint8_t*) malloc(npixels*3);

    
    //
    // All image streams off

    stop();

    //
    // Publish image calibration

    if (Status_Ok != driver_->getImageCalibration(image_calibration_))
        printf("Failed to query image calibration\n");


    //
    // Get current sensor configuration

    queryConfig();

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
    

    if ( config_.output_mode_ ==0){
      connectStream(Source_Disparity);
      connectStream(Source_Luma_Left);
      connectStream(Source_Chroma_Left);
    } else if ( config_.output_mode_ ==1){
      connectStream(Source_Luma_Rectified_Left);
      connectStream(Source_Luma_Rectified_Right);
    } else if ( config_.output_mode_ ==2){
      connectStream(Source_Luma_Left);
      connectStream(Source_Chroma_Left);
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
  
  
  const float radiansPerSecondToRpm = 9.54929659643;
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
    
  status = driver_->setImageConfig(cfg);
  if (Status_Ok != status)
      printf("Failed to set image config. Error code %d", status);

    
    lighting::Config leds;
    /*
    if (false == config.lighting) {
        leds.setFlash(false);
        leds.setDutyCycle(0.0);
    } else {
        leds.setFlash(config.flash);
        leds.setDutyCycle(config.led_duty_cycle * 100.0);
    }
    */
    
    
    leds.setFlash(true);
    leds.setDutyCycle( 0.0 * 100.0);

    status = driver_->setLightingConfig(leds);
    if (Status_Ok != status)
        printf("Failed to set lighting config. Error code %d\n", status);    
  
}




void Camera::rectCallback(const image::Header& header,
                          const void*          imageDataP)
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
        lcm_left_.data.resize( imageSize);
        memcpy(&lcm_left_.data[0], imageDataP, imageSize);
        lcm_left_.size =imageSize;
        lcm_left_.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY; 
        lcm_left_.utime = data_utime;
        lcm_left_.width = header.width;
        lcm_left_.height = header.height;
        lcm_left_.nmetadata =0;
        lcm_left_.row_stride=header.width;
        lcm_left_frame_id_ = header.frameId;
        multisense_msg_out_.image_types[0] = 0; // left
        //lcm_publish_.publish("CAMERALEFT", &lcm_left_);
        break;
    case Source_Luma_Rectified_Right:

        // LCM Right:        
        lcm_right_.data.resize( imageSize);
        memcpy(&lcm_right_.data[0], imageDataP, imageSize);
        lcm_right_.size =imageSize;
        lcm_right_.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY; 
        lcm_right_.utime = data_utime;
        lcm_right_.width = header.width;
        lcm_right_.height = header.height;
        lcm_right_.nmetadata =0;
        lcm_right_.row_stride=header.width;
        lcm_right_frame_id_ = header.frameId;
        multisense_msg_out_.image_types[1] = 1; // right
        //lcm_publish_.publish("CAMERARIGHT", &lcm_right_);        
        break;
    }
   
    if ( lcm_left_frame_id_ == lcm_right_frame_id_ ){
        // publish sync'ed left/right images:
        multisense_msg_out_.utime = data_utime;
        multisense_msg_out_.n_images =2;
        multisense_msg_out_.images[0]= lcm_left_;
        multisense_msg_out_.images[1]= lcm_right_;
        //printf("Syncd frames. publish pair [id %lld]", header.frameId);
        lcm_publish_.publish("CAMERA", &multisense_msg_out_);
    }else{
      // This error will happen for about 50% of frames
      //printf("Left [%lld] and Right [%lld]: Frame Ids dont match", lcm_left_frame_id_, lcm_right_frame_id_);
    }
    
}

void Camera::depthCallback(const image::Header& header,
                           const void*          imageDataP)
{
    if (verbose_){
      printf("depth\n");
    }

    if (Source_Disparity != header.source) {
        printf("Unexpected image source: 0x%x\n", header.source);
        return;
    }
    
    const float    bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint32_t depthSize = header.height * header.width * sizeof(float);
    const uint32_t imageSize = header.width * header.height;

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
          compress2( depth_compress_buf_, &compressed_size, (const Bytef*) imageDataP, uncompressed_size,
                  Z_BEST_SPEED);
          lcm_disp_.data.resize(compressed_size);
          memcpy(&lcm_disp_.data[ 0 ], depth_compress_buf_, compressed_size ); 
          lcm_disp_.size = compressed_size;
          multisense_msg_out_.image_types[1] = multisense_msg_out_.DISPARITY_ZIPPED;
        }else{
          lcm_disp_.data.resize(isize);
          memcpy(&lcm_disp_.data[ 0 ], imageDataP, isize);
          lcm_disp_.size = isize;
          multisense_msg_out_.image_types[1] = multisense_msg_out_.DISPARITY;
        }
        // printf("disp      frame id: %lld", header.frameId);
        lcm_disp_frame_id_ = header.frameId;
        //////////////////////////////////////////////// 

        {
          std::shared_ptr<bot_core::image_t> img;
          img.reset(new bot_core::image_t(lcm_disp_));
          std::unique_lock<std::mutex> lock(queue_mutex_);
          right_img_queue_.push_back(img);
          while (right_img_queue_.size() > max_queue_len_) {
            right_img_queue_.pop_front();
            printf("dropped disp image\n");
          }
          queue_condition_.notify_one();
        }
          
    } else {
        printf("unsupported disparity bpp: %d\n", header.bitsPerPixel);
        return;
    }

}




void Camera::colorImageCallback(const image::Header& header,
                                const void*          imageDataP)
{
    if (verbose_) printf("colorImageCallback\n");

    // The left-luma image is currently published before
    // the matching chroma image. 

    if (false == got_left_luma_) {

        if (Source_Luma_Left == header.source) {
            const uint32_t imageSize = header.width * header.height;
            memcpy(lumaP, imageDataP, imageSize);
            left_luma_frame_id_ = header.frameId;
            got_left_luma_      = true;
        }
        
    } else if (Source_Chroma_Left == header.source) {

        if (header.frameId == left_luma_frame_id_) {

            const uint32_t height    = 544;//left_luma_image_.height;
            const uint32_t width     = 1024;//left_luma_image_.width;
            const uint32_t imageSize = 3 * height * width;

            //
            // Convert YCbCr 4:2:0 to RGB
            // TODO: speed this up

            //const uint8_t *lumaP     = reinterpret_cast<const uint8_t*>(&(left_luma_image_.data[0]));
            const uint8_t *chromaP   = reinterpret_cast<const uint8_t*>(imageDataP);
            //uint8_t       *rgbP      = reinterpret_cast<uint8_t*>(&(left_rgb_image_.data[0]));
            const uint32_t rgbStride = width * 3;

            for(uint32_t y=0; y<height; y++) {
                for(uint32_t x=0; x<width; x++) {

                    const uint32_t lumaOffset   = (y * width) + x;
                    const uint32_t chromaOffset = 2 * (((y/2) * (width/2)) + (x/2));
                    
                    const float px_y  = static_cast<float>(lumaP[lumaOffset]);
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

                    rgbP[rgbOffset + 0] = static_cast<uint8_t>(px_r);
                    rgbP[rgbOffset + 1] = static_cast<uint8_t>(px_g);
                    rgbP[rgbOffset + 2] = static_cast<uint8_t>(px_b);
                }
            }


            if (1==1){//(0 != left_rgb_rect_cam_pub_.getNumSubscribers()) {
                std::unique_lock<std::mutex> lock(cal_lock_);

                if (width  != image_config_.width() ||
                    height != image_config_.height())
                    printf("calibration/image size mismatch: image=%dx%d, calibration=%dx%d\n",
                              width, height, image_config_.width(), image_config_.height());
                else if (NULL == calibration_map_left_1_ || NULL == calibration_map_left_2_)
                    printf("undistort maps not initialized\n");
                else {

                    const CvScalar outlierColor = {{0.0}};

                    IplImage *sourceImageP  = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
                    sourceImageP->imageData = reinterpret_cast<char*>( rgbP );
                    IplImage *destImageP    = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
                    destImageP->imageData   = reinterpret_cast<char*>( rgbP_rect );
                    
                    cvRemap(sourceImageP, destImageP, 
                            calibration_map_left_1_, 
                            calibration_map_left_2_,
                            CV_INTER_LINEAR, outlierColor);
                    
                  
                    ///////////////////////////////////////////////////////////////////////////////
                    // LCM:
                    if (config_.do_jpeg_compress_){
                      std::vector<int> params;
                      params.push_back(cv::IMWRITE_JPEG_QUALITY);
                      params.push_back( config_.jpeg_quality_);
                      
                      cvCvtColor( destImageP, destImageP, CV_BGR2RGB);
                      cv::Mat img = cv::cvarrToMat(destImageP);
                      if (!cv::imencode(".jpg", img, lcm_left_.data, params)) {
                        printf("Error encoding jpeg image\n");
                      }
                      lcm_left_.size = lcm_left_.data.size();
                      lcm_left_.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG; 
                    }else{
                      int left_isize = 3*width*height;
                      lcm_left_.data.resize( left_isize);
                      memcpy(&lcm_left_.data[0], destImageP->imageData, left_isize);
                      // destImageP->imageData   = reinterpret_cast<char*>(&(lcm_left_.data[0]));
                      lcm_left_.size =left_isize;
                      lcm_left_.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB; 
                    }

                    int64_t data_utime = header.timeSeconds*1E6 + header.timeMicroSeconds;
                    lcm_left_.utime = data_utime;
                    lcm_left_.width = width;
                    lcm_left_.height = height;
                    lcm_left_.nmetadata =0;
                    lcm_left_.row_stride=3*width;
                    //printf("left luma frame id: %lld", left_luma_frame_id_);
                    //printf("leftchromaframe id: %lld", header.frameId);
                    lcm_left_frame_id_ = header.frameId;

                    if (config_.output_mode_ ==2){ // publish left only
                      lcm_publish_.publish("CAMERA_LEFT", &lcm_left_);
                      
                    }else if(config_.output_mode_ == 0){
                      std::shared_ptr<bot_core::image_t> img;
                      img.reset(new bot_core::image_t(lcm_left_));
                      std::unique_lock<std::mutex> lock(queue_mutex_);
                      left_img_queue_.push_back(img);
                      while (left_img_queue_.size() > max_queue_len_) {
                        left_img_queue_.pop_front();
                        printf("dropped left image\n");
                      }
                      queue_condition_.notify_one();                      
                      
                      // publish is handled by publisher class
                    }

                    cvReleaseImageHeader(&sourceImageP);
                    cvReleaseImageHeader(&destImageP);
                    
                }
            }
        }
        got_left_luma_ = false;
    }
}


void Camera::queryConfig()
{
    std::unique_lock<std::mutex> lock(cal_lock_);

    // Get the camera config (TODO: clean this up to better handle multiple resolutions)

    if (Status_Ok != driver_->getImageConfig(image_config_)) {
        printf("failed to query sensor configuration\n");
        return;
    }

    // For local rectification of color images
    
    if (calibration_map_left_1_)
        cvReleaseMat(&calibration_map_left_1_);
    if (calibration_map_left_2_)
        cvReleaseMat(&calibration_map_left_2_);

    calibration_map_left_1_ = cvCreateMat(image_config_.height(), image_config_.width(), CV_32F);
    calibration_map_left_2_ = cvCreateMat(image_config_.height(), image_config_.width(), CV_32F);

    // Calibration from sensor is for 2 Mpix, must adjust here (TODO: fix on firmware side, this
    // will get messy when we support arbitrary resolutions.)
    
    image::Calibration cal = image_calibration_;

    if (1024 == image_config_.width()) {
        cal.left.M[0][0] /= 2.0;
        cal.left.M[0][2] /= 2.0;
        cal.left.M[1][1] /= 2.0;
        cal.left.M[1][2] /= 2.0;
        cal.left.P[0][0] /= 2.0;
        cal.left.P[0][2] /= 2.0;
        cal.left.P[0][3] /= 2.0;
        cal.left.P[1][1] /= 2.0;
        cal.left.P[1][2] /= 2.0;
    }

    CvMat M1 = cvMat(3, 3, CV_32F, &cal.left.M);
    CvMat D1 = cvMat(1, 8, CV_32F, &cal.left.D);
    CvMat R1 = cvMat(3, 3, CV_32F, &cal.left.R);
    CvMat P1 = cvMat(3, 4, CV_32F, &cal.left.P);
    
    cvInitUndistortRectifyMap(&M1, &D1, &R1, &P1, calibration_map_left_1_, calibration_map_left_2_);
    
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
