/**
 * @file camera.h
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

#ifndef MULTISENSE_ROS_CAMERA_H
#define MULTISENSE_ROS_CAMERA_H

#include <thread>
#include <memory>
#include <list>
//#include <multisense_ros/state_publisher.h>

#include <MultiSenseChannel.hh>
#include <opencv2/opencv.hpp>

//// LCM:
#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/multisense.hpp>
#include <lcm/lcm-cpp.hpp>
#include <zlib.h>

// example default settings from ROS driver
// fps = 10
// gain 3.0
// auto_exposure true
// auto exposure max time 0.43
// auto exposure decay 7
// auto exposure thres 0.75
// exposure time 0.025
// auto white balance true
// auto whiel balance decay 3
// auto white balance thresh 0.5
// white balance red 1
// white balance blue 1
// lighting and flash off (not relevent)
// led duty cycle 0.25 (not relevent)
// motor_speed 0 (not relevent)


struct CameraConfig{
  
  float spindle_rpm_;
  float fps_;
  bool do_jpeg_compress_;
  bool do_zlib_compress_;
  int jpeg_quality_;
  
  // 0 leftgrey,rightgrey | 1 leftcolor,disp | 2 leftcolor
  int output_mode_;
  
  float gain_;
  int agc_;
  
  CameraConfig () {
        fps_ = 5;
        spindle_rpm_ = 0;
        do_jpeg_compress_ = true;
        do_zlib_compress_ = true;
        jpeg_quality_ = 94;
        output_mode_ = 0;
        gain_ = 3.0;
        agc_ = 1;
  }
};

namespace multisense_ros {

class Camera {
public:
    Camera(crl::multisense::Channel* driver, CameraConfig& config_);
    ~Camera();

    void rectCallback(const crl::multisense::image::Header& header,
                      const void *imageDataP);

    void depthCallback(const crl::multisense::image::Header& header,
                       const void *imageDataP);
    void colorImageCallback(const crl::multisense::image::Header& header,
                            const void *imageDataP);

    void applyConfig(CameraConfig& config);
    
private:

    //
    // Device stream control

    void connectStream(crl::multisense::DataSource enableMask);
    void disconnectStream(crl::multisense::DataSource disableMask);
    void stop();

    //
    // Query sensor status and calibration

    void queryConfig();

    //
    // Update diagnostics

    void updateDiagnostics();

    //
    // CRL sensor API

    crl::multisense::Channel* driver_;

    //
    // Store outgoing messages for efficiency

    uint8_t       *rgbP;
    uint8_t       *lumaP;
    uint8_t       *rgbP_rect; 

    bool                       got_left_luma_;
    int64_t                    left_luma_frame_id_;
    int64_t                    left_rect_frame_id_;

    // Calibration from sensor

    crl::multisense::image::Config      image_config_;
    crl::multisense::image::Calibration image_calibration_;

    //
    // For local rectification of color images
    
    std::mutex cal_lock_;
    CvMat *calibration_map_left_1_;
    CvMat *calibration_map_left_2_;

    //
    // For queues and threads
    struct Publisher;
    std::shared_ptr<Publisher> publisher_;
    int max_queue_len_;
    std::list<std::shared_ptr<bot_core::image_t> > left_img_queue_;
    std::list<std::shared_ptr<bot_core::image_t> > right_img_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_condition_;

    //
    // Stream subscriptions
    
    typedef std::map<crl::multisense::DataSource, int32_t> StreamMapType;
    std::mutex stream_lock_;
    StreamMapType stream_map_;

    //
    // The current capture rate of the sensor

    float capture_fps_;
    
    // LCM stuff:
    lcm::LCM lcm_publish_ ;
    // this message bundles both together:    
    multisense::images_t multisense_msg_out_;
    bot_core::image_t lcm_disp_;
    bot_core::image_t lcm_left_;
    bot_core::image_t lcm_right_;
    int64_t lcm_disp_frame_id_;
    int64_t lcm_left_frame_id_;
    int64_t lcm_right_frame_id_;
    
    int depth_compress_buf_size_;
    uint8_t* depth_compress_buf_;

    bool verbose_;
    CameraConfig& config_;
  
};

}

#endif
