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

#include <multisense_ros/camera.h>
#include <multisense_ros/RawCamConfig.h>

#include <MultiSenseChannel.hh>

#include <opencv2/opencv.hpp>
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

void monoCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->monoCallback(header, imageDataP); }
void rectCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rectCallback(header, imageDataP); }
void depthCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->depthCallback(header, imageDataP); }
void pointCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->pointCloudCallback(header, imageDataP); }
void rawCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rawCamDataCallback(header, imageDataP); }
void colorCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->colorImageCallback(header, imageDataP); }

// added by mfallon:
void rawcolorCB(const image::Header& header, const void* imageDataP, void* userDataP)
{ reinterpret_cast<Camera*>(userDataP)->rawcolorCamDataCallback(header, imageDataP); }

bool isValidPoint(const cv::Vec3f& pt)
{
    //
    // Check both for disparities explicitly marked as invalid (where 
    // OpenCV maps pt.z to MISSING_Z) and zero disparities (point 
    // mapped to infinity).

    return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && std::isfinite(pt[2]);
}

}; // anonymous

Camera::Camera(Channel* driver) :
    driver_(driver),
    device_nh_("multisense_sl"),
    diagnostics_nh_(device_nh_, "diagnostics"),
    left_nh_(device_nh_, "left"),
    right_nh_(device_nh_, "right"),
    left_mono_transport_(left_nh_),
    right_mono_transport_(right_nh_),
    left_rect_transport_(left_nh_),
    right_rect_transport_(right_nh_),
    left_rgb_transport_(left_nh_),
    left_rgb_rect_transport_(left_nh_),
    depth_transport_(device_nh_),
    depth_diagnostics_(ros::NodeHandle(), "multisense_sl/depth_diagnostics", 0.0),
    camera_diagnostics_(ros::NodeHandle(), "multisense_sl/camera_diagnostics", 0.0),
    reconfigure_server_(device_nh_),
    got_raw_cam_left_(false),
    got_left_luma_(false),
    left_luma_frame_id_(0),
    left_rect_frame_id_(0),
    calibration_map_left_1_(NULL),
    calibration_map_left_2_(NULL),
    capture_fps_(0.0f)
{
    device_nh_.param("frame_id", frame_id_, std::string("/left_camera_optical_frame"));

    if(!lcm_publish_.good()){
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }
  
    //
    // Create topic publishers

    left_mono_cam_pub_  = left_mono_transport_.advertise("image_mono", 5, 
                          boost::bind(&Camera::connectStream, this, Source_Luma_Left),
                          boost::bind(&Camera::disconnectStream, this, Source_Luma_Left));
    right_mono_cam_pub_ = right_mono_transport_.advertise("image_mono", 5, 
                          boost::bind(&Camera::connectStream, this, Source_Luma_Right),
                          boost::bind(&Camera::disconnectStream, this, Source_Luma_Right));
    left_rect_cam_pub_  = left_rect_transport_.advertiseCamera("image_rect", 5, 
                          boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left),
                          boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left));
    right_rect_cam_pub_ = right_rect_transport_.advertiseCamera("image_rect", 5, 
                          boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Right),
                          boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Right));
    depth_cam_pub_      = depth_transport_.advertiseCamera("depth", 5, 
                          boost::bind(&Camera::connectStream, this, Source_Disparity),
                          boost::bind(&Camera::disconnectStream, this, Source_Disparity));
    left_rgb_cam_pub_   = left_rgb_transport_.advertise("image_color", 5,
                          boost::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                          boost::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));
    left_rgb_rect_cam_pub_ = left_rgb_rect_transport_.advertiseCamera("image_rect_color", 5,
                          boost::bind(&Camera::connectStream, this, Source_Luma_Left | Source_Chroma_Left),
                          boost::bind(&Camera::disconnectStream, this, Source_Luma_Left | Source_Chroma_Left));
    point_cloud_pub_    = device_nh_.advertise<sensor_msgs::PointCloud2>("points2", 5,
                          boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                          boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

    ros::NodeHandle calibration_nh(device_nh_, "calibration");
    raw_cam_config_pub_ = calibration_nh.advertise<multisense_ros::RawCamConfig>("raw_cam_config", 1, true);
    raw_cam_data_pub_   = calibration_nh.advertise<multisense_ros::RawCamData>("raw_cam_data", 5,
                          boost::bind(&Camera::connectStream, this, Source_Luma_Rectified_Left | Source_Disparity),
                          boost::bind(&Camera::disconnectStream, this, Source_Luma_Rectified_Left | Source_Disparity));

    //
    // All image streams off

    stop();

    //
    // Set up dynamic reconfigure

    reconfigure_server_.setCallback(boost::bind(&Camera::configureCallback, this, _1, _2));

    //
    // Get current sensor configuration

    queryConfig();

    //
    // Add driver-level callbacks.
    //
    //    -Driver creates individual background thread for each callback.
    //    -Images are queued (depth=5) per callback, with oldest silently dropped if not keeping up.
    //    -All images presented are backed by a referenced buffer system (no copying of image data is done.)

    driver_->addIsolatedCallback(monoCB,  Source_Luma_Left | Source_Luma_Right, this);
    driver_->addIsolatedCallback(rectCB,  Source_Luma_Rectified_Left | Source_Luma_Rectified_Right, this);
    driver_->addIsolatedCallback(depthCB, Source_Disparity, this);
    driver_->addIsolatedCallback(pointCB, Source_Disparity, this);
    driver_->addIsolatedCallback(rawCB,   Source_Disparity | Source_Luma_Rectified_Left, this);
    driver_->addIsolatedCallback(colorCB, Source_Luma_Left | Source_Chroma_Left, this);
    
    driver_->addIsolatedCallback(rawcolorCB,   Source_Disparity | Source_Luma_Left | Source_Chroma_Left, this);
    
}

Camera::~Camera()
{
    stop();
}

//
// Note: ROS uses std::vector<> to back images. We copy each
//       and every image below in order to publish it. Is there a way
//       to avoid this copy?
//

void Camera::monoCallback(const image::Header& header,
                          const void*          imageDataP)
{    
    if (Source_Luma_Left  != header.source &&
        Source_Luma_Right != header.source) {
     
        ROS_ERROR("Unexpected image source: 0x%x", header.source);
        return;
    }

    const uint32_t imageSize = header.width * header.height;

    ros::Time t = ros::Time::now();

    switch(header.source) {
    case Source_Luma_Left:

        left_mono_image_.data.resize(imageSize);
        memcpy(&left_mono_image_.data[0], imageDataP, imageSize);

        left_mono_image_.header.frame_id = frame_id_;
        left_mono_image_.header.stamp    = t;
        left_mono_image_.height          = header.height;
        left_mono_image_.width           = header.width;
        
        left_mono_image_.encoding        = "mono8";
        left_mono_image_.is_bigendian    = false;
        left_mono_image_.step            = header.width;

        left_mono_cam_pub_.publish(left_mono_image_);

        camera_diagnostics_.countStream();

        break;
    case Source_Luma_Right:

        right_mono_image_.data.resize(imageSize);
        memcpy(&right_mono_image_.data[0], imageDataP, imageSize);

        right_mono_image_.header.frame_id = frame_id_;
        right_mono_image_.header.stamp    = t;
        right_mono_image_.height          = header.height;
        right_mono_image_.width           = header.width;
        
        right_mono_image_.encoding        = "mono8";
        right_mono_image_.is_bigendian    = false;
        right_mono_image_.step            = header.width;
        
        right_mono_cam_pub_.publish(right_mono_image_);

        camera_diagnostics_.countStream();

        break;
    }
}

void Camera::rectCallback(const image::Header& header,
                          const void*          imageDataP)
{    
    if (Source_Luma_Rectified_Left  != header.source &&
        Source_Luma_Rectified_Right != header.source) {
     
        ROS_ERROR("Unexpected image source: 0x%x", header.source);
        return;
    }

    const uint32_t imageSize = header.width * header.height;

    ros::Time t = ros::Time::now();

    switch(header.source) {
    case Source_Luma_Rectified_Left:

        left_rect_image_.data.resize(imageSize);
        memcpy(&left_rect_image_.data[0], imageDataP, imageSize);

        left_rect_image_.header.frame_id = frame_id_;
        left_rect_image_.header.stamp    = t;
        left_rect_image_.height          = header.height;
        left_rect_image_.width           = header.width;

        left_rect_frame_id_              = header.frameId;
        
        left_rect_image_.encoding        = "mono8";
        left_rect_image_.is_bigendian    = false;
        left_rect_image_.step            = header.width;
        
        left_rect_cam_info_.header = left_rect_image_.header;

        left_rect_cam_pub_.publish(left_rect_image_, left_rect_cam_info_);

        camera_diagnostics_.countStream();

        break;
    case Source_Luma_Rectified_Right:

        right_rect_image_.data.resize(imageSize);
        memcpy(&right_rect_image_.data[0], imageDataP, imageSize);

        right_rect_image_.header.frame_id = frame_id_;
        right_rect_image_.header.stamp    = t;
        right_rect_image_.height          = header.height;
        right_rect_image_.width           = header.width;
        
        right_rect_image_.encoding        = "mono8";
        right_rect_image_.is_bigendian    = false;
        right_rect_image_.step            = header.width;
        
        right_rect_cam_info_.header = right_rect_image_.header;
        
        right_rect_cam_pub_.publish(right_rect_image_, right_rect_cam_info_);

        camera_diagnostics_.countStream();

        break;
    }
}

void Camera::depthCallback(const image::Header& header,
                           const void*          imageDataP)
{
    if (Source_Disparity != header.source) {
     
        ROS_ERROR("Unexpected image source: 0x%x", header.source);
        return;
    }

    depth_diagnostics_.countStream();

    if (0 == depth_cam_pub_.getNumSubscribers())
        return;

    const float    bad_point = std::numeric_limits<float>::quiet_NaN();
    const uint32_t depthSize = header.height * header.width * sizeof(float);
    const uint32_t imageSize = header.width * header.height;

    depth_image_.header.stamp    = ros::Time::now();
    depth_image_.header.frame_id = frame_id_;
    depth_image_.height          = header.height;
    depth_image_.width           = header.width;
    depth_image_.encoding        = "32FC1";
    depth_image_.is_bigendian    = (htonl(1) == 1);
    depth_image_.step            = header.width * 4;
    
    depth_image_.data.resize(depthSize);
    
    float *depthImageP = reinterpret_cast<float*>(&depth_image_.data[0]);

    cv::Mat_<float> depth(header.height, header.width, 
                          reinterpret_cast<float*>(&depth_image_.data[0]));

    //
    // Disparity is in 32-bit floating point

    if (32 == header.bitsPerPixel) {

        cv::Mat_<float> disparity(header.height, header.width, 
                                  const_cast<float*>(reinterpret_cast<const float*>(imageDataP)));
        
        //
        // Depth = focal_length*baseline/disparity

        const double scale = (right_rect_cam_info_.P[3] * right_rect_cam_info_.P[0]);
        cv::divide(scale, disparity, depth);

        //
        // Mark all 0 disparity points as NaNs

        const float *disparityImageP = reinterpret_cast<const float*>(imageDataP);

        for(uint32_t i=0; i<imageSize; ++i)
            if (0.0 == disparityImageP[i])
                depthImageP[i] = bad_point;

    //
    // Disparity is in 1/16th pixel, unsigned integer

    } else if (16 == header.bitsPerPixel) {

        cv::Mat_<uint16_t> disparity(header.height, header.width, 
                                     const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(imageDataP)));
        
        //
        // Depth = focal_length*baseline/disparity

        const float scale = (16.0f * right_rect_cam_info_.P[3] * right_rect_cam_info_.P[0]);
        cv::divide(scale, disparity, depth);

        //
        // Mark all 0 disparity points as NaNs

        const uint16_t *disparityImageP = reinterpret_cast<const uint16_t*>(imageDataP);

        for(uint32_t i=0; i<imageSize; ++i)
            if (0 == disparityImageP[i])
                depthImageP[i] = bad_point;

    } else {
        ROS_ERROR("unsupported disparity bpp: %d", header.bitsPerPixel);
        return;
    }

    depth_cam_pub_.publish(depth_image_, left_rect_cam_info_);
}

void Camera::pointCloudCallback(const image::Header& header,
                                const void*          imageDataP)
{    
    if (Source_Disparity != header.source) {
     
        ROS_ERROR("Unexpected image source: 0x%x", header.source);
        return;
    }

    if (0 == point_cloud_pub_.getNumSubscribers())
        return;

    //
    // If we do not have a matching left-rectified image, bail

    if (left_rect_frame_id_ != header.frameId) {
        //ROS_WARN("Frame mismatch: left-rectified=%lld, disparity=%lld",
        //         left_rect_frame_id_, header.frameId);
        return;
    }

    const ros::Time now            = ros::Time::now();
    const bool      handle_missing = true;
    const uint32_t  imageSize      = header.height * header.width;

    //
    // Resize buffers

    points_buff_.resize(imageSize);
    disparity_buff_.resize(imageSize);

    //
    // Allocate buffer for reprojection output

    cv::Mat_<cv::Vec3f> points(header.height, header.width, 
                               &(points_buff_[0]));

    //
    // Create projection model

    right_rect_cam_info_.header.frame_id = frame_id_;
    left_rect_cam_info_.header.frame_id  = frame_id_;
    model_.fromCameraInfo(left_rect_cam_info_,
                          right_rect_cam_info_);

    //
    // Image is already 32-bit floating point

    if (32 == header.bitsPerPixel) {

        cv::Mat_<float> disparity(header.height, header.width, (float *) imageDataP);
        
        cv::reprojectImageTo3D(disparity, points, model_.reprojectionMatrix(), 
                               handle_missing);
    //
    // Convert CRL 1/16th pixel disparity to floating point

    } else if (16 == header.bitsPerPixel) {

        cv::Mat_<int16_t> disparityOrigP(header.height, header.width, (int16_t *) imageDataP);
        cv::Mat_<float>   disparity(header.height, header.width, &(disparity_buff_[0]));
        disparity = disparityOrigP / 16.0f;

        cv::reprojectImageTo3D(disparity, points, model_.reprojectionMatrix(), 
                               handle_missing);
    } else {
        ROS_ERROR("unsupported disparity bpp: %d", header.bitsPerPixel);
        return;
    }

    //
    // Copy data to PointCloud2 msg

    const uint32_t cloud_step = 13;

    point_cloud_.data.reserve(cloud_step * imageSize);

    if (4 != point_cloud_.fields.size()) {

        point_cloud_.is_bigendian    = (htonl(1) == 1);
        point_cloud_.is_dense        = true;
        point_cloud_.point_step      = cloud_step;
        point_cloud_.height          = 1;
        point_cloud_.header.frame_id = frame_id_;

        point_cloud_.fields.resize(4);
        point_cloud_.fields[0].name     = "x";
        point_cloud_.fields[0].offset   = 0;
        point_cloud_.fields[0].count    = 1;
        point_cloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        point_cloud_.fields[1].name     = "y";
        point_cloud_.fields[1].offset   = 4;
        point_cloud_.fields[1].count    = 1;
        point_cloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        point_cloud_.fields[2].name     = "z";
        point_cloud_.fields[2].offset   = 8;
        point_cloud_.fields[2].count    = 1;
        point_cloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        point_cloud_.fields[3].name     = "lumanince";
        point_cloud_.fields[3].offset   = 12;
        point_cloud_.fields[3].count    = 1;
        point_cloud_.fields[3].datatype = sensor_msgs::PointField::UINT8;
    }

    //
    // Pack the data into the point cloud structure

    uint8_t       *cloudP    = reinterpret_cast<uint8_t*>(&point_cloud_.data[0]);
    const uint8_t *grayP     = &(left_rect_image_.data[0]);
    const uint32_t pointSize = 3 * sizeof(float); // x, y, z

    uint32_t validPoints = 0;

    for(uint32_t i=0; i<imageSize; ++i)
        if (isValidPoint(points_buff_[i])) {

            memcpy(cloudP, &(points_buff_[i]), pointSize);
            *(cloudP + pointSize) = grayP[i];

            cloudP += cloud_step;
            validPoints ++;
        }
   
    point_cloud_.row_step        = validPoints;
    point_cloud_.width           = validPoints;
    point_cloud_.header.stamp    = now;

    point_cloud_.data.resize(validPoints * cloud_step);
    point_cloud_pub_.publish(point_cloud_);
}

void Camera::rawCamDataCallback(const image::Header& header,
                                const void*          imageDataP)
{
    if (0 == raw_cam_data_pub_.getNumSubscribers()) {
        got_raw_cam_left_ = false;
        return;
    }

    const uint32_t imageSize = header.width * header.height;

    //
    // The left-rectified image is currently published before
    // the matching disparity image. 

    if (false == got_raw_cam_left_) {

        if (Source_Luma_Rectified_Left == header.source) {

            raw_cam_data_.gray_scale_image.resize(imageSize);
            memcpy(&(raw_cam_data_.gray_scale_image[0]), 
                   imageDataP,
                   imageSize * sizeof(uint8_t));

            raw_cam_data_.frames_per_second = header.framesPerSecond;
            raw_cam_data_.gain              = header.gain;
            raw_cam_data_.exposure_time     = header.exposure;
            raw_cam_data_.frame_count       = header.frameId;
            raw_cam_data_.time_stamp        = ros::Time(header.timeSeconds,
                                                        1000 * header.timeMicroSeconds);
            raw_cam_data_.width             = header.width;
            raw_cam_data_.height            = header.height;
            
            got_raw_cam_left_ = true;
        }
        
    } else if (Source_Disparity == header.source) {

        const uint32_t imageSize = header.width * header.height * sizeof(uint16_t);

        if (header.frameId == raw_cam_data_.frame_count) {

            raw_cam_data_.disparity_image.resize(imageSize);
            memcpy(&(raw_cam_data_.disparity_image[0]), 
                   imageDataP, imageSize * sizeof(uint16_t));

            raw_cam_data_pub_.publish(raw_cam_data_);
        }

        got_raw_cam_left_ = false;
    }
}

void Camera::colorImageCallback(const image::Header& header,
                                const void*          imageDataP)
{
    if (0 == left_rgb_cam_pub_.getNumSubscribers() &&
        0 == left_rgb_rect_cam_pub_.getNumSubscribers()) {
        got_left_luma_ = false;
        return;
    }

    //
    // Just count the chroma.. the luma image is counted in monoCallback

    if (Source_Chroma_Left == header.source)
        camera_diagnostics_.countStream();

    //
    // The left-luma image is currently published before
    // the matching chroma image. 

    if (false == got_left_luma_) {

        if (Source_Luma_Left == header.source) {

            const uint32_t imageSize = header.width * header.height;

            left_luma_image_.data.resize(imageSize);
            memcpy(&left_luma_image_.data[0], imageDataP, imageSize);

            left_luma_image_.height = header.height;
            left_luma_image_.width  = header.width;

            left_luma_frame_id_ = header.frameId;
            got_left_luma_      = true;
        }
        
    } else if (Source_Chroma_Left == header.source) {

        if (header.frameId == left_luma_frame_id_) {

            const uint32_t height    = left_luma_image_.height;
            const uint32_t width     = left_luma_image_.width;
            const uint32_t imageSize = 3 * height * width;
            
            left_rgb_image_.data.resize(imageSize);

            left_rgb_image_.header.frame_id = frame_id_;
            left_rgb_image_.header.stamp    = ros::Time::now();
            left_rgb_image_.height          = height;
            left_rgb_image_.width           = width;
             
            left_rgb_image_.encoding        = "rgb8";
            left_rgb_image_.is_bigendian    = false;
            left_rgb_image_.step            = width;

            //
            // Convert YCbCr 4:2:0 to RGB
            // TODO: speed this up

            const uint8_t *lumaP     = reinterpret_cast<const uint8_t*>(&(left_luma_image_.data[0]));
            const uint8_t *chromaP   = reinterpret_cast<const uint8_t*>(imageDataP);
            uint8_t       *rgbP      = reinterpret_cast<uint8_t*>(&(left_rgb_image_.data[0]));
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

            if (0 != left_rgb_cam_pub_.getNumSubscribers())
                left_rgb_cam_pub_.publish(left_rgb_image_);

            if (0 != left_rgb_rect_cam_pub_.getNumSubscribers()) {
                boost::mutex::scoped_lock lock(cal_lock_);

                if (width  != image_config_.width() ||
                    height != image_config_.height())
                    ROS_ERROR("calibration/image size mismatch: image=%dx%d, calibration=%dx%d",
                              width, height, image_config_.width(), image_config_.height());
                else if (NULL == calibration_map_left_1_ || NULL == calibration_map_left_2_)
                    ROS_ERROR("undistort maps not initialized");
                else {

                    const CvScalar outlierColor = {{0.0}};

                    left_rgb_rect_image_.data.resize(imageSize);

                    IplImage *sourceImageP  = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
                    sourceImageP->imageData = reinterpret_cast<char*>(&(left_rgb_image_.data[0]));
                    IplImage *destImageP    = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
                    destImageP->imageData   = reinterpret_cast<char*>(&(left_rgb_rect_image_.data[0]));
                    
                    // LCM 1:
                    int n_colors=3;
                    int left_isize = n_colors*width*height;
                    left_msg_out_.data.resize( left_isize);
                    destImageP->imageData   = reinterpret_cast<char*>(&(left_msg_out_.data[0]));
                    ////////////////////////////////////////////////
                    

                    cvRemap(sourceImageP, destImageP, 
                            calibration_map_left_1_, 
                            calibration_map_left_2_,
                            CV_INTER_LINEAR, outlierColor);

                    cvReleaseImageHeader(&sourceImageP);
                    cvReleaseImageHeader(&destImageP);

                    left_rgb_rect_image_.header.frame_id = frame_id_;
                    ros::Time cam_time = ros::Time::now();
                    left_rgb_rect_image_.header.stamp    =cam_time;
                    left_rgb_rect_image_.height          = height;
                    left_rgb_rect_image_.width           = width;
                    
                    left_rgb_rect_image_.encoding        = "rgb8";
                    left_rgb_rect_image_.is_bigendian    = false;
                    left_rgb_rect_image_.step            = width;
                    left_rgb_rect_cam_info_.header = left_rgb_rect_image_.header;
                    left_rgb_rect_cam_pub_.publish(left_rgb_rect_image_, left_rgb_rect_cam_info_);
                    
                    // LCM 2:
                    left_msg_out_.utime = (int64_t) floor(cam_time.toNSec()/1000);
                    left_msg_out_.width = width;
                    left_msg_out_.height = height;
                    left_msg_out_.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB; //bot_core::image_t::PIXEL_FORMAT_GRAY;
                    left_msg_out_.nmetadata =0;
                    left_msg_out_.row_stride=n_colors*width;
                    left_msg_out_.size =left_isize;
                    //memcpy(&left_msg_out_.data[0], destImageP->imageData, left_isize); // didn't work in newer version
                    //ROS_ERROR("postcam");
                    lcm_publish_.publish("CAMERALEFT", &left_msg_out_);                    
                    //ROS_ERROR("postpub");
                    ////////////////////////////////////////////////
                    
                    
                    
                }
            }
        }

        got_left_luma_ = false;
    }
}



//void Camera::colorImageCallback(const image::Header& header,
void Camera::rawcolorCamDataCallback(const image::Header& header,
                                const void*          imageDataP)
{
    if (0 == left_rgb_cam_pub_.getNumSubscribers() &&
        0 == left_rgb_rect_cam_pub_.getNumSubscribers()) {
        got_left_luma_ = false;
        return;
    }

    //
    // Just count the chroma.. the luma image is counted in monoCallback

    if (Source_Chroma_Left == header.source)
        camera_diagnostics_.countStream();

    //
    // The left-luma image is currently published before
    // the matching chroma image. 

    if (false == got_left_luma_) {

        if (Source_Luma_Left == header.source) {

            const uint32_t imageSize = header.width * header.height;

            left_luma_image_.data.resize(imageSize);
            memcpy(&left_luma_image_.data[0], imageDataP, imageSize);

            left_luma_image_.height = header.height;
            left_luma_image_.width  = header.width;

            left_luma_frame_id_ = header.frameId;
            got_left_luma_      = true;
        }
        
    } else if (Source_Chroma_Left == header.source) {

        if (header.frameId == left_luma_frame_id_) {

            const uint32_t height    = left_luma_image_.height;
            const uint32_t width     = left_luma_image_.width;
            const uint32_t imageSize = 3 * height * width;
            
            left_rgb_image_.data.resize(imageSize);

            left_rgb_image_.header.frame_id = frame_id_;
            left_rgb_image_.header.stamp    = ros::Time::now();
            left_rgb_image_.height          = height;
            left_rgb_image_.width           = width;
             
            left_rgb_image_.encoding        = "rgb8";
            left_rgb_image_.is_bigendian    = false;
            left_rgb_image_.step            = width;

            //
            // Convert YCbCr 4:2:0 to RGB
            // TODO: speed this up

            const uint8_t *lumaP     = reinterpret_cast<const uint8_t*>(&(left_luma_image_.data[0]));
            const uint8_t *chromaP   = reinterpret_cast<const uint8_t*>(imageDataP);
            uint8_t       *rgbP      = reinterpret_cast<uint8_t*>(&(left_rgb_image_.data[0]));
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

            if (0 != left_rgb_cam_pub_.getNumSubscribers())
                left_rgb_cam_pub_.publish(left_rgb_image_);

            if (0 != left_rgb_rect_cam_pub_.getNumSubscribers()) {
                boost::mutex::scoped_lock lock(cal_lock_);

                if (width  != image_config_.width() ||
                    height != image_config_.height())
                    ROS_ERROR("calibration/image size mismatch: image=%dx%d, calibration=%dx%d",
                              width, height, image_config_.width(), image_config_.height());
                else if (NULL == calibration_map_left_1_ || NULL == calibration_map_left_2_)
                    ROS_ERROR("undistort maps not initialized");
                else {

                    const CvScalar outlierColor = {{0.0}};

                    left_rgb_rect_image_.data.resize(imageSize);

                    IplImage *sourceImageP  = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
                    sourceImageP->imageData = reinterpret_cast<char*>(&(left_rgb_image_.data[0]));
                    IplImage *destImageP    = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
                    destImageP->imageData   = reinterpret_cast<char*>(&(left_rgb_rect_image_.data[0]));
                    
                    // LCM 1:
                    int n_colors=3;
                    int left_isize = n_colors*width*height;
                    left_msg_out_.data.resize( left_isize);
                    destImageP->imageData   = reinterpret_cast<char*>(&(left_msg_out_.data[0]));
                    ////////////////////////////////////////////////
                    

                    cvRemap(sourceImageP, destImageP, 
                            calibration_map_left_1_, 
                            calibration_map_left_2_,
                            CV_INTER_LINEAR, outlierColor);

                    cvReleaseImageHeader(&sourceImageP);
                    cvReleaseImageHeader(&destImageP);

                    left_rgb_rect_image_.header.frame_id = frame_id_;
                    ros::Time cam_time = ros::Time::now();
                    left_rgb_rect_image_.header.stamp    =cam_time;
                    left_rgb_rect_image_.height          = height;
                    left_rgb_rect_image_.width           = width;
                    
                    left_rgb_rect_image_.encoding        = "rgb8";
                    left_rgb_rect_image_.is_bigendian    = false;
                    left_rgb_rect_image_.step            = width;
                    left_rgb_rect_cam_info_.header = left_rgb_rect_image_.header;
                    left_rgb_rect_cam_pub_.publish(left_rgb_rect_image_, left_rgb_rect_cam_info_);
                    
                    // LCM 2:
                    left_msg_out_.utime = (int64_t) floor(cam_time.toNSec()/1000);
                    left_msg_out_.width = width;
                    left_msg_out_.height = height;
                    left_msg_out_.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB; //bot_core::image_t::PIXEL_FORMAT_GRAY;
                    left_msg_out_.nmetadata =0;
                    left_msg_out_.row_stride=n_colors*width;
                    left_msg_out_.size =left_isize;
                    //memcpy(&left_msg_out_.data[0], destImageP->imageData, left_isize); // didn't work in newer version
                    //ROS_ERROR("postcam");
                    lcm_publish_.publish("CAMERALEFT", &left_msg_out_);                    
                    //ROS_ERROR("postpub");
                    ////////////////////////////////////////////////
                    
                }
            }
        }

        got_left_luma_ = false;
    }
}

void Camera::queryConfig()
{
    boost::mutex::scoped_lock lock(cal_lock_);

    camera_diagnostics_.publish(SensorStatus::STARTING);
    depth_diagnostics_.publish(SensorStatus::STARTING);

    //
    // Get the camera config (TODO: clean this up to better handle multiple resolutions)

    if (Status_Ok != driver_->getImageConfig(image_config_) ||
        Status_Ok != driver_->getImageCalibration(image_calibration_)) {
        ROS_ERROR("failed to query sensor configuration");
        return;
    }

    //
    // Cache internally

    left_rect_cam_info_.width  = image_config_.width();
    left_rect_cam_info_.height = image_config_.height();

    left_rect_cam_info_.P[0]   = image_config_.fx();       left_rect_cam_info_.P[1]   = 0.0;   
    left_rect_cam_info_.P[4]   = 0.0;                      left_rect_cam_info_.P[5]   = image_config_.fy();  
    left_rect_cam_info_.P[8]   = 0.0;                      left_rect_cam_info_.P[9]   = 0.0;   
    left_rect_cam_info_.P[2]   = image_config_.width()/2;  left_rect_cam_info_.P[3]   = 0.0;
    left_rect_cam_info_.P[6]   = image_config_.height()/2; left_rect_cam_info_.P[7]   = 0.0;
    left_rect_cam_info_.P[10]  = 1.0;                      left_rect_cam_info_.P[11]  = 0.0;
    
    right_rect_cam_info_.width  = image_config_.width();
    right_rect_cam_info_.height = image_config_.height();
    
    right_rect_cam_info_.P[0]  = image_config_.fx();       right_rect_cam_info_.P[1]  = 0.0;  
    right_rect_cam_info_.P[4]  = 0.0;                      right_rect_cam_info_.P[5]  = image_config_.fy();  
    right_rect_cam_info_.P[8]  = 0.0;                      right_rect_cam_info_.P[9]  = 0.0;  
    right_rect_cam_info_.P[2]  = image_config_.width()/2;  right_rect_cam_info_.P[3]  = image_config_.tx() * image_config_.fx();
    right_rect_cam_info_.P[6]  = image_config_.height()/2; right_rect_cam_info_.P[7]  = 0.0;
    right_rect_cam_info_.P[10] = 1.0;                      right_rect_cam_info_.P[11] = 0.0;
    
    left_rgb_rect_cam_info_ = left_rect_cam_info_;

    //
    // For local rectification of color images
    
    if (calibration_map_left_1_)
        cvReleaseMat(&calibration_map_left_1_);
    if (calibration_map_left_2_)
        cvReleaseMat(&calibration_map_left_2_);

    calibration_map_left_1_ = cvCreateMat(image_config_.height(), image_config_.width(), CV_32F);
    calibration_map_left_2_ = cvCreateMat(image_config_.height(), image_config_.width(), CV_32F);

    //
    // Calibration from sensor is for 2 Mpix, must adjust here (TODO: fix on firmware side, this
    // will get messy when we support arbitrary resolutions.)
    
    if (1024 == image_config_.width()) {
        image_calibration_.left.M[0][0] /= 2.0;
        image_calibration_.left.M[0][2] /= 2.0;
        image_calibration_.left.M[1][1] /= 2.0;
        image_calibration_.left.M[1][2] /= 2.0;
        image_calibration_.left.P[0][0] /= 2.0;
        image_calibration_.left.P[0][2] /= 2.0;
        image_calibration_.left.P[0][3] /= 2.0;
        image_calibration_.left.P[1][1] /= 2.0;
        image_calibration_.left.P[1][2] /= 2.0;
    }

    CvMat M1 = cvMat(3, 3, CV_32F, &image_calibration_.left.M);
    CvMat D1 = cvMat(1, 8, CV_32F, &image_calibration_.left.D);
    CvMat R1 = cvMat(3, 3, CV_32F, &image_calibration_.left.R);
    CvMat P1 = cvMat(3, 4, CV_32F, &image_calibration_.left.P);
    
    cvInitUndistortRectifyMap(&M1, &D1, &R1, &P1, calibration_map_left_1_, calibration_map_left_2_);
    
    //
    // Publish the "raw" message
    
    multisense_ros::RawCamConfig ros_msg;
    
    ros_msg.width             = image_config_.width();
    ros_msg.height            = image_config_.height();
    ros_msg.frames_per_second = image_config_.fps();
    ros_msg.gain              = image_config_.gain();
    ros_msg.exposure_time     = image_config_.exposure();
    
    ros_msg.fx    = image_config_.fx();
    ros_msg.fy    = image_config_.fy();
    ros_msg.cx    = image_config_.cx();
    ros_msg.cy    = image_config_.cy();
    ros_msg.tx    = image_config_.tx();
    ros_msg.ty    = image_config_.ty();
    ros_msg.tz    = image_config_.tx();
    ros_msg.roll  = image_config_.roll();
    ros_msg.pitch = image_config_.pitch();
    ros_msg.yaw   = image_config_.yaw();

    raw_cam_config_pub_.publish(ros_msg);

    camera_diagnostics_.publish(SensorStatus::RUNNING);
    depth_diagnostics_.publish(SensorStatus::RUNNING);
}

void Camera::stop()
{
    boost::mutex::scoped_lock lock(stream_lock_);

    camera_diagnostics_.publish(SensorStatus::STOPPING);
    depth_diagnostics_.publish(SensorStatus::STOPPING);

    stream_map_.clear();

    Status status = driver_->stopStreams(allImageSources);
    if (Status_Ok != status)
        ROS_ERROR("Failed to stop all streams: Error code %d",
                  status);
    else {
        camera_diagnostics_.publish(SensorStatus::STOPPED);
        depth_diagnostics_.publish(SensorStatus::STOPPED);
    }
}

void Camera::updateDiagnostics()
{
    DataSource streamsEnabled = 0;

    if (Status_Ok != driver_->getEnabledStreams(streamsEnabled)) {
        ROS_ERROR("failed to query enabled streams");
        return;
    }

    streamsEnabled &= allImageSources;

    if (Source_Disparity & streamsEnabled) {
        depth_diagnostics_.setExpectedRate(capture_fps_);
        streamsEnabled &= ~Source_Disparity;
    } else
        depth_diagnostics_.setExpectedRate(0.0);

    float aggregateImageRate = 0;

    for(uint32_t i=0; i<32; i++) 
        if ((1<<i) & streamsEnabled)
            aggregateImageRate += capture_fps_;
    
    camera_diagnostics_.setExpectedRate(aggregateImageRate);
}

void Camera::connectStream(DataSource enableMask)
{    
    boost::mutex::scoped_lock lock(stream_lock_);

    DataSource notStarted = 0;

    for(uint32_t i=0; i<32; i++) 
        if ((1<<i) & enableMask && 0 == stream_map_[(1<<i)]++)
            notStarted |= (1<<i);

    if (0 != notStarted) {

        Status status = driver_->startStreams(notStarted);
        if (Status_Ok != status)
            ROS_ERROR("Failed to start streams 0x%x. Error code %d", 
                      notStarted, status);
    }

    updateDiagnostics();
}

void Camera::disconnectStream(DataSource disableMask)
{
    boost::mutex::scoped_lock lock(stream_lock_);

    DataSource notStopped = 0;

    for(uint32_t i=0; i<32; i++) 
        if ((1<<i) & disableMask && 0 == --stream_map_[(1<<i)])
            notStopped |= (1<<i);

    if (0 != notStopped) {
        Status status = driver_->stopStreams(notStopped);
        if (Status_Ok != status)
            ROS_ERROR("Failed to stop streams 0x%x. Error code %d\n", 
                      notStopped, status);
    }

    updateDiagnostics();
}

//
// The dynamic reconfigure callback
// TODO: the lighting and motor-speed controls are lumped in here, move this 
//       entire interface into its own module.

void Camera::configureCallback(multisense_ros::CameraConfig & config, uint32_t level)
{
    boost::mutex::scoped_lock lock(stream_lock_);

    image::Config cfg;

    DataSource streamsEnabled = 0;
    uint32_t   width, height;

    bool resolutionChanged=false;

    //
    // Query the current configuration of the sensor

    Status status = driver_->getImageConfig(cfg);
    if (Status_Ok != status) {
        ROS_ERROR("Failed to query image config. Error code %d", status);
        return;
    }

    //
    // Decode the resolution string

    if (2 != sscanf(config.resolution.c_str(), "%dx%d", &width, &height))
        ROS_ERROR("Malformed resolution string: \"%s\"", config.resolution.c_str());

    //
    // If a resolution change is desired
    
    else if (width != cfg.width() || height != cfg.height()) {
        
        //
        // Query all supported resolutions from the sensor
        
        std::vector<system::DeviceMode> modes;
        if (Status_Ok != driver_->getDeviceModes(modes))
            ROS_ERROR("Failed to query sensor modes");
        else {

            //
            // Verify that this resolution is supported

            bool supported = false;
            std::vector<system::DeviceMode>::const_iterator it = modes.begin();
            for(; it != modes.end(); ++it)
                if (width == (*it).width && height == (*it).height) {
                    supported = true;
                    break;
                }
            
            if (false == supported)
                ROS_ERROR("Sensor does not support a resolution of: %dx%d", width, height);
            else {
                
                //
                // Halt streams during the resolution change
                
                if (Status_Ok != driver_->getEnabledStreams(streamsEnabled) ||
                    Status_Ok != driver_->stopStreams(streamsEnabled & allImageSources))
                    ROS_ERROR("Failed to stop streams for a resolution change");
                else {

                    ROS_WARN("Changing sensor resolution to %dx%d (from %dx%d): reconfiguration may take up to 30 seconds",
                             width, height, cfg.width(), cfg.height());

                    cfg.setResolution(width, height);
                    resolutionChanged = true;
                }
            }
        }
    }

    //
    // Set all image config from dynamic reconfigure

    cfg.setFps(config.fps);
    cfg.setGain(config.gain);
    cfg.setExposure(config.exposure_time * 1e6);    
    cfg.setAutoExposure(config.auto_exposure);
    cfg.setAutoExposureMax(config.auto_exposure_max_time * 1e6);
    cfg.setAutoExposureDecay(config.auto_exposure_decay);
    cfg.setAutoExposureThresh(config.auto_exposure_thresh);
    cfg.setWhiteBalance(config.white_balance_red,
                        config.white_balance_blue);
    cfg.setAutoWhiteBalance(config.auto_white_balance);
    cfg.setAutoWhiteBalanceDecay(config.auto_white_balance_decay);
    cfg.setAutoWhiteBalanceThresh(config.auto_white_balance_thresh);

    //
    // Apply, sensor enforces limits per setting.

    status = driver_->setImageConfig(cfg);
    if (Status_Ok != status)
        ROS_ERROR("Failed to set image config. Error code %d", status);
    else {
        
        //
        // Store the desired framerate, and update expected streaming
        // rates via diagnostics.

        capture_fps_ = config.fps;
        updateDiagnostics();
    }

    //
    // If we are changing the resolution, we need to re-query calibration and
    // restart all subscribed streams

    if (resolutionChanged) {
        queryConfig();
        if (Status_Ok != driver_->startStreams(streamsEnabled & allImageSources))
            ROS_ERROR("Failed to restart streams after a resolution change");
    }

    //
    // Send the desired lighting configuration

    lighting::Config leds;

    if (false == config.lighting) {
        leds.setFlash(false);
        leds.setDutyCycle(0.0);
    } else {
        leds.setFlash(config.flash);
        leds.setDutyCycle(config.duty_cycle * 100.0);
    }

    status = driver_->setLightingConfig(leds);
    if (Status_Ok != status)
        ROS_ERROR("Failed to set lighting config. Error code %d", status);

    //
    // Send the desired motor speed

    const float radiansPerSecondToRpm = 9.54929659643;

    status = driver_->setMotorSpeed(radiansPerSecondToRpm * config.motor_speed);
    if (Status_Ok != status)
        ROS_ERROR("Failed to set motor speed. Error code %d", status);
}

} // namespace
