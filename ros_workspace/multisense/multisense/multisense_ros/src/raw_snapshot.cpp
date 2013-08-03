/**
 * @file raw_snapshot.cpp
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
#include <ros/ros.h>
#include <ros/topic.h>
#include <rosbag/bag.h>
#include <multisense_ros/DeviceInfo.h>
#include <multisense_ros/RawCamConfig.h>
#include <multisense_ros/RawCamCal.h>
#include <multisense_ros/RawCamData.h>
#include <multisense_ros/RawLidarData.h>
#include <multisense_ros/RawLidarCal.h>
#include <stdio.h>
#include <ros/callback_queue.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace { // anonymous

#define TOPIC_DEVICE_INFO     "/multisense_sl/calibration/device_info"
#define TOPIC_RAW_CAM_CAL     "/multisense_sl/calibration/raw_cam_cal"
#define TOPIC_RAW_CAM_CONFIG  "/multisense_sl/calibration/raw_cam_config"
#define TOPIC_RAW_CAM_DATA    "/multisense_sl/calibration/raw_cam_data"
#define TOPIC_RAW_LIDAR       "/laser/calibration/raw_lidar_data"
#define TOPIC_RAW_LIDAR_CAL   "/laser/calibration/raw_lidar_cal"

//
// Helper class for getting a full rotation of laser scan data

class LaserHelper {
public:

    LaserHelper() : motion_started_(false) {}

    void callback(const multisense_ros::RawLidarData::ConstPtr& msg)
    {
        //
        // Keep around one scan until the head starts spinning, then
        // start collecting

        if (motion_started_ || 0 == scans_.size())
            scans_.push_back(msg);
      
        if (!motion_started_) {
            std::list<multisense_ros::RawLidarData::ConstPtr>::const_iterator it = scans_.begin();

            if (msg->angle_start != (*it)->angle_start) {
                motion_started_ = true;
                scans_.clear();
                scans_.push_back(msg);
            }
        }
    }

    //
    // Check if we have received an entire rotation of laser data
    // return True if we have a full rotation of data data

    bool done() const
    {
        std::list<multisense_ros::RawLidarData::ConstPtr>::const_iterator it;

        //
        // If we have no scans, then we're definitely not done

        it = scans_.begin();
        if (scans_.end() == it)
            return false;

        //
        // Calculate the total angle swept through all the scans, accounting for wrap-around

        double previous_start = (*it)->angle_start / 1000000.0;
        double total_swept    = 0.0;

        for (it = scans_.begin(); it != scans_.end(); it++) {

            double current_start = (*it)->angle_start / 1000000.0;
            double current_swept = current_start - previous_start;

            //
            // Unwrap the swept angle

            while(current_swept > M_PI)
                current_swept -= 2*M_PI;
            while(current_swept < -M_PI)
                current_swept += 2*M_PI;

            //
            // Accumulate

            total_swept += current_swept;

            //
            // Save the previous start angle
            
            previous_start = current_start;
        }

        return ( fabs(total_swept) > 2.0 * M_PI);
    }

    //
    // Gets an entire rotation of laser data.  This method constructs it own subscriber
    // and callback queue, and services them itself.
    // It will return list of the captured messages for this single rotation

    bool getRotation(std::list<multisense_ros::RawLidarData::ConstPtr>& scan_collection)
    {
        scans_.clear();
        ros::NodeHandle nh;
        nh.setCallbackQueue(&queue_);

        ros::Subscriber sub_ = nh.subscribe<multisense_ros::RawLidarData>(TOPIC_RAW_LIDAR, 5,
                                            boost::bind(&LaserHelper::callback, this, _1));

        ros::Time start = ros::Time::now();

        //
        // Service our own callback queue
        
        while(1) {

            queue_.callOne(ros::WallDuration(1.0));

            if (done()) {
                scan_collection = scans_;
                return true;
            }

            if ((ros::Time::now().toSec() - start.toSec()) >= SCAN_COLLECT_TIMEOUT)
                return false;
        }
    }

private:

    static const double SCAN_COLLECT_TIMEOUT = 20.0; // seconds

    bool motion_started_;
    ros::CallbackQueue queue_;
    std::list<multisense_ros::RawLidarData::ConstPtr> scans_;
};

void setConf(const dynamic_reconfigure::Config& conf)
{
    dynamic_reconfigure::ReconfigureRequest  srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;

    srv_req.config = conf;

    ros::service::call("/multisense_sl/set_parameters", srv_req, srv_resp);
}

void setMotorSpeed(double radPerSec)
{
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config          conf;

    double_param.name  = "motor_speed";
    double_param.value = radPerSec;
    conf.doubles.push_back(double_param);

    setConf(conf);
}

void setResolution(const std::string& res)
{
    dynamic_reconfigure::StrParameter str_param;
    dynamic_reconfigure::Config       conf;

    str_param.name  = "resolution";
    str_param.value = res;
    conf.strs.push_back(str_param);
    
    setConf(conf);
}

}; // anonymous

int main(int    argc, 
         char** argvPP)
{
    ros::init(argc, argvPP, "raw_snapshot");
    ros::NodeHandle nh;

    if (argc != 2 || 
        std::string(argvPP[1]) == "--help" ||
        std::string(argvPP[1]) == "-h") {

        printf("Usage: %s <out_bag_filename>\n", argvPP[0]);
        return -1;
    }

    std::string outfile(argvPP[1]);

    setResolution("1024x544");
    ros::Duration(1.0).sleep();

    printf("Capturing device information... "); fflush(stdout);
    multisense_ros::DeviceInfo::ConstPtr device_info = ros::topic::waitForMessage<multisense_ros::DeviceInfo>(TOPIC_DEVICE_INFO, nh);
    if (!device_info) {
        printf("  Error capturing DeviceInfo. Exiting\n");
        return -1;
    }
    printf("  Success\n");

    printf("Capturing lidar calibration... ");
    multisense_ros::RawLidarCal::ConstPtr lidar_cal = ros::topic::waitForMessage<multisense_ros::RawLidarCal>(TOPIC_RAW_LIDAR_CAL, nh);
    if (!lidar_cal) {
        printf("  Error capturing RawLidarCal. Exiting\n");
        return -1;
    }
    printf("  Success\n");

    printf("Capturing camera calibration... "); fflush(stdout);
    multisense_ros::RawCamCal::ConstPtr cam_cal = ros::topic::waitForMessage<multisense_ros::RawCamCal>(TOPIC_RAW_CAM_CAL, nh);
    if (!cam_cal) {
        printf("  Error capturing RawCamCal. Exiting\n");
        return -1;
    }
    printf("  Success\n");

    printf("Capturing camera config... "); fflush(stdout);
    multisense_ros::RawCamConfig::ConstPtr cam_config = ros::topic::waitForMessage<multisense_ros::RawCamConfig>(TOPIC_RAW_CAM_CONFIG, nh);
    if (!cam_config) {
        printf("  Error capturing RawCamConfig. Exiting\n");
        return -1;
    }
    printf("  Success\n");

    printf("Capturing a single left-rectified/disparity image pair... "); fflush(stdout);
    multisense_ros::RawCamData::ConstPtr cam_data = ros::topic::waitForMessage<multisense_ros::RawCamData>(TOPIC_RAW_CAM_DATA, nh);
    if (!cam_data) {
        printf("  Error capturing RawCamData. Exiting\n");
        return -1;
    }
    printf("  Success\n");

    printf("Capturing a full rotation of lidar scans... "); fflush(stdout);

    setMotorSpeed(0.785);

    LaserHelper laser_helper;
    std::list<multisense_ros::RawLidarData::ConstPtr> raw_lidar_data;
    
    if (false == laser_helper.getRotation(raw_lidar_data)) {
        printf("  Error capturing RawLidarData...\n");
        return -1;
    }
    printf("  Captured %zu lidar scans\n", raw_lidar_data.size());

    setMotorSpeed(0.0);

    //
    // Save all of the data to a bag file

    printf("Saving data to file [%s]\n", outfile.c_str());
    rosbag::Bag bag;
    bag.open(outfile, rosbag::bagmode::Write);
    
    bag.write(TOPIC_DEVICE_INFO, ros::TIME_MIN, *device_info);
    bag.write(TOPIC_RAW_LIDAR_CAL, ros::TIME_MIN, *lidar_cal);
    bag.write(TOPIC_RAW_CAM_CAL, ros::TIME_MIN, *cam_cal);
    bag.write(TOPIC_RAW_CAM_CONFIG, ros::TIME_MIN, *cam_config);
    bag.write(TOPIC_RAW_CAM_DATA, ros::TIME_MIN, *cam_data);

    std::list<multisense_ros::RawLidarData::ConstPtr>::const_iterator it;
    for (it = raw_lidar_data.begin(); it != raw_lidar_data.end(); it++)
        bag.write(TOPIC_RAW_LIDAR, ros::TIME_MIN, **it);

    bag.close();
    printf("  Success\n");

    return 0;
}

