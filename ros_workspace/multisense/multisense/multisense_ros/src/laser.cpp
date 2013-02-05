#include <multisense_ros/laser.h>

#include <LibSensorPodCommunications/LidarStartScanMessage.h>
#include <LibSensorPodCommunications/LidarStopScanMessage.h>



static const float HOKUYO_SCAN_ANGLE_RADIANS = 4.71238898038469;
static const float RANGE_METERS_MIN = 0;
static const float RANGE_METERS_MAX = 100;


namespace multisense_ros
{

  const float Laser::EXPECTED_RATE = 40.0;

  Laser::Laser(multisense_driver::MultisenseDriver* driver)
    : driver_(driver),
      scan_caller_(1,20),
      diagnostics_(ros::NodeHandle(), "laser/diagnostics", EXPECTED_RATE)
  {
    ros::NodeHandle nh("laser");

    // get frame_id
    nh.param("frame_id", frame_id_, std::string("/hokuyo_link"));

    // create publishers
    scan_pub_  = nh.advertise<sensor_msgs::LaserScan>("scan", 20,
                                                      boost::bind(&Laser::connectScanCB, this),
                                                      boost::bind(&Laser::disconnectScanCB, this));

    // subscribe to driver callbacks
    scan_sub_ = driver_->subscribe<LidarDataMessage>(boost::bind(&Laser::scanCB, this, _1));

    if(!lcm_publish_.good()){
      std::cerr <<"ERROR: lcm is not good()" <<std::endl;
    }

    //make sure that the sensor stream is off to begin with
    stopStream();
  }



  void Laser::startAck(const boost::shared_ptr<const LidarStartScanAckMessage>& ack)
  {
    if (ack->status == 0)
      diagnostics_.publish(SensorStatus::RUNNING);
    else
      ROS_ERROR("Failed to start laser, error code %d", ack->status);
  }

  void Laser::startStream()
  {
    ROS_INFO("Starting laser");
    //request the laser to start and publish diagnostics
    diagnostics_.publish(SensorStatus::STARTING);
    command_.reset(new Command<LidarStartScanMessage, LidarStartScanAckMessage>(LidarStartScanMessage(), driver_, boost::bind(&Laser::startAck, this, _1)));
    command_->run();
  }

  void Laser::stopAck(const boost::shared_ptr<const LidarStopScanAckMessage>& ack)
  {
    ROS_INFO("Laser stopped");
    if (ack->status == 0)
      diagnostics_.publish(SensorStatus::STOPPED);
    else
      ROS_ERROR("Failed to stop laser, error code %d", ack->status);
  }

  void Laser::stopStream()
  {
    ROS_INFO("Stopping laser");
    diagnostics_.publish(SensorStatus::STOPPING);
    command_.reset(new Command<LidarStopScanMessage, LidarStopScanAckMessage>(LidarStopScanMessage(), driver_, boost::bind(&Laser::stopAck, this, _1)));
    command_->run();
  }

  void Laser::scanCB(const boost::shared_ptr<const LidarDataMessage>& msg)
  {
    scan_caller_.addFunction(boost::bind(&Laser::processScan, this, msg));
  }

  void Laser::processScan(const boost::shared_ptr<const LidarDataMessage>& msg)
  {
    // publish scan
    ROS_DEBUG("Publish laser scan");

    ros::Time start_absolute_time = convertTime(msg->timeStart.getCurrentTime());
    ros::Time start_relative_time = convertTime(msg->timeStart);
    ros::Time end_relative_time = convertTime(msg->timeEnd);

    // LCM:
    msg_out_.utime = (int64_t) floor(start_absolute_time.toNSec()/1000);
    msg_out_.ranges.resize(msg->points);
    msg_out_.intensities.resize(msg->points);
    for (size_t i=0; i < msg->points; i++){
      msg_out_.ranges[i] = (float)msg->distance[i] / 1000.0; // from millimeters
      msg_out_.intensities[i] = (float)msg->intensity[i]; // in device units
    }
    msg_out_.nranges =msg->points;
    //  msg_out_.ranges=range_line;
    msg_out_.nintensities=0;
    //msg_out_.intensities=NULL;
    msg_out_.rad0 = -HOKUYO_SCAN_ANGLE_RADIANS / 2.0;
    msg_out_.radstep = HOKUYO_SCAN_ANGLE_RADIANS / (msg->points - 1);
    lcm_publish_.publish("ROTATING_SCAN", &msg_out_);
    /////////////////////////////////////////////////////////////

    laser_msg_.header.frame_id = frame_id_;
    laser_msg_.header.stamp = start_absolute_time;
    laser_msg_.scan_time = (end_relative_time - start_relative_time).toSec();
    laser_msg_.time_increment = laser_msg_.scan_time / msg->points;

    laser_msg_.angle_min = -HOKUYO_SCAN_ANGLE_RADIANS / 2.0;
    laser_msg_.angle_max = HOKUYO_SCAN_ANGLE_RADIANS / 2.0;
    laser_msg_.angle_increment = HOKUYO_SCAN_ANGLE_RADIANS / (msg->points - 1);

    laser_msg_.range_min = RANGE_METERS_MIN;
    laser_msg_.range_max = RANGE_METERS_MAX;

    assert(msg->points == msg->points);
    laser_msg_.ranges.resize(msg->points);
    laser_msg_.intensities.resize(msg->points);
    for (size_t i=0; i<msg->points; i++)
    {
      laser_msg_.ranges[i] = (float)msg->distance[i] / 1000.0; // from millimeters
      laser_msg_.intensities[i] = (float)msg->intensity[i]; // in device units
    }
    scan_pub_.publish(laser_msg_);

    diagnostics_.countStream();
  }


  void Laser::connectScanCB()
  {
    ROS_INFO("Connection on laser created");
    // start running the laser stream
    if (scan_pub_.getNumSubscribers() > 0)
    {
      ROS_INFO("Start streaming laser scans");
      startStream();
    }
  }

  void Laser::disconnectScanCB()
  {
    ROS_INFO("Connection on laser deleted");
    // stop running the laser stream
    if (scan_pub_.getNumSubscribers() == 0)
    {
      ROS_INFO("Stop streaming laser scans");
      stopStream();
    }
  }
}
