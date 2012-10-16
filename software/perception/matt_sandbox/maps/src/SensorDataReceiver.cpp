#include "SensorDataReceiver.hpp"

#include <pcl/io/io.h>
#include <bot_frames/bot_frames.h>

SensorDataReceiver::
SensorDataReceiver() {
  mIsRunning = false;
  setMaxBufferSize(100);
}

SensorDataReceiver::
~SensorDataReceiver() {
  clearChannels();
}

void SensorDataReceiver::
setLcm(boost::shared_ptr<lcm::LCM>& iLcm) {
  mLcm = iLcm;
  clearChannels();
}

void SensorDataReceiver::
setBotParam(BotParam* iParam) {
  mBotParam = iParam;
  bot_frames_get_global(mLcm->getUnderlyingLCM(), mBotParam);
}

bool SensorDataReceiver::
addChannel(const std::string& iSensorChannel,
           const SensorType iSensorType,
           const std::string& iTransformFrom,
           const std::string& iTransformTo) {
  SubscriptionInfo info;
  info.mSensorChannel = iSensorChannel;
  info.mSensorType = iSensorType;
  info.mTransformFrom = iTransformFrom;
  info.mTransformTo = iTransformTo;

  lcm::Subscription* sub = NULL;

  switch(iSensorType) {
  case SensorTypePlanarLidar:
    sub = mLcm->subscribe(iSensorChannel,
                          &SensorDataReceiver::onLidar, this);
    break;
  case SensorTypePointCloud:
    sub = mLcm->subscribe(iSensorChannel,
                          &SensorDataReceiver::onPointCloud, this);
    break;
  case SensorTypeVelodyne:
    sub = mLcm->subscribe(iSensorChannel,
                          &SensorDataReceiver::onVelodyne, this);
    break;
  default:
    return false;
  }

  info.mSubscription = sub;
  {
    boost::mutex::scoped_lock lock(mSubscriptionsMutex);
    mSubscriptions[info.mSensorChannel] = info;
  }

  return true;
}

void SensorDataReceiver::
clearChannels() {
  boost::mutex::scoped_lock lock(mSubscriptionsMutex);
  SubscriptionMap::const_iterator iter;
  for (iter = mSubscriptions.begin(); iter != mSubscriptions.end(); ++iter) {
    mLcm->unsubscribe(iter->second.mSubscription);
  }
  mSubscriptions.clear();
}

bool SensorDataReceiver::
removeChannel(const std::string& iSensorChannel) {
  boost::mutex::scoped_lock lock(mSubscriptionsMutex);
  SubscriptionMap::iterator item = mSubscriptions.find(iSensorChannel);
  if (item == mSubscriptions.end()) {
    return false;
  }
  mLcm->unsubscribe(item->second.mSubscription);
  mSubscriptions.erase(item);
  return true;
}

void SensorDataReceiver::
setMaxBufferSize(const int iSize) {
  mDataBuffer.setMaxSize(iSize);
}

bool SensorDataReceiver::
pop(PointCloudWithPose& oData) {
  return mDataBuffer.pop(oData);
}

bool SensorDataReceiver::
waitForData(PointCloudWithPose& oData) {
  return mDataBuffer.waitForData(oData);
}

bool SensorDataReceiver::
start() {
  if (mIsRunning) {
    return false;
  }
  mIsRunning = true;
  return true;
}

bool SensorDataReceiver::
stop() {
  if (!mIsRunning) {
    return false;
  }
  mIsRunning = false;
  return true;
}

bool SensorDataReceiver::
getPose(const std::string& iChannel, const int64_t iTimestamp,
        Eigen::Isometry3d& oPose) {
  BotFrames* frames = bot_frames_get_global(mLcm->getUnderlyingLCM(),
                                            mBotParam);
  double matx[16];
  boost::mutex::scoped_lock lock(mSubscriptionsMutex);
  SubscriptionMap::const_iterator item = mSubscriptions.find(iChannel);
  if (item == mSubscriptions.end()) {
    false;
  }
  int status =
    bot_frames_get_trans_mat_4x4_with_utime(
        frames,
        item->second.mTransformFrom.c_str(),
        item->second.mTransformTo.c_str(),
        iTimestamp,
        matx);
  if (0 == status) {
    std::cerr << "SensorDataReceiver: cannot get transform from " <<
      item->second.mTransformFrom << " to " << item->second.mTransformTo <<
      std::endl;
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      oPose(i,j) = matx[i*4+j];
    }
  }
  return true;
}


void SensorDataReceiver::
onPointCloud(const lcm::ReceiveBuffer* iBuf,
             const std::string& iChannel,
             const drc::pointcloud2_t* iMessage) {

  if (!mIsRunning) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
  pointCloud.width = iMessage->width;
  pointCloud.height = iMessage->height;
  pointCloud.points.resize(iMessage->width * iMessage->height);
  pointCloud.is_dense = false;
  uint8_t* cloudData = reinterpret_cast<uint8_t*>(&pointCloud.points[0]);
  const uint8_t* messageData =
    reinterpret_cast<const uint8_t*>(&iMessage->data[0]);
  memcpy(cloudData, messageData, iMessage->data_nbytes);
  PointCloud::Ptr newCloud(new PointCloud());
  pcl::copyPointCloud(pointCloud, *newCloud);

  Eigen::Isometry3d pose;
  if (!getPose(iChannel, iMessage->utime, pose)) {
    return;
  }
  PointCloudWithPose data(iMessage->utime, newCloud, pose);
  mDataBuffer.push(data);
}

void SensorDataReceiver::
onLidar(const lcm::ReceiveBuffer* iBuf,
        const std::string& iChannel,
        const bot_core::planar_lidar_t* iMessage) {

  if (!mIsRunning) {
    return;
  }

  PointCloud::Ptr cloud(new PointCloud());
  cloud->width = iMessage->nranges;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(iMessage->nranges);
  for (int i = 0; i < iMessage->nranges; ++i) {
    double theta = iMessage->rad0 + i*iMessage->radstep;
    double range = iMessage->ranges[i];
    cloud->points[i] =
      PointCloud::PointType(cos(theta)*range, sin(theta)*range, 0);
  }

  Eigen::Isometry3d pose;
  if (!getPose(iChannel, iMessage->utime, pose)) {
    return;
  }
  PointCloudWithPose data(iMessage->utime, cloud, pose);
  mDataBuffer.push(data);
}

void SensorDataReceiver::
onVelodyne(const lcm::ReceiveBuffer* iBuf,
           const std::string& iChannel,
           const senlcm::velodyne_list_t* iMessage) {
  if (!mIsRunning) {
    return;
  }

  PointCloud::Ptr cloud(new PointCloud());
  /*
  cloud->width = iMessage->nranges;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(iMessage->nranges);
  for (int i = 0; i < iMessage->nranges; ++i) {
    double theta = iMessage->rad0 + i*iMessage->radstep;
    double range = iMessage->ranges[i];
    cloud->points[i] =
      PointCloud::PointType(cos(theta)*range, sin(theta)*range, 0);
  }
  */

  /*
  for (int i = 0; i < iMessage->num_packets; ++i) {
    process_velodyne (&(msg->packets[i]));
  }

  int do_push_motion = 0; // only push motion data if we are starting a new collection or there is a new pose

  // MFallon: I chose this parameter - not sure what its effect is:
  double hist_spc = 1000.0;


  // Algorithm:
  // Decodes and pushes velodyne_t packets into the collector
  // When the collector is full it pushes the collector in its entirety into the circular buffer
  // The collector is a portion of a scan

  // Is this a scan packet?
  if (v->packet_type == SENLCM_VELODYNE_T_TYPE_DATA_PACKET) {

    velodyne_laser_return_collection_t *lrc =
        velodyne_decode_data_packet(calib, v->data, v->datalen, v->utime);

    int ret = velodyne_collector_push_laser_returns (collector, lrc);

    velodyne_free_laser_return_collection (lrc);

    if (VELODYNE_COLLECTION_READY == ret) {

      velodyne_laser_return_collection_t *lrc =
          velodyne_collector_pull_collection (collector);

      // if enough time has elapsed since the last scan push it onto the circular buffer
      if (abs (lrc->utime - last_velodyne_data_utime) > (int64_t)(1E6/hist_spc)) {

        bot_ptr_circular_add (velodyne_data_circ, lrc);
        last_velodyne_data_utime = lrc->utime;
      } else {
        // memory leak city if this isnt here as soon as you increase the history spacing
        velodyne_free_laser_return_collection (lrc);
      }

      //starting a new collection
      do_push_motion = 1;
    }
    else if(VELODYNE_COLLECTION_READY_LOW == ret) {
      fprintf(stderr,"Low packet - ignoring");

      velodyne_laser_return_collection_t *lrc =
          velodyne_collector_pull_collection (collector);

      velodyne_free_laser_return_collection (lrc);
    }
  }

  // Update the Velodyne's state information (pos, rpy, linear/angular velocity)
  if (do_push_motion) {
    //cout << "do_push_motion 1\n";

    if (!bot_pose_last)
      return 0;

    //cout << "do_push_motion 2\n";


    // push new motion onto collector
    velodyne_state_t state;

    state.utime = v->utime;

    // find sensor pose in local/world frame
    //
    // double x_lr[6] = {self->pose->x, self->pose->y, self->pose->z,
    //                   self->pose->r, self->pose->p, self->pose->h};
    // double x_ls[6] = {0};
    // ssc_head2tail (x_ls, NULL, x_lr, self->x_vs);

    BotTrans velodyne_to_local;
    bot_frames_get_trans_with_utime (frames, "VELODYNE", "local", v->utime, &velodyne_to_local);

    memcpy (state.xyz, velodyne_to_local.trans_vec, 3*sizeof(double));
    bot_quat_to_roll_pitch_yaw (velodyne_to_local.rot_quat, state.rph);

    // Compute translational velocity
    //
    // v_velodyne = v_bot + r x w
    BotTrans velodyne_to_body;
    bot_frames_get_trans (frames, "VELODYNE", "body", &velodyne_to_body);

    double v_velodyne[3];
    double r_body_to_velodyne_local[3];
    bot_quat_rotate_to (bot_pose_last->orientation, velodyne_to_body.trans_vec, r_body_to_velodyne_local);

    // vel_rot = r x w
    double vel_rot[3];
    bot_vector_cross_3d (r_body_to_velodyne_local, bot_pose_last->rotation_rate, vel_rot);

    bot_vector_add_3d (bot_pose_last->vel, vel_rot, state.xyz_dot);

    // Compute angular rotation rate
    memcpy (state.rph_dot, bot_pose_last->rotation_rate, 3*sizeof(double));

    velodyne_collector_push_state (collector, state);
    do_push_motion = 0;
  }






  int size = bot_ptr_circular_size(velodyne_data_circ);
  vector < xyzi_t > points;
  int hist_len = VELODYNE_DATA_CIRC_SIZE;

  for (unsigned int cidx = 0;
      cidx < bot_ptr_circular_size(velodyne_data_circ) && cidx < hist_len;
      cidx++) {

    velodyne_laser_return_collection_t *lrc =(velodyne_laser_return_collection_t *) bot_ptr_circular_index(velodyne_data_circ, cidx);

    double sensor_to_local[12];
    // Working here on bot frames

    if (!bot_frames_get_trans_mat_3x4 (frames, "VELODYNE",
        "local",
        sensor_to_local)) {
      fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
      return;
    }

    //printf("LASER returns : %d\n", lrc->num_lr);

    int chunk_size = 32;// * 12;

    // Resize here:
    cloud->width   = lrc->num_lr;
    cloud->height   = 1;
    cloud->points.resize (cloud->width  *cloud->height);

    for (int s = 0; s < lrc->num_lr; s++) {
      velodyne_laser_return_t *lr = &(lrc->laser_returns[s]);

      if(s % chunk_size == 0){
        //updated the sensor_to_local transform
        if (!bot_frames_get_trans_mat_3x4_with_utime (frames, "VELODYNE",
            "local", lr->utime,
            sensor_to_local)) {
          fprintf (stderr, "Error getting bot_frames transformation from VELODYNE to local!\n");
          return;
        }
      }

      //fprintf(stderr, "\t %d - %d : %f\n", lr->physical, lr->logical, lr->phi);
      //double local_xyz[3];

      if (lr->range > SHORT_RANGE_FILTER){
        double local_xyz[3];
        bot_vector_affine_transform_3x4_3d (sensor_to_local, lr->xyz, local_xyz);
        cloud->points[s].x = local_xyz[0];
        cloud->points[s].y = local_xyz[1];
        cloud->points[s].z = local_xyz[2];
        cloud->points[s].r = (int) lr->intensity; // should use XYZI instead
        cloud->points[s].g = (int) lr->intensity; // should use XYZI instead
        cloud->points[s].b = (int) lr->intensity; // should use XYZI instead
      }
    }
  }
  */

  Eigen::Isometry3d pose;
  if (!getPose(iChannel, iMessage->utime, pose)) {
    return;
  }
  PointCloudWithPose data(iMessage->utime, cloud, pose);
  mDataBuffer.push(data);
}
