
#include <multisense_test/multisense_stub.h>
#include <LibPlatform/StandardException.hh>

#include <LibSensorPodCommunications/CamConfigMessage.h>
#include <LibSensorPodCommunications/CamControlAckMessage.h>
#include <LibSensorPodCommunications/CamControlMessage.h>
#include <LibSensorPodCommunications/CamDataMessage.h>
#include <LibSensorPodCommunications/CamGetConfigMessage.h>
#include <LibSensorPodCommunications/CamGetHistoryMessage.h>
#include <LibSensorPodCommunications/CamHistoryMessage.h>
#include <LibSensorPodCommunications/CamImageDataMessage.h>
#include <LibSensorPodCommunications/CamSetHdrAckMessage.h>
#include <LibSensorPodCommunications/CamSetHdrMessage.h>
#include <LibSensorPodCommunications/CamSetResolutionAckMessage.h>
#include <LibSensorPodCommunications/CamSetResolutionMessage.h>
#include <LibSensorPodCommunications/CamStartImageStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStartImageStreamMessage.h>
#include <LibSensorPodCommunications/CamStartStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStartStreamMessage.h>
#include <LibSensorPodCommunications/CamStopImageStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStopImageStreamMessage.h>
#include <LibSensorPodCommunications/CamStopStreamAckMessage.h>
#include <LibSensorPodCommunications/CamStopStreamMessage.h>
#include <LibSensorPodCommunications/LedGetStatusMessage.h>
#include <LibSensorPodCommunications/LedSetAckMessage.h>
#include <LibSensorPodCommunications/LedSetMessage.h>
#include <LibSensorPodCommunications/LedStatusMessage.h>
#include <LibSensorPodCommunications/LidarDataMessage.h>
#include <LibSensorPodCommunications/LidarSetMotorAckMessage.h>
#include <LibSensorPodCommunications/LidarSetMotorMessage.h>
#include <LibSensorPodCommunications/LidarStartScanAckMessage.h>
#include <LibSensorPodCommunications/LidarStartScanMessage.h>
#include <LibSensorPodCommunications/LidarStopScanAckMessage.h>
#include <LibSensorPodCommunications/LidarStopScanMessage.h>
#include <LibSensorPodCommunications/StatusRequestMessage.h>
#include <LibSensorPodCommunications/StatusResponseMessage.h>
#include <LibSensorPodCommunications/SysFlashOpAckMessage.h>
#include <LibSensorPodCommunications/SysFlashOpMessage.h>
#include <LibSensorPodCommunications/SysSetMtuAckMessage.h>
#include <LibSensorPodCommunications/SysSetMtuMessage.h>
#include <LibSensorPodCommunications/VersionRequestMessage.h>
#include <LibSensorPodCommunications/VersionResponseMessage.h>

using namespace multisense_test;


void listener(SensorPodMessageBuffer& message, const void* userData)
{
  printf("Got a message of type 0x%02x\n", message.getType());
}

MultisenseStub::MultisenseStub(int dest_addr, int dest_port, int incoming_port) :
  comm_(incoming_port, 7160),
  dest_port_(dest_port),
  dest_addr_(dest_addr)
{
  sub_ = nh_.subscribe("multisense_stub_in", 1, &MultisenseStub::rosCb, this);
  pub_ = nh_.advertise<std_msgs::UInt8>("multisense_stub_out", 1, true);


  comm_.addListener(listener, this);
  comm_.bind();
  comm_.createDispatchThread();
}

#define CASE_MSG(MsgType) \
  case MsgType::MSG_ID: \
  { \
    MsgType msg; \
    msg.serialize(buf); \
    comm_.publish(buf); \
    break;\
  }

void MultisenseStub::rosCb(const boost::shared_ptr<const std_msgs::UInt8>& msg)
{
  SensorPodMessageBuffer buf(dest_addr_, dest_port_);
  ROS_INFO("Got msg over ros with ID 0x%02x", msg->data);

  switch (msg->data)
  {
    CASE_MSG(CamConfigMessage)
    CASE_MSG(CamControlAckMessage)
    CASE_MSG(CamControlMessage)
    CASE_MSG(CamDataMessage)
    CASE_MSG(CamGetConfigMessage)
    CASE_MSG(CamGetHistoryMessage)
    CASE_MSG(CamHistoryMessage)
    CASE_MSG(CamImageDataMessage)
    CASE_MSG(CamSetHdrAckMessage)
    CASE_MSG(CamSetHdrMessage)
    CASE_MSG(CamSetResolutionAckMessage)
    CASE_MSG(CamSetResolutionMessage)
    CASE_MSG(CamStartImageStreamAckMessage)
    CASE_MSG(CamStartImageStreamMessage)
    CASE_MSG(CamStartStreamAckMessage)
    CASE_MSG(CamStartStreamMessage)
    CASE_MSG(CamStopImageStreamAckMessage)
    CASE_MSG(CamStopImageStreamMessage)
    CASE_MSG(CamStopStreamAckMessage)
    CASE_MSG(CamStopStreamMessage)
    CASE_MSG(LedGetStatusMessage)
    CASE_MSG(LedSetAckMessage)
    CASE_MSG(LedSetMessage)
    CASE_MSG(LedStatusMessage)
    CASE_MSG(LidarDataMessage)
    CASE_MSG(LidarSetMotorAckMessage)
    CASE_MSG(LidarSetMotorMessage)
    CASE_MSG(LidarStartScanAckMessage)
    CASE_MSG(LidarStartScanMessage)
    CASE_MSG(LidarStopScanAckMessage)
    CASE_MSG(LidarStopScanMessage)
    CASE_MSG(StatusRequestMessage)
    CASE_MSG(StatusResponseMessage)
    CASE_MSG(SysFlashOpAckMessage)
    CASE_MSG(SysFlashOpMessage)
    CASE_MSG(SysSetMtuAckMessage)
    CASE_MSG(SysSetMtuMessage)
    CASE_MSG(VersionRequestMessage)
    CASE_MSG(VersionResponseMessage)
    default:
      ROS_INFO("Received unknown Multisense msg ID from ROS. MSG_ID=%u", msg->data);
      break;
  }
}
