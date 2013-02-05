#include <multisense_driver/multisense_driver.h>

#include <boost/bind.hpp>

#include <arpa/inet.h> // For inet_addr

#include <LibSensorPodCommunications/MessageIdentifiers.h>
#include <LibPlatform/StandardException.hh>

#include <LibSensorPodCommunications/AbstractSerializedMessage.h>
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

namespace multisense_driver
{

void listener(SensorPodMessageBuffer& message_buffer, const void* user_data)
{
  //TODO: The fact that the API passes a const object is silly, but we can't use
  //the static cast below, instead we'll do something bad
  //const MultisenseDriver* driver = static_cast<const MultisenseDriver*>(user_data);
  MultisenseDriver* driver = (MultisenseDriver*) user_data;
  driver->processMessage(message_buffer);
}

void MultisenseDriver::processMessage(SensorPodMessageBuffer& message_buffer)
{
  uint8_t msg_type = message_buffer.getType();
  std::map<uint8_t, boost::shared_ptr<SignalWrapper> >::iterator sig_it = signal_map_.find(msg_type);
  if(sig_it == signal_map_.end())
  {
    fprintf(stderr, "Received an unknown message type %u!!!!!.\n", msg_type);
    throw std::runtime_error("Received an unknown message type.");
  }
  //printf("Received message 0x%02x over UDP.\n", msg_type);
  sig_it->second->call(message_buffer);
}

MultisenseDriver::MultisenseDriver(const std::string& dest_ip):
  comm_(10000, 7160), sensor_pod_address_(inet_addr(dest_ip.c_str())), sensor_pod_command_port_(9000)
{
  //Set up our signals for each type of message that can be received
  signal_map_[CamConfigMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamConfigMessage>);
  signal_map_[CamControlAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamControlAckMessage>);
  signal_map_[CamControlMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamControlMessage>);
  signal_map_[CamDataMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamDataMessage>);
  signal_map_[CamGetConfigMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamGetConfigMessage>);
  signal_map_[CamGetHistoryMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamGetHistoryMessage>);
  signal_map_[CamHistoryMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamHistoryMessage>);
  signal_map_[CamImageDataMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamImageDataMessage>);
  signal_map_[CamSetHdrAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamSetHdrAckMessage>);
  signal_map_[CamSetHdrMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamSetHdrMessage>);
  signal_map_[CamSetResolutionAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamSetResolutionAckMessage>);
  signal_map_[CamSetResolutionMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamSetResolutionMessage>);
  signal_map_[CamStartImageStreamAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamStartImageStreamAckMessage>);
  signal_map_[CamStartImageStreamMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamStartImageStreamMessage>);
  signal_map_[CamStartStreamAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamStartStreamAckMessage>);
  signal_map_[CamStartStreamMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamStartStreamMessage>);
  signal_map_[CamStopImageStreamAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamStopImageStreamAckMessage>);
  signal_map_[CamStopImageStreamMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamStopImageStreamMessage>);
  signal_map_[CamStopStreamAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamStopStreamAckMessage>);
  signal_map_[CamStopStreamMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<CamStopStreamMessage>);
  signal_map_[LedGetStatusMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LedGetStatusMessage>);
  signal_map_[LedSetAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LedSetAckMessage>);
  signal_map_[LedSetMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LedSetMessage>);
  signal_map_[LedStatusMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LedStatusMessage>);
  signal_map_[LidarDataMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LidarDataMessage>);
  signal_map_[LidarSetMotorAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LidarSetMotorAckMessage>);
  signal_map_[LidarSetMotorMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LidarSetMotorMessage>);
  signal_map_[LidarStartScanAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LidarStartScanAckMessage>);
  signal_map_[LidarStartScanMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LidarStartScanMessage>);
  signal_map_[LidarStopScanAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LidarStopScanAckMessage>);
  signal_map_[LidarStopScanMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<LidarStopScanMessage>);
  signal_map_[StatusRequestMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<StatusRequestMessage>);
  signal_map_[StatusResponseMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<StatusResponseMessage>);
  signal_map_[SysFlashOpAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<SysFlashOpAckMessage>);
  signal_map_[SysFlashOpMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<SysFlashOpMessage>);
  signal_map_[SysSetMtuAckMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<SysSetMtuAckMessage>);
  signal_map_[SysSetMtuMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<SysSetMtuMessage>);
  signal_map_[VersionRequestMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<VersionRequestMessage>);
  signal_map_[VersionResponseMessage::MSG_ID] = boost::shared_ptr<SignalWrapper>(new SignalWrapperTemplated<VersionResponseMessage>);

  //Initialize communication with the sensor
  comm_.bind();
  comm_.addListener(listener, this);
  comm_.createDispatchThread();
}

}
