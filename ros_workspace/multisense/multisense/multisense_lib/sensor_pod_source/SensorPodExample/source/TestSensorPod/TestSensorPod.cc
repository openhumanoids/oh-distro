/**
 * @file TestSensorPod/TestSensorPod.cc
 *
 * A simple application to test sending/receiving messages from the
 * MultiSense-SL head.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-11, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#include <iomanip>
#include <iostream>
#include <sstream>
#include <deque>
#include <cmath>

#include <LibSensorPodCommunications/SensorPodCommunications.h>

#include <LibSensorPodCommunications/VersionResponseMessage.h>
#include <LibSensorPodCommunications/VersionRequestMessage.h>

#include <LibSensorPodCommunications/StatusResponseMessage.h>
#include <LibSensorPodCommunications/StatusRequestMessage.h>

#include <LibSensorPodCommunications/CamConfigMessage.h>
#include <LibSensorPodCommunications/CamGetConfigMessage.h>

#include <LibSensorPodCommunications/CamControlMessage.h>

#include <LibSensorPodCommunications/CamStartStreamMessage.h>
#include <LibSensorPodCommunications/CamStartStreamAckMessage.h>

#include <LibSensorPodCommunications/CamStopStreamMessage.h>
#include <LibSensorPodCommunications/CamStopStreamAckMessage.h>

#include <LibSensorPodCommunications/CamStartImageStreamMessage.h>
#include <LibSensorPodCommunications/CamStartImageStreamAckMessage.h>

#include <LibSensorPodCommunications/CamStopImageStreamMessage.h>
#include <LibSensorPodCommunications/CamStopImageStreamAckMessage.h>

#include <LibSensorPodCommunications/CamDataMessage.h>
#include <LibSensorPodCommunications/CamImageDataMessage.h>

#include <LibSensorPodCommunications/LidarStartScanMessage.h>
#include <LibSensorPodCommunications/LidarStartScanAckMessage.h>

#include <LibSensorPodCommunications/LedGetStatusMessage.h>
#include <LibSensorPodCommunications/LedStatusMessage.h>

#include <LibSensorPodCommunications/LedSetMessage.h>
#include <LibSensorPodCommunications/LedSetAckMessage.h>

#include <LibSensorPodCommunications/LidarStopScanMessage.h>
#include <LibSensorPodCommunications/LidarStopScanAckMessage.h>

#include <LibSensorPodCommunications/LidarDataMessage.h>

#include <LibPlatform/StandardException.hh>
#include <LibThread/StandardThread.hh>

#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// #include <crl/utilities.hh>


// #define TUNNEL_VERSION                  (0x0202)
#define TUNNEL_VERSION                  (0x0301)


#define TEST_ANNOUNCE(testName) {                                 \
  std::cout << "**********************************************\n" \
            << "Beginning test: " << #testName << "\n"            \
            << "**********************************************\n" \
            << std::flush;                                        \
}


#define TEST_ASSERT(testName, assertion) {       \
  std::cout << (testName) << ": "                \
            << (#assertion) << "\n";             \
  if((assertion)) {                              \
    std::cout << "  >>>> PASS" << std::endl;     \
  } else {                                       \
    std::cout << "  >>>> FAIL" << std::endl;     \
    /* STANDARD_EXCEPTION((#assertion)); */      \
  }                                              \
}


// Anonymous namespace for local symbols.
namespace {

  /**
   * Static structure through which tests can monitor what's coming
   * back from the sensor head.  This structure gets filled in by the
   * listener() callback function, which is called whenever a message
   * comes in from the sensor over ethernet.
   */
  struct ListenerData {

    static size_t const   maxDequeSize = 1000000;

    int                   sensorPodAddress;
    unsigned int          sensorPodCommandPort;

    pthread_mutex_t       laserMutex;
    pthread_mutex_t       camDataMutex;
    
    uint16_t              sensorPodVersion;
    double                statusUptime;
    uint32_t              statusStatus;
    uint16_t              camConfigWidth;
    uint16_t              camConfigHeight;
    float                 camConfigFramesPerSecond;
    float                 camConfigGain;
    uint32_t              camConfigExposureTime;
    uint8_t               ledAvailable;
    uint8_t               ledIntensity[LedSetMessage::MAX_LEDS];
    uint8_t               ledFlash;
    bool                  camControlAcknowledged;
    bool                  ledSetAcknowledged;
    std::deque<uint32_t>  laserFrameNumbers;
    std::deque<crl::TimeStamp> laserTimeStamps;
    bool                  laserStartScanAckReceived;
    bool                  laserStopScanAckReceived;
    std::deque<uint32_t>  camDataFrameNumbers;
    std::deque<crl::TimeStamp> camDataTimeStamps;
    bool                  camStartStreamAckReceived;
    bool                  camStopStreamAckReceived;
    std::deque<uint32_t>  camImageDataFrameNumbers;
    std::deque<crl::TimeStamp> camImageDataTimeStamps;
    bool                  camStartImageStreamAckReceived;
    bool                  camStopImageStreamAckReceived;
    
    // Constructor.
    ListenerData()
      : sensorPodAddress(0),
        sensorPodCommandPort(9000),
        laserMutex(),
        camDataMutex(),
        sensorPodVersion(0),
        statusUptime(0.0),
        statusStatus(0),
        camConfigWidth(0),
        camConfigHeight(0),
        camConfigFramesPerSecond(0.0),
        camConfigGain(0.0),
        camConfigExposureTime(1000),
        ledAvailable(),
        ledIntensity(),
        ledFlash(0),
        camControlAcknowledged(false),
        ledSetAcknowledged(false),
        laserFrameNumbers(),
        laserTimeStamps(),
        laserStartScanAckReceived(false),
        laserStopScanAckReceived(false),
        camDataFrameNumbers(),
        camDataTimeStamps(),
        camStartStreamAckReceived(false),
        camStopStreamAckReceived(false)
      {
        pthread_mutex_init(&(this->laserMutex), NULL);
        pthread_mutex_init(&(this->camDataMutex), NULL);
        
        std::fill(&(this->ledIntensity[0]),
                  &(this->ledIntensity[LedSetMessage::MAX_LEDS]),
                  uint8_t(0));
      }
    
  };
  

  /**
   * This listener accepts messages from the SensorPod hardware, and
   * copies relevant inforation into the (file scope) ListenerData
   * instance.
   */
  void listener(SensorPodMessageBuffer& messageBuffer, const void* userData)
  {
    ListenerData* listenerDataP = (ListenerData*) userData;
    
    switch(messageBuffer.getType())
    {
    case SP_VERSION_RESPONSE:
    {
      VersionResponseMessage version;
      version.deserialize(messageBuffer);
      listenerDataP->sensorPodVersion = version.versionIdentifier;
      break;
    }

    case SP_STATUS_RESPONSE:
    {
      StatusResponseMessage status;
      status.deserialize(messageBuffer);
      listenerDataP->statusUptime = status.uptime;
      listenerDataP->statusStatus = status.status;

      std::cout << "Temperatures: " << status.temperature0 << ", " << status.temperature1 << std::endl;
      
      break;
    }

    case SP_CAM_CONFIG:
    {
      CamConfigMessage config;
      config.deserialize(messageBuffer);
      listenerDataP->camConfigWidth           = config.width;
      listenerDataP->camConfigHeight          = config.height;
      listenerDataP->camConfigFramesPerSecond = config.framesPerSecond;
      listenerDataP->camConfigGain            = config.gain;
      listenerDataP->camConfigExposureTime    = config.exposureTime;
              
      break;
    }

    case SP_CAM_CONTROL_ACK:
    {
      listenerDataP->camControlAcknowledged = true;
      break;
    }
    
    case SP_LED_SET_ACK:
    {
      listenerDataP->ledSetAcknowledged = true;
      break;
    }
    
    case SP_LED_STATUS:
    {
      LedStatusMessage ledStatusMessage;
      ledStatusMessage.deserialize(messageBuffer);
      listenerDataP->ledAvailable = ledStatusMessage.available;
      std::copy(&(ledStatusMessage.intensity[0]),
                &(ledStatusMessage.intensity[LedStatusMessage::MAX_LEDS]),
                &(listenerDataP->ledIntensity[0]));
      listenerDataP->ledFlash = ledStatusMessage.flash;
      break;
    }

    case SP_LIDAR_DATA:
    {
      // Unpack the message.
      LidarDataMessage lidarDataMessage;
      lidarDataMessage.deserialize(messageBuffer);

      // Record frame number.
      listenerDataP->laserFrameNumbers.push_back(lidarDataMessage.scanCount);
      if(listenerDataP->laserFrameNumbers.size()
         > ListenerData::maxDequeSize) {
        listenerDataP->laserFrameNumbers.pop_front();
      }

      // Record timestamp.
      listenerDataP->laserTimeStamps.push_back(lidarDataMessage.timeStart);
      if(listenerDataP->laserTimeStamps.size()
         > ListenerData::maxDequeSize) {
        listenerDataP->laserTimeStamps.pop_front();
      }

      break;
    }

    case SP_LIDAR_START_SCAN_ACK:
    {
      listenerDataP->laserStartScanAckReceived = true;
      break;
    }

    case SP_LIDAR_STOP_SCAN_ACK:
    {
      listenerDataP->laserStopScanAckReceived = true;
      break;
    }

    case SP_CAM_DATA:
    {
      // Unpack the message.
      CamDataMessage camDataMessage;
      camDataMessage.deserialize(messageBuffer);

      // Record frame number.
      listenerDataP->camDataFrameNumbers.push_back(camDataMessage.frameCount);
      if(listenerDataP->camDataFrameNumbers.size()
         > ListenerData::maxDequeSize) {
        listenerDataP->camDataFrameNumbers.pop_front();
      }

      // Record timestamp.
      listenerDataP->camDataTimeStamps.push_back(camDataMessage.timeStamp);
      if(listenerDataP->camDataTimeStamps.size()
         > ListenerData::maxDequeSize) {
        listenerDataP->camDataTimeStamps.pop_front();
      }

      break;
    }

    case SP_CAM_IMAGE_DATA:
    {
      // Unpack the message.
      CamImageDataMessage camImageDataMessage;
      camImageDataMessage.deserialize(messageBuffer);

      // Record frame number.
      listenerDataP->camImageDataFrameNumbers.push_back(camImageDataMessage.frameCount);
      if(listenerDataP->camImageDataFrameNumbers.size()
         > ListenerData::maxDequeSize) {
        listenerDataP->camImageDataFrameNumbers.pop_front();
      }

      // Record timestamp.
      listenerDataP->camImageDataTimeStamps.push_back(camImageDataMessage.timeStamp);
      if(listenerDataP->camImageDataTimeStamps.size()
         > ListenerData::maxDequeSize) {
        listenerDataP->camImageDataTimeStamps.pop_front();
      }

      break;
    }

    case SP_CAM_START_STREAM_ACK:
    {
      listenerDataP->camStartStreamAckReceived = true;
      break;
    }

    case SP_CAM_START_IMAGE_STREAM_ACK:
    {
      listenerDataP->camStartImageStreamAckReceived = true;
      break;
    }

    case SP_CAM_STOP_STREAM_ACK:
    {
      listenerDataP->camStopStreamAckReceived = true;
      break;
    }

    case SP_CAM_STOP_IMAGE_STREAM_ACK:
    {
      listenerDataP->camStopImageStreamAckReceived = true;
      break;
    }

    default:
    {
      std::cout << "Unrecognized message: " << (int)messageBuffer.getType()
                << std::endl;
      break;
    }
    }
  }

  void verboseWrite(std::string const& message, int priority = 0)
  {
    if(priority >= 2) {
      std::cout << message << std::flush;
    }
  }


  void redundantPublishMessageBuffer(
    SensorPodCommunications& communications,
    SensorPodMessageBuffer& messageBuffer)
  {
    // Try this several times to allow for latency and dropped packets.
    for(unsigned int tries = 0; tries < 5; ++tries) {
      communications.publish(messageBuffer);

      // Allow time for packets to arrive.
      usleep(100000);
    }
  }

  
  void requestCamConfig(SensorPodCommunications& communications,
                        int sensorPodAddress, unsigned int port)
  {
    // Request that camConfig information be sent.
    SensorPodMessageBuffer messageBuffer(sensorPodAddress, port);
    CamGetConfigMessage camGetConfigMessage;
    camGetConfigMessage.serialize(messageBuffer);
    redundantPublishMessageBuffer(communications, messageBuffer);
  }
  

  void requestLedStatus(SensorPodCommunications& communications,
                        int sensorPodAddress, unsigned int port)
  {
    // Request that led status information be sent.
    SensorPodMessageBuffer messageBuffer(sensorPodAddress, port);
    LedGetStatusMessage ledGetStatusMessage;
    ledGetStatusMessage.serialize(messageBuffer);
    redundantPublishMessageBuffer(communications, messageBuffer);
  }
  

  void setCamConfig(SensorPodCommunications& communications,
                    int sensorPodAddress,
                    unsigned int port,
                    float framesPerSecond,
                    float gain,
                    uint32_t exposureTime)
  {
    SensorPodMessageBuffer messageBuffer(sensorPodAddress, port);
    CamControlMessage camControlMessage;
    camControlMessage.framesPerSecond = framesPerSecond;
    camControlMessage.gain = gain;
    camControlMessage.exposureTime = exposureTime;
    camControlMessage.serialize(messageBuffer);
    redundantPublishMessageBuffer(communications, messageBuffer);
  }


  void setLeds(SensorPodCommunications& communications,
               int sensorPodAddress,
               unsigned int port,
               uint8_t mask,
               uint8_t intensity,
               uint8_t flash)
  {
    SensorPodMessageBuffer messageBuffer(sensorPodAddress, port);
    LedSetMessage ledSetMessage;
    ledSetMessage.mask = mask;
    std::fill(&(ledSetMessage.intensity[0]),
              &(ledSetMessage.intensity[LedSetMessage::MAX_LEDS]),
              intensity);
    ledSetMessage.flash = flash;
    ledSetMessage.serialize(messageBuffer);
    redundantPublishMessageBuffer(communications, messageBuffer);
  }
  
  
  void testVersionReporting(SensorPodCommunications& communications,
                            ListenerData& listenerData)
  {
    TEST_ANNOUNCE("Version Reporting");

    // Initialize to known state:
    bool gotVersionResponse = false;
    listenerData.sensorPodVersion = 0;

    // Request that version information be sent.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      VersionRequestMessage version;
      version.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
      
    // Allow time for packets to arrive.
    usleep(100000);
      
    // Check that version has been received.
    if(listenerData.sensorPodVersion != 0) {
      std::ostringstream message;
      message << "testVersionReporting(): Got version "
              << std::hex << listenerData.sensorPodVersion << std::endl;
      verboseWrite(message.str());
      gotVersionResponse = true;
    }

    TEST_ASSERT("testVersionReporting", gotVersionResponse);
    TEST_ASSERT("testVersionReporting",
                listenerData.sensorPodVersion == TUNNEL_VERSION);
  }

  
  void testStatusReporting(SensorPodCommunications& communications,
                           ListenerData& listenerData)
  {
    TEST_ANNOUNCE("Status Reporting");

    // Initialize to known state.
    bool gotStatusResponse = false;
    listenerData.statusStatus = 0;

    // Stop camera and laser streams.  This should make status be
    // not-ok for each.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      LidarStopScanMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStopStreamMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStopImageStreamMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Request that status information be sent.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      StatusRequestMessage status;
      status.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Allow time for packets to arrive.
    usleep(100000);

    TEST_ASSERT("testStatusReporting", gotStatusResponse);
    TEST_ASSERT("testStatusReporting", listenerData.statusUptime > 0.0);
    TEST_ASSERT("testStatusReporting",
                listenerData.statusStatus & STATUS_GENERAL_OK);
    TEST_ASSERT("testStatusReporting",
                !(listenerData.statusStatus & STATUS_LASER_OK));
    TEST_ASSERT("testStatusReporting",
                !(listenerData.statusStatus & STATUS_CAMERAS_OK));


    // Start laser stream.  This should make status be ok for the laser.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      LidarStartScanMessage start;
      start.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
    
    // Allow time for the laser carrier to make a complete revolution.
    sleep(10);
    
    // Request that status information be sent.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      StatusRequestMessage status;
      status.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Allow time for packets to arrive.
    usleep(100000);

    TEST_ASSERT("testStatusReporting", gotStatusResponse);
    TEST_ASSERT("testStatusReporting", listenerData.statusUptime > 0.0);
    TEST_ASSERT("testStatusReporting",
                listenerData.statusStatus & STATUS_GENERAL_OK);
    TEST_ASSERT("testStatusReporting",
                listenerData.statusStatus & STATUS_LASER_OK);
    TEST_ASSERT("testStatusReporting",
                !(listenerData.statusStatus & STATUS_CAMERAS_OK));

    
    // Start camera stream.  This should make status be ok for the cameras.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStartStreamMessage start;
      start.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
    
    // Allow time for frames to start rolling.
    sleep(1);
    
    // Request that status information be sent.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      StatusRequestMessage status;
      status.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Allow time for packets to arrive.
    usleep(100000);

    TEST_ASSERT("testStatusReporting", gotStatusResponse);
    TEST_ASSERT("testStatusReporting", listenerData.statusUptime > 0.0);
    TEST_ASSERT("testStatusReporting",
                listenerData.statusStatus & STATUS_GENERAL_OK);
    TEST_ASSERT("testStatusReporting",
                listenerData.statusStatus & STATUS_LASER_OK);
    TEST_ASSERT("testStatusReporting",
                (listenerData.statusStatus & STATUS_CAMERAS_OK));

    // Clean up.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      LidarStopScanMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStopStreamMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
  }
    

  void testCamConfigReporting(SensorPodCommunications& communications,
                              ListenerData& listenerData,
                              float targetFramesPerSecond = 0.0,
                              float targetGain = 0.0)
  {
    TEST_ANNOUNCE("Camera Configuration Reporting");

    // Initialize to known state.
    listenerData.camConfigWidth = 0;
    listenerData.camConfigHeight = 0;
    listenerData.camConfigFramesPerSecond = 0.0;
    listenerData.camConfigGain = 0.0;

    requestCamConfig(communications, listenerData.sensorPodAddress,
                     listenerData.sensorPodCommandPort);
      
    TEST_ASSERT("testCamConfigReporting", listenerData.camConfigWidth == 1024);
    TEST_ASSERT("testCamConfigReporting", listenerData.camConfigHeight == 1088);

    if(targetFramesPerSecond > 0.0) {
      TEST_ASSERT("testCamConfigReporting",
                  (listenerData.camConfigFramesPerSecond
                   == targetFramesPerSecond));
    }

    if(targetGain > 0.0) {
      TEST_ASSERT("testCamConfigReporting",
                  (listenerData.camConfigGain == targetGain));
    }
  }


  void testCamControl(SensorPodCommunications& communications,
                      ListenerData& listenerData)
  {
    TEST_ANNOUNCE("Camera Control");

    // Query for current config data.
    requestCamConfig(communications, listenerData.sensorPodAddress,
                     listenerData.sensorPodCommandPort);

    // Cam configuration reporting is not initialized until we send a
    // command.  This will be fixed in i3 firmware.
    //
    // TEST_ASSERT("testCamControl",
    //             listenerData.camConfigFramesPerSecond > 0.0);
    // TEST_ASSERT("testCamControl",
    //             listenerData.camConfigGain > 0.0);

    // Choose some new values to set.
#if 0
    float previousFramesPerSecond = listenerData.camConfigFramesPerSecond;
    float previousGain = listenerData.camConfigGain;
    float newFramesPerSecond = previousFramesPerSecond / 5.0;
    float newGain = previousGain / 2.0;
#else
    float    previousFramesPerSecond = 10.0;
    float    previousGain = 1.4;
    uint32_t previousExposureTime = 1000;
    float    newFramesPerSecond = 10.0;
    float    newGain = 1.6;
    uint32_t newExposureTime = 1000;
#endif

    // Set the new values (this is what we want to test.
    listenerData.camControlAcknowledged = 0;
    setCamConfig(communications, listenerData.sensorPodAddress,
                 listenerData.sensorPodCommandPort,
                 newFramesPerSecond, newGain,
                 newExposureTime);

    // Query to see that our changes took effect.
    requestCamConfig(communications, listenerData.sensorPodAddress,
                     listenerData.sensorPodCommandPort);

    // Verify that our messages got through.
    TEST_ASSERT("testCamControl",
                listenerData.camControlAcknowledged != 0);
    TEST_ASSERT("testCamControl",
                listenerData.camConfigFramesPerSecond == newFramesPerSecond);
    TEST_ASSERT("testCamControl",
                listenerData.camConfigGain == newGain);

    // Pause for the operator to notice the changed framerate.
    sleep(5);

    // Reset to the previous values.
    // Try this several times to allow for latency and dropped packets.
    listenerData.camControlAcknowledged = 0;
    setCamConfig(communications, listenerData.sensorPodAddress,
                 listenerData.sensorPodCommandPort,
                 previousFramesPerSecond, previousGain,
                 previousExposureTime);

    // Query to see that our changes took effect.
    requestCamConfig(communications, listenerData.sensorPodAddress,
                     listenerData.sensorPodCommandPort);
    
    // Verify that our messages got through.
    TEST_ASSERT("testCamControl",
                listenerData.camControlAcknowledged != 0);
    TEST_ASSERT("testCamControl",
                listenerData.camConfigFramesPerSecond
                == previousFramesPerSecond);
    TEST_ASSERT("testCamControl",
                listenerData.camConfigGain == previousGain);
  }
  

  // Verify that changing LED settings works.
  void testLedSet(SensorPodCommunications& communications,
                  ListenerData& listenerData)
  {
    TEST_ANNOUNCE("LED Control");

    // Turn on the camera data stream.  This will illuminate the LEDs,
    // and allow someone watching the test to verify that our LED
    // control is actually changing the lights on the front of the
    // sensor head.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStartStreamMessage start;
      start.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Query for current config data.
    requestLedStatus(communications, listenerData.sensorPodAddress,
                     listenerData.sensorPodCommandPort);

    // Choose some new values to set.
    uint8_t mask = 0xff;
    uint8_t nominalIntensity = 0x0c;
    uint8_t nominalFlash = 0;
    uint8_t newIntensity = 0x80;
    uint8_t newFlash = 1;
    
    // Set the new values (this is what we want to test.
    listenerData.ledSetAcknowledged = 0;
    setLeds(communications, listenerData.sensorPodAddress,
            listenerData.sensorPodCommandPort,
            mask, newIntensity, newFlash);

    // Query to see that our changes took effect.
    requestLedStatus(communications, listenerData.sensorPodAddress,
                     listenerData.sensorPodCommandPort);
    
    // Verify that our messages got through.
    TEST_ASSERT("testLedSet",
                listenerData.ledSetAcknowledged != 0);
    TEST_ASSERT("testLedSet",
                listenerData.ledIntensity[0] == newIntensity);
    TEST_ASSERT("testLedSet", listenerData.ledFlash == newFlash);

    // Pause for the operator to notice the changed LED settings.
    sleep(5);

    // Reset to nominal values.
    // Try this several times to allow for latency and dropped packets.
    listenerData.ledSetAcknowledged = 0;
    setLeds(communications, listenerData.sensorPodAddress,
            listenerData.sensorPodCommandPort,
            mask, nominalIntensity, nominalFlash);

    // Query to see that our changes took effect.
    requestLedStatus(communications, listenerData.sensorPodAddress,
                     listenerData.sensorPodCommandPort);
    
    // Verify that our messages got through.
    TEST_ASSERT("testLedSet",
                listenerData.ledSetAcknowledged != 0);
    TEST_ASSERT("testLedSet",
                listenerData.ledIntensity[0] == nominalIntensity);
    TEST_ASSERT("testLedSet", listenerData.ledFlash == nominalFlash);

    // Turn off camera data stream, as we no longer need to see the
    // LEDs.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStopStreamMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

  }


  // Verify that changing LED settings works.
  void testLaserStream(SensorPodCommunications& communications,
                       ListenerData& listenerData)
  {
    TEST_ANNOUNCE("Laser Data Stream");

    // Initialize state.
    listenerData.laserStopScanAckReceived = false;
    listenerData.laserStartScanAckReceived = false;

    // Request that message sending stop, so that we can start it again.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      LidarStopScanMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Clear state, so we'll notice if laser messages arrive.
    {
      crl::ScopedLock(&(listenerData.laserMutex));
      listenerData.laserFrameNumbers.clear();
      listenerData.laserTimeStamps.clear();
    }
    
    // Pause for a moment to let things stablize.
    sleep(5);

    // Fail if the stream appears to still be running, or if we never
    // received an ack for the stop message.  We lock here to avoid
    // thread safety issues with listenerData.laserFrameNumbers.
    {
      crl::ScopedLock(&(listenerData.laserMutex));
      TEST_ASSERT("testLaserStream",
                  listenerData.laserStopScanAckReceived);
      TEST_ASSERT("testLaserStream",
                  !listenerData.laserStartScanAckReceived);
      TEST_ASSERT("testLaserStream",
                  listenerData.laserFrameNumbers.empty());
    }
    
    // Request that message sending start.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      LidarStartScanMessage start;
      start.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Pause to let data arrive.
    sleep(1);

    // Fail if the stream doesn't appear to be receiving laser frames.
    // Fail if we don't have enough incoming frames to complete the
    // remaining tests.
    {
      crl::ScopedLock(&(listenerData.laserMutex));
      TEST_ASSERT("testLaserStream",
                  listenerData.laserStartScanAckReceived);
      TEST_ASSERT("testLaserStream",
                  listenerData.laserFrameNumbers.size() > 2);

      // Subsequent tests depend on having enough frames, so quit if
      // we don't have enough.
      if(listenerData.laserFrameNumbers.size() <= 2) {
        return;
      }
    }
    
    // Pause to let lots of frames arrive.
    sleep(30);
    
    // Check that the incoming frame data looks good.
    {
      crl::ScopedLock(&(listenerData.laserMutex));
      uint32_t firstFrameNumber = listenerData.laserFrameNumbers[0];
      bool foundDiscontinuity = false;
      for(uint32_t ii = 0; ii < listenerData.laserFrameNumbers.size(); ++ii) {
        if(listenerData.laserFrameNumbers[ii] != (firstFrameNumber + ii)) {
          foundDiscontinuity = true;
        }
      }
      TEST_ASSERT("testLaserStream", !foundDiscontinuity);
    }

    // Check that the incoming timestamps look good.
    {
      crl::ScopedLock(&(listenerData.laserMutex));
      double firstFrameInterval =
        static_cast<double>(listenerData.laserTimeStamps[1])
        - static_cast<double>(listenerData.laserTimeStamps[0]);
      TEST_ASSERT("testLaserStream", firstFrameInterval > 0.0);
      
      bool foundRepeatPacket = false;
      bool foundTimeSkip = false;
      for(uint32_t ii = 1; ii < listenerData.laserFrameNumbers.size(); ++ii) {
        double newFrameInterval =
          static_cast<double>(listenerData.laserTimeStamps[ii])
          - static_cast<double>(listenerData.laserTimeStamps[ii - 1]);
        if(newFrameInterval <= 0.0) {
          foundRepeatPacket = true;
        }
        
        double delta = std::fabs(newFrameInterval - firstFrameInterval);
        if((10 * delta) >= firstFrameInterval) {
          foundTimeSkip = true;
        }
      }
      TEST_ASSERT("testLaserStream", !foundRepeatPacket);
      TEST_ASSERT("testLaserStream", !foundTimeSkip);
    }

    // Clean up by requesting that message sending stop.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      LidarStopScanMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
  }


  // Verify that changing LED settings works.
  void testCamDataStream(SensorPodCommunications& communications,
                       ListenerData& listenerData)
  {
    TEST_ANNOUNCE("Camera Data Stream");

    // Initialize state.
    listenerData.camStopStreamAckReceived = false;
    listenerData.camStartStreamAckReceived = false;

    // Request that message sending stop, so that we can start it again.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStopStreamMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Clear state, so we'll notice if images arrive.
    {
      crl::ScopedLock(&(listenerData.camDataMutex));
      listenerData.camDataFrameNumbers.clear();
      listenerData.camDataTimeStamps.clear();
    }
    
    // Pause for a moment to let things stablize.
    sleep(5);

    // Fail if the stream appears to still be running, or if
    // appropriate acks haven't been received.  We lock here to avoid
    // thread safety issues with listenerData.camDataFrameNumbers.
    {
      crl::ScopedLock(&(listenerData.camDataMutex));
      TEST_ASSERT("testCamDataStream",
                  listenerData.camStopStreamAckReceived);
      TEST_ASSERT("testCamDataStream",
                  !listenerData.camStartStreamAckReceived);
      TEST_ASSERT("testCamDataStream",
                  listenerData.camDataFrameNumbers.empty());
    }
    
    // Request that message sending start.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStartStreamMessage start;
      start.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Pause to let data arrive.
    sleep(2);

    // Fail if the stream doesn't appear to be receiving camData frames.
    // Fail if we don't have enough incoming frames to complete the
    // remaining tests.
    {
      crl::ScopedLock(&(listenerData.camDataMutex));
      TEST_ASSERT("testCamDataStream",
                  listenerData.camStartStreamAckReceived);
      TEST_ASSERT("testCamDataStream",
                  listenerData.camDataFrameNumbers.size() > 2);

      // Subsequent tests depend on having enough frames, so quit if
      // we don't have enough.
      if(listenerData.camDataFrameNumbers.size() <= 2) {
        return;
      }
    }
    
    // Pause to let lots of frames arrive.
    sleep(300);
    
    // Check that the incoming frame data looks good.
    {
      crl::ScopedLock(&(listenerData.camDataMutex));
      uint32_t discontinuityCount = 0;
      for(uint32_t ii = 1; ii < listenerData.camDataFrameNumbers.size(); ++ii) {
        if(listenerData.camDataFrameNumbers[ii]
           != (listenerData.camDataFrameNumbers[ii - 1] + 1)) {
          ++discontinuityCount;
        }
      }
      {
        std::ostringstream message;
        message << "testCamDataStream(): discontinuityCount == "
                << discontinuityCount << std::endl;
        verboseWrite(message.str(), 2);
      }
      TEST_ASSERT("testCamDataStream", discontinuityCount == 0);

      // Check that the incoming timestamps look good.
      double firstFrameInterval =
        static_cast<double>(listenerData.camDataTimeStamps[1])
        - static_cast<double>(listenerData.camDataTimeStamps[0]);
      TEST_ASSERT("testCamDataStream", firstFrameInterval > 0.0);
      
      bool foundRepeatPacket = false;
      bool foundTimeSkip = false;
      for(uint32_t ii = 1; ii < listenerData.camDataFrameNumbers.size(); ++ii) {
        double newFrameInterval =
          static_cast<double>(listenerData.camDataTimeStamps[ii])
          - static_cast<double>(listenerData.camDataTimeStamps[ii - 1]);
        if(newFrameInterval <= 0.0) {
          foundRepeatPacket = true;
        }
        
        double delta = std::fabs(newFrameInterval - firstFrameInterval);
        if((10 * delta) >= firstFrameInterval) {
          foundTimeSkip = true;
        }
      }
      TEST_ASSERT("testCamDataStream", !foundRepeatPacket);
      TEST_ASSERT("testCamDataStream", !foundTimeSkip);
    }

    // Clean up by requesting that message sending stop.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStopStreamMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
  }
  
  
  // Verify that changing LED settings works.
  void testCamImageStream(SensorPodCommunications& communications,
                          ListenerData& listenerData)
  {
    TEST_ANNOUNCE("Camera Image Stream");

    // Initialize state.
    listenerData.camStopImageStreamAckReceived = false;
    listenerData.camStartImageStreamAckReceived = false;

    // Request that message sending stop, so that we can start it again.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStopImageStreamMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Clear state, so we'll notice if images arrive.
    {
      crl::ScopedLock(&(listenerData.camDataMutex));
      listenerData.camImageDataFrameNumbers.clear();
      listenerData.camImageDataTimeStamps.clear();
    }
    
    // Pause for a moment to let things stablize.
    sleep(5);

    // Fail if the stream appears to still be running, or if
    // appropriate acks haven't been received.  We lock here to avoid
    // thread safety issues with listenerData.camDataFrameNumbers.
    {
      crl::ScopedLock(&(listenerData.camDataMutex));
      TEST_ASSERT("testCamImageDataStream",
                  listenerData.camStopImageStreamAckReceived);
      TEST_ASSERT("testCamImageDataStream",
                  !listenerData.camStartImageStreamAckReceived);
      TEST_ASSERT("testCamImageDataStream",
                  listenerData.camImageDataFrameNumbers.empty());
    }
    
    // Request that message sending start.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStartImageStreamMessage start;
      start.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }

    // Pause to let data arrive.
    sleep(2);

    // Fail if the stream doesn't appear to be receiving camData frames.
    // Fail if we don't have enough incoming frames to complete the
    // remaining tests.
    {
      crl::ScopedLock(&(listenerData.camDataMutex));
      TEST_ASSERT("testCamImageDataStream",
                  listenerData.camStartImageStreamAckReceived);
      TEST_ASSERT("testCamImageDataStream",
                  listenerData.camImageDataFrameNumbers.size() > 2);

      // Subsequent tests depend on having enough frames, so quit if
      // we don't have enough.
      if(listenerData.camImageDataFrameNumbers.size() <= 2) {
        return;
      }
    }
    
    // Pause to let lots of frames arrive.
    sleep(300);
    
    // Check that the incoming frame data looks good.
    {
      crl::ScopedLock(&(listenerData.camDataMutex));
      uint32_t discontinuityCount = 0;
      for(uint32_t ii = 1; ii < listenerData.camImageDataFrameNumbers.size(); ++ii) {
        if(listenerData.camImageDataFrameNumbers[ii]
           != (listenerData.camImageDataFrameNumbers[ii - 1] + 1)) {
          ++discontinuityCount;
        }
      }
      {
        std::ostringstream message;
        message << "testCamImageDataStream(): discontinuityCount == "
                << discontinuityCount << std::endl;
        verboseWrite(message.str(), 2);
      }
      TEST_ASSERT("testCamImageDataStream", discontinuityCount == 0);

      // Check that the incoming timestamps look good.
      double firstFrameInterval =
        static_cast<double>(listenerData.camImageDataTimeStamps[1])
        - static_cast<double>(listenerData.camImageDataTimeStamps[0]);
      TEST_ASSERT("testCamImageDataStream", firstFrameInterval > 0.0);
      
      bool foundRepeatPacket = false;
      bool foundTimeSkip = false;
      for(uint32_t ii = 1; ii < listenerData.camImageDataFrameNumbers.size();
          ++ii) {
        double newFrameInterval =
          static_cast<double>(listenerData.camImageDataTimeStamps[ii])
          - static_cast<double>(listenerData.camImageDataTimeStamps[ii - 1]);
        if(newFrameInterval <= 0.0) {
          foundRepeatPacket = true;
        }
        
        double delta = std::fabs(newFrameInterval - firstFrameInterval);
        if((10 * delta) >= firstFrameInterval) {
          foundTimeSkip = true;
        }
      }
      TEST_ASSERT("testCamImageDataStream", !foundRepeatPacket);
      TEST_ASSERT("testCamImageDataStream", !foundTimeSkip);
    }

    // Clean up by requesting that message sending stop.
    {
      SensorPodMessageBuffer messageBuffer(
        listenerData.sensorPodAddress, listenerData.sensorPodCommandPort);
      CamStopImageStreamMessage stop;
      stop.serialize(messageBuffer);
      redundantPublishMessageBuffer(communications, messageBuffer);
    }
  }
  
  
  void parseArguments(int argc, char* argv[], int& sensorPodAddress)
  {
#if 0
    // Create a parser that calls exit(65) if there's a problem with the
    // command line.
    crl::utilities::OptionParser optionParser(65);

    // Specify positional arguments which are not required.
    optionParser.addPositionalArgument(
      "ADDRESS", "IP address of sensorpod (a common value is \"10.10.72.52\")."
      "  If not specified, this defaults to \"127.0.0.1\" (localhost).", false);

    // Specify required positional arguments.
    //
    // optionParser.addPositionalArgument(
    //   "INPUT_FILE", "File to be compiled.", true);

    // Specify options that do not take arguments.
    // 
    // optionParser.addOption(
    //   "NOLINK", "-c", "--no_link",
    //   "Don't run the linker.  Instead, generate .o files.");

    // Specify options that require values.
    // 
    // optionParser.addOptionWithValue(
    //   "OPTIMIZATION_LEVEL", "-O", "--optimization_level", "0",
    //   "Set optimization level.");
    
    // Parse program arguments.
    optionParser.parseCommandLine(argc, argv);

    // Recover results of parse.

    std::string ipAddress = optionParser.getValue("ADDRESS");

    // bool noLink = optionParser.getCount("NOLINK") != 0;
    // int  optimizationLevel =
    //   optionParser.convertValue<int>("OPTIMIZATION_LEVEL");


    // Check arguments and do final conversion.
    struct in_addr inAddr;
    if(0 == inet_aton(ipAddress.c_str(), &inAddr)) {
      fprintf(stderr, "Invalid IP address: %s\n", ipAddress.c_str());
      perror("inet_aton");
      exit(66);
    }

    // Copy data for the calling context to use.
    sensorPodAddress = inAddr.s_addr;
#endif /* #if 0 */
  }

} // namespace


/**
 * Entrance point for the application.
 */
int main(int argc, char* argv[])
{
  // Extract command line options.
  int sensorPodAddress = 0;
  parseArguments(argc, argv, sensorPodAddress);
    
  //
  // Initialize communications.
  //

  SensorPodCommunications communications(10000, 7160);
  // SensorPodCommunications communications(10000, 1460);
  communications.bind();

  // Add a listener for all of our requests.
  ListenerData listenerData;
  listenerData.sensorPodAddress = sensorPodAddress;
  listenerData.sensorPodCommandPort = 9000;
  communications.addListener(listener, &listenerData);
  communications.createDispatchThread();

  // Tests follow.
  testVersionReporting(communications, listenerData);
  testStatusReporting(communications, listenerData);
  testCamConfigReporting(communications, listenerData);
  testCamControl(communications, listenerData);
  testLedSet(communications, listenerData);
  testLaserStream(communications, listenerData);
  testCamDataStream(communications, listenerData);
  testCamImageStream(communications, listenerData);
  
  return 0;
}
