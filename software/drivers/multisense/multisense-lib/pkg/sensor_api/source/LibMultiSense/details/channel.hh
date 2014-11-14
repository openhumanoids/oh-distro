/**
 * @file LibMultiSense/details/flash.cc
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
 *
 * Significant history (date, user, job code, action):
 *   2013-04-25, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/
#ifndef _LibMultiSense_details_channel_hh
#define _LibMultiSense_details_channel_hh

#include "MultiSenseChannel.hh"

#include "details/utility/Thread.hh"
#include "details/utility/BufferStream.hh"
#include "details/utility/Units.hh"
#include "details/listeners.hh"
#include "details/signal.hh"
#include "details/storage.hh"
#include "details/wire/Protocol.h"
#include "details/wire/ImageMetaMessage.h"
#include "details/wire/VersionResponseMessage.h"

#include <netinet/ip.h>

#include <unistd.h>
#include <vector>
#include <list>
#include <set>
#include <map>
#include <iostream>
#include <fstream>

namespace crl {
namespace multisense {
namespace details {

//
// The implementation details

class impl : public Channel {
public:

    //
    // Construction

    impl(const std::string& address);
    ~impl();

    //
    // Public API

    virtual Status addIsolatedCallback   (image::Callback callback,
                                          DataSource      imageSourceMask,
                                          void           *userDataP);
    virtual Status addIsolatedCallback   (lidar::Callback callback,
                                          void           *userDataP);
    virtual Status addIsolatedCallback   (pps::Callback   callback,
                                          void           *userDataP);
    virtual Status addIsolatedCallback   (imu::Callback   callback,
                                          void           *userDataP);

    virtual Status removeIsolatedCallback(image::Callback callback);
    virtual Status removeIsolatedCallback(lidar::Callback callback);
    virtual Status removeIsolatedCallback(pps::Callback   callback);
    virtual Status removeIsolatedCallback(imu::Callback   callback);

    virtual void*  reserveCallbackBuffer ();
    virtual Status releaseCallbackBuffer (void *referenceP);

    virtual Status networkTimeSynchronization(bool enabled);

    virtual Status startStreams          (DataSource mask);
    virtual Status stopStreams           (DataSource mask);
    virtual Status getEnabledStreams     (DataSource& mask);

    virtual Status setTriggerSource      (TriggerSource s);

    virtual Status setMotorSpeed         (float rpm);

    virtual Status getLightingConfig     (lighting::Config& c);
    virtual Status setLightingConfig     (const lighting::Config& c);

    virtual Status getSensorVersion      (VersionType& version);
    virtual Status getApiVersion         (VersionType& version);
    virtual Status getVersionInfo        (system::VersionInfo& v);

    virtual Status getImageConfig        (image::Config& c);
    virtual Status setImageConfig        (const image::Config& c);

    virtual Status getImageCalibration   (image::Calibration& c);
    virtual Status setImageCalibration   (const image::Calibration& c);

    virtual Status getLidarCalibration   (lidar::Calibration& c);
    virtual Status setLidarCalibration   (const lidar::Calibration& c);

    virtual Status getImageHistogram     (int64_t frameId, image::Histogram& histogram);

    virtual Status getDeviceModes        (std::vector<system::DeviceMode>& modes);

    virtual Status getMtu                (int32_t& mtu);
    virtual Status setMtu                (int32_t mtu);

    virtual Status getNetworkConfig      (system::NetworkConfig& c);
    virtual Status setNetworkConfig      (const system::NetworkConfig& c);

    virtual Status getDeviceInfo         (system::DeviceInfo& info);
    virtual Status setDeviceInfo         (const std::string& key,
                                          const system::DeviceInfo& i);

    virtual Status flashBitstream        (const std::string& file);
    virtual Status flashFirmware         (const std::string& file);

    virtual Status verifyBitstream       (const std::string& file);
    virtual Status verifyFirmware        (const std::string& file);

    virtual Status getImuInfo            (uint32_t& maxSamplesPerMessage,
                                          std::vector<imu::Info>& info);
    virtual Status getImuConfig          (uint32_t& samplesPerMessage,
                                          std::vector<imu::Config>& c);
    virtual Status setImuConfig          (bool storeSettingsInFlash,
                                          uint32_t samplesPerMessage,
                                          const std::vector<imu::Config>& c);

    virtual Status getLargeBufferDetails (uint32_t& bufferCount,
                                          uint32_t& bufferSize);
    virtual Status setLargeBuffers       (const std::vector<uint8_t*>& buffers,
                                          uint32_t                     bufferSize);

private:

    //
    // A handler prototype for custom UDP datagram reassembly

    typedef void (*UdpAssembler)(utility::BufferStreamWriter& stream,
                                 const uint8_t               *dataP,
                                 uint32_t                     offset,
                                 uint32_t                     length);

    //
    // The version of this API

    static const VersionType API_VERSION = 0x0300; // 3.0

    //
    // Misc. internal constants

    static const uint32_t MAX_MTU_SIZE               = 9000;
    static const uint16_t DEFAULT_SENSOR_TX_PORT     = 9001;
    static const uint32_t RX_POOL_LARGE_BUFFER_SIZE  = (10 * (1024 * 1024));
    static const uint32_t RX_POOL_LARGE_BUFFER_COUNT = 50;
    static const uint32_t RX_POOL_SMALL_BUFFER_SIZE  = (10 * (1024));
    static const uint32_t RX_POOL_SMALL_BUFFER_COUNT = 100;

    static const double   DEFAULT_ACK_TIMEOUT        = 0.2; // seconds
    static const uint32_t DEFAULT_ACK_ATTEMPTS       = 5;
    static const uint32_t IMAGE_META_CACHE_DEPTH     = 20;
    static const uint32_t UDP_TRACKER_CACHE_DEPTH    = 10;
    static const uint32_t TIME_SYNC_OFFSET_DECAY     = 8;

    //
    // We must protect ourselves from user callbacks misbehaving
    // and gobbling up all of our RX buffers
    //
    // These define the maximum number of datums that we will
    // queue up in a user dispatch thread.

    static const uint32_t MAX_USER_IMAGE_QUEUE_SIZE = 5;
    static const uint32_t MAX_USER_LASER_QUEUE_SIZE = 20;

    //
    // PPS and IMU callbacks do not reserve an RX buffer, so queue
    // depths are limited by RAM (via heap.)

    static const uint32_t MAX_USER_PPS_QUEUE_SIZE = 2;
    static const uint32_t MAX_USER_IMU_QUEUE_SIZE = 50;

    //
    // A re-assembler for multi-packet messages

    class UdpTracker {
    public:
        
        UdpTracker(uint32_t                     t,
                   UdpAssembler                 a,
                   utility::BufferStreamWriter& s) :
            m_totalBytesInMessage(t),
            m_bytesAssembled(0), 
            m_packetsAssembled(0),
            m_lastByteOffset(-1),
            m_assembler(a),
            m_stream(s) {};

        utility::BufferStreamWriter& stream() { return m_stream;           };
        uint32_t packets()                    { return m_packetsAssembled; };
        
        bool assemble(uint32_t       bytes,
                      uint32_t       offset,
                      const uint8_t *dataP) {

            if (offset <= m_lastByteOffset)
                CRL_EXCEPTION("out-of-order or duplicate packet");

            m_assembler(m_stream, dataP, offset, bytes);

            m_bytesAssembled   += bytes;
            m_lastByteOffset    = offset;
            m_packetsAssembled ++;

            if (m_bytesAssembled == m_totalBytesInMessage)
                return true;
            return false;
        }

    private:

        uint32_t                    m_totalBytesInMessage;
        uint32_t                    m_bytesAssembled;
        uint32_t                    m_packetsAssembled;
        int64_t                     m_lastByteOffset; 
        UdpAssembler                m_assembler;
        utility::BufferStreamWriter m_stream;
    };

    //
    // The socket identifier

    int32_t m_serverSocket;

    //
    // The address of the sensor

    struct sockaddr_in m_sensorAddress;
    
    //
    // The operating MTU of the sensor

    int32_t m_sensorMtu;

    //
    // A buffer to receive incoming UDP packets

    std::vector<uint8_t> m_incomingBuffer;

    //
    // Sequence ID for multi-packet message reassembly

    uint16_t m_txSeqId;
    int32_t  m_lastRxSeqId;
    int64_t  m_unWrappedRxSeqId;

    //
    // A cache to track incoming messages by sequence ID

    DepthCache<int64_t, UdpTracker> m_udpTrackerCache;

    //
    // A pool of RX buffers, to reduce the amount of internal copying
    
    typedef std::vector<utility::BufferStreamWriter*> BufferPool;

    BufferPool m_rxLargeBufferPool;
    BufferPool m_rxSmallBufferPool;

    //
    // A cache of image meta data

    DepthCache<int64_t, wire::ImageMeta> m_imageMetaCache;

    //
    // A map of custom UDP assemblers
    
    typedef std::map<wire::IdType, UdpAssembler> UdpAssemblerMap;

    UdpAssemblerMap m_udpAssemblerMap;

    //
    // Mutex for callback registration and dispatching

    utility::Mutex m_dispatchLock;

    //
    // Mutex for stream control

    utility::Mutex m_streamLock;

    //
    // A flag to shut down the internal threads

    bool m_threadsRunning;

    //
    // Internal UDP reception thread

    utility::Thread *m_rxThreadP;
    utility::Mutex   m_rxLock;

    //
    // Internal status thread

    utility::Thread *m_statusThreadP;

    //
    // The lists of user callbacks

    std::list<ImageListener*> m_imageListeners;
    std::list<LidarListener*> m_lidarListeners;
    std::list<PpsListener*>   m_ppsListeners;
    std::list<ImuListener*>   m_imuListeners;

    //
    // A message signal interface

    MessageWatch m_watch;

    //
    // A message storage interface

    MessageMap m_messages;

    //
    // The mask of currently enabled streams (desired)
    
    DataSource m_streamsEnabled;

    //
    // The current sensor time offset

    utility::Mutex m_timeLock;
    bool           m_timeOffsetInit;
    double         m_timeOffset;
    bool           m_networkTimeSyncEnabled;

    //
    // Cached version info from the device

    wire::VersionResponse m_sensorVersion;

    //
    // Private procedures

    template<class T, class U> Status waitData(const T&      command,
                                               U&            data,
                                               const double& timeout=double(DEFAULT_ACK_TIMEOUT),
                                               int32_t       attempts=DEFAULT_ACK_ATTEMPTS);
    template<class T> Status          waitAck (const T&      msg,
                                               wire::IdType  id=MSG_ID(T::ID),
                                               const double& timeout=double(DEFAULT_ACK_TIMEOUT),
                                               int32_t       attempts=DEFAULT_ACK_ATTEMPTS);
    
    template<class T> void       publish      (const T& message); 
    void                         publish      (const utility::BufferStreamWriter& stream);
    void                         dispatch     (utility::BufferStreamWriter& buffer);
    void                         dispatchImage(utility::BufferStream& buffer,
                                               image::Header&         header);
    void                         dispatchLidar(utility::BufferStream& buffer,
                                               lidar::Header&         header);
    void                         dispatchPps  (pps::Header& header);
    void                         dispatchImu  (imu::Header& header);


    utility::BufferStreamWriter& findFreeBuffer  (uint32_t messageLength);
    const int64_t&               unwrapSequenceId(uint16_t id);
    UdpAssembler                 getUdpAssembler (const uint8_t *udpDatagramP,
                                                  uint32_t       length);

    void                         eraseFlashRegion          (uint32_t region);
    void                         programOrVerifyFlashRegion(std::ifstream& file,
                                                            uint32_t       operation,
                                                            uint32_t       region);
    Status                       doFlashOp                 (const std::string& filename,
                                                            uint32_t           operation,
                                                            uint32_t           region);

    void                         applySensorTimeOffset(const double& offset);
    double                       sensorToLocalTime    (const double& sensorTime);
    void                         sensorToLocalTime    (const double& sensorTime,
                                                       uint32_t&     seconds,
                                                       uint32_t&     microseconds);

    void                         cleanup();
    void                         bind   ();
    void                         handle ();

    //
    // Static members

    static wire::SourceType      sourceApiToWire(DataSource mask);
    static DataSource            sourceWireToApi(wire::SourceType mask);
    static uint32_t              hardwareApiToWire(uint32_t h);
    static uint32_t              hardwareWireToApi(uint32_t h);
    static uint32_t              imagerApiToWire(uint32_t h);
    static uint32_t              imagerWireToApi(uint32_t h);
    static void                 *rxThread       (void *userDataP);
    static void                 *statusThread   (void *userDataP);
};


}}}; // namespaces

#endif // LibMultiSense_details_channel_hh
