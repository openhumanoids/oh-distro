/**
 * @file LibMultiSense/details/channel.cc
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

#include "details/channel.hh"

#include "details/wire/DisparityMessage.h"
#include "details/wire/SysMtuMessage.h"
#include "details/wire/SysGetMtuMessage.h"

#include <netdb.h>
#include <errno.h>
#include <fcntl.h>

namespace crl {
namespace multisense {
namespace details {

//
// Implementation constructor

impl::impl(const std::string& address,
           int32_t            rxPort,
           int32_t            txPort) :
    m_serverSocket(-1),
    m_sensorAddress(),
    m_rxPort(rxPort),
    m_txPort(txPort),
    m_sensorMtu(MAX_MTU_SIZE),
    m_incomingBuffer(MAX_MTU_SIZE),
    m_txSeqId(0),
    m_lastRxSeqId(-1),
    m_unWrappedRxSeqId(0),
    m_udpTrackerCache(UDP_TRACKER_CACHE_DEPTH, 0),
    m_rxLargeBufferPool(),
    m_rxSmallBufferPool(),
    m_imageMetaCache(IMAGE_META_CACHE_DEPTH, 0),
    m_udpAssemblerMap(),
    m_dispatchLock(),
    m_streamLock(),
    m_threadsRunning(false),
    m_rxThreadP(NULL),
    m_imageListeners(),
    m_lidarListeners(),
    m_streamsEnabled(0)
{
    //
    // Make sure the sensor address is sane

    struct hostent *hostP = gethostbyname(address.c_str());
    if (NULL == hostP)
        CRL_EXCEPTION("unable to resolve \"%s\": %s",
                      address.c_str(), strerror(errno));

    //
    // Set up the address for transmission

    in_addr addr;

    memcpy(&(addr.s_addr), hostP->h_addr, hostP->h_length);
    memset(&m_sensorAddress, 0, sizeof(m_sensorAddress));

    m_sensorAddress.sin_family = AF_INET;
    m_sensorAddress.sin_port   = htons(m_txPort);
    m_sensorAddress.sin_addr   = addr;

    //
    // Create a pool of RX buffers

    for(uint32_t i=0; i<RX_POOL_LARGE_BUFFER_COUNT; i++)
        m_rxLargeBufferPool.push_back(new utility::BufferStreamWriter(RX_POOL_LARGE_BUFFER_SIZE));
    for(uint32_t i=0; i<RX_POOL_SMALL_BUFFER_COUNT; i++)
        m_rxSmallBufferPool.push_back(new utility::BufferStreamWriter(RX_POOL_SMALL_BUFFER_SIZE));

    //
    // Bind to the port

    bind();

    //
    // Register any special UDP reassemblers

    m_udpAssemblerMap[MSG_ID(wire::Disparity::ID)] = wire::Disparity::assembler;

    //
    // Create UDP reception thread

    m_threadsRunning = true;
    m_rxThreadP      = new utility::Thread(rxThread, this);

    //
    // Request the current operating MTU of the device

    wire::SysMtu mtu;

    Status status = waitData(wire::SysGetMtu(), mtu);
    if (Status_Ok != status) {
        cleanup();
        CRL_EXCEPTION("failed to establish comms with the sensor at \"%s\"",
                      address.c_str());
    }

    //
    // Use the same MTU for TX 

    m_sensorMtu = mtu.mtu;
}

//
// Implementation cleanup

void impl::cleanup()
{
    m_threadsRunning = false;

    if (m_rxThreadP)
        delete m_rxThreadP;

    std::list<ImageListener*>::const_iterator iti;
    for(iti  = m_imageListeners.begin();
        iti != m_imageListeners.end();
        iti ++)
        delete *iti;

    std::list<LidarListener*>::const_iterator itl;
    for(itl  = m_lidarListeners.begin();
        itl != m_lidarListeners.end();
        itl ++)
        delete *itl;

    BufferPool::const_iterator it;
    for(it  = m_rxLargeBufferPool.begin();
        it != m_rxLargeBufferPool.end();
        ++it)
        delete *it;
    for(it  = m_rxSmallBufferPool.begin();
        it != m_rxSmallBufferPool.end();
        ++it)
        delete *it;

    if (m_serverSocket > 0)
        close(m_serverSocket);
}

//
// Implementation destructor

impl::~impl()
{
    cleanup();
}

//
// Binds the communications channel, preparing it to send/receive data
// over the network.

void impl::bind()
{
    //
    // Create the socket.

    m_serverSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_serverSocket < 0)
        CRL_EXCEPTION("failed to create the UDP socket: %s",
                      strerror(errno));

    //
    // Turn non-blocking on.

    const int flags = fcntl(m_serverSocket, F_GETFL, 0);
    
    if (0 != fcntl(m_serverSocket, F_SETFL, flags | O_NONBLOCK))
        CRL_EXCEPTION("failed to make a socket non-blocking: %s",
                      strerror(errno));

    //
    // Allow reusing sockets.

    int reuseSocket = 1;

    if (0 != setsockopt(m_serverSocket, SOL_SOCKET, SO_REUSEADDR, (void*) &reuseSocket, 
                        sizeof(reuseSocket)))
        CRL_EXCEPTION("failed to turn on socket reuse flag: %s",
                      strerror(errno));

    //
    // We want very large buffers to store several images

    int bufferSize = 48 * 1024 * 1024;

    if (0 != setsockopt(m_serverSocket, SOL_SOCKET, SO_RCVBUF, (void*) &bufferSize, 
                        sizeof(bufferSize)) ||
        0 != setsockopt(m_serverSocket, SOL_SOCKET, SO_SNDBUF, (void*) &bufferSize, 
                        sizeof(bufferSize)))
        CRL_EXCEPTION("failed to adjust socket buffer sizes (%d bytes): %s",
                      bufferSize, strerror(errno));

    //
    // Bind the connection to the port.

    struct sockaddr_in address;

    address.sin_family      = AF_INET;
    address.sin_port        = htons(m_rxPort);
    address.sin_addr.s_addr = htonl(INADDR_ANY);

    if (0 != ::bind(m_serverSocket, (struct sockaddr*) &address, sizeof(address)))
        CRL_EXCEPTION("failed to bind the server socket to port %d: %s", 
                      m_rxPort, strerror(errno));
}

//
// Publish a stream to the sensor

void impl::publish(const utility::BufferStreamWriter& stream)
{
    //
    // Install the header 
   
    wire::Header& header = *(reinterpret_cast<wire::Header*>(stream.data()));
     
    header.magic              = wire::HEADER_MAGIC;
    header.version            = wire::HEADER_VERSION;
    header.group              = wire::HEADER_GROUP;
    header.flags              = 0;
    header.sequenceIdentifier = __sync_fetch_and_add(&m_txSeqId, 1);
    header.messageLength      = stream.tell() - sizeof(wire::Header);
    header.byteOffset         = 0;

    //
    // Send the packet along

    const int32_t ret = sendto(m_serverSocket, stream.data(), stream.tell(), 0,
                               (struct sockaddr *) &m_sensorAddress,
                               sizeof(m_sensorAddress));        
    
    if (ret != stream.tell())
        CRL_EXCEPTION("error sending data to sensor, %d/%d bytes written: %s", 
                      ret, stream.tell(), strerror(errno));
}

//
// Convert data source types from wire<->API. These match 1:1 right now, but we
// want the freedom to change the wire protocol as we see fit.

wire::SourceType impl::sourceApiToWire(DataSource mask)
{
    wire::SourceType wire_mask = 0;

    if (mask & Source_Raw_Left)               wire_mask |= wire::SOURCE_RAW_LEFT;
    if (mask & Source_Raw_Right)              wire_mask |= wire::SOURCE_RAW_RIGHT;
    if (mask & Source_Luma_Left)              wire_mask |= wire::SOURCE_LUMA_LEFT;
    if (mask & Source_Luma_Right)             wire_mask |= wire::SOURCE_LUMA_RIGHT;
    if (mask & Source_Luma_Rectified_Left)    wire_mask |= wire::SOURCE_LUMA_RECT_LEFT;
    if (mask & Source_Luma_Rectified_Right)   wire_mask |= wire::SOURCE_LUMA_RECT_RIGHT;
    if (mask & Source_Chroma_Left)            wire_mask |= wire::SOURCE_CHROMA_LEFT;
    if (mask & Source_Chroma_Right)           wire_mask |= wire::SOURCE_CHROMA_RIGHT;
    if (mask & Source_Disparity)              wire_mask |= wire::SOURCE_DISPARITY;
    if (mask & Source_Lidar_Scan)             wire_mask |= wire::SOURCE_LIDAR_SCAN;

    return wire_mask;
};

DataSource impl::sourceWireToApi(wire::SourceType mask)
{
    DataSource api_mask = 0;

    if (mask & wire::SOURCE_RAW_LEFT)         api_mask |= Source_Raw_Left;
    if (mask & wire::SOURCE_RAW_RIGHT)        api_mask |= Source_Raw_Right;
    if (mask & wire::SOURCE_LUMA_LEFT)        api_mask |= Source_Luma_Left;
    if (mask & wire::SOURCE_LUMA_RIGHT)       api_mask |= Source_Luma_Right;
    if (mask & wire::SOURCE_LUMA_RECT_LEFT)   api_mask |= Source_Luma_Rectified_Left;
    if (mask & wire::SOURCE_LUMA_RECT_RIGHT)  api_mask |= Source_Luma_Rectified_Right;
    if (mask & wire::SOURCE_CHROMA_LEFT)      api_mask |= Source_Chroma_Left;
    if (mask & wire::SOURCE_CHROMA_RIGHT)     api_mask |= Source_Chroma_Right;
    if (mask & wire::SOURCE_DISPARITY)        api_mask |= Source_Disparity;
    if (mask & wire::SOURCE_LIDAR_SCAN)       api_mask |= Source_Lidar_Scan;

    return api_mask;
};

}; // namespace details

//
// Factory methods

Channel* Channel::Create(const std::string& address,
                         int32_t            rxPort,
                         int32_t            txPort)
{
    try {

        return new details::impl(address, rxPort, txPort);

    } catch (const details::utility::Exception& e) {

        CRL_DEBUG("exception: %s\n", e.what());
        return NULL;
    }
}

void Channel::Destroy(Channel *instanceP)
{
    try {

        if (instanceP)
            delete static_cast<details::impl*>(instanceP);

    } catch (const details::utility::Exception& e) {
        
        CRL_DEBUG("exception: %s\n", e.what());
    }
}

}; // namespace multisense
}; // namespace crl
