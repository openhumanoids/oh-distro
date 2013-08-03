/**
 * @file LibMultiSense/details/dispatch.cc
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
 *   2013-05-15, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#include "details/channel.hh"

#include "details/wire/AckMessage.h"

#include "details/wire/VersionResponseMessage.h"
#include "details/wire/StatusResponseMessage.h"

#include "details/wire/CamConfigMessage.h"
#include "details/wire/ImageMessage.h"
#include "details/wire/ImageMetaMessage.h"
#include "details/wire/DisparityMessage.h"

#include "details/wire/CamHistoryMessage.h"

#include "details/wire/LidarDataMessage.h"

#include "details/wire/LedStatusMessage.h"

#include "details/wire/SysMtuMessage.h"
#include "details/wire/SysNetworkMessage.h"
#include "details/wire/SysFlashResponseMessage.h"
#include "details/wire/SysDeviceInfoMessage.h"
#include "details/wire/SysCameraCalibrationMessage.h"
#include "details/wire/SysLidarCalibrationMessage.h"
#include "details/wire/SysDeviceModesMessage.h"

#include <limits>

namespace crl {
namespace multisense {
namespace details {
namespace {

//
// Default UDP assembler

void defaultUdpAssembler(utility::BufferStreamWriter& stream,
                         const uint8_t               *dataP,
                         uint32_t                     offset,
                         uint32_t                     length)
{
    stream.seek(offset);
    stream.write(dataP, length);
}

}; // anonymous

//
// Publish an image 

void impl::dispatchImage(utility::BufferStream& buffer,
                         image::Header&         header,
                         void                  *imageP)
{
    utility::ScopedLock lock(m_dispatchLock);

    std::list<ImageListener*>::const_iterator it;

    for(it  = m_imageListeners.begin();
        it != m_imageListeners.end();
        it ++)
        (*it)->dispatch(buffer, header, imageP);
}

//
// Publish a laser scan

void impl::dispatchLaser(utility::BufferStream& buffer,
                         lidar::Header&         header,
                         lidar::RangeType      *rangesP,
                         lidar::IntensityType  *intensitiesP)
{
    utility::ScopedLock lock(m_dispatchLock);

    std::list<LidarListener*>::const_iterator it;

    for(it  = m_lidarListeners.begin();
        it != m_lidarListeners.end();
        it ++)
        (*it)->dispatch(buffer, header, rangesP, intensitiesP);
}

//
// Dispatch incoming messages

void impl::dispatch(utility::BufferStreamWriter& buffer)
{
    utility::BufferStreamReader stream(buffer);

    //
    // Extract the type and version fields, which are stored
    // first in the stream

    wire::IdType      id;
    wire::VersionType version;

    stream & id;
    stream & version;
    
    //
    // Handle the message. 
    //
    // Simple, low-rate messages are dynamically stored 
    // off for possible presentation to the user in a signal 
    // handler. 
    // 
    // Larger data types use a threaded dispatch 
    // mechanism with a reference-counted buffer back-end.

    switch(id) {
    case MSG_ID(wire::LidarData::ID):
    {
        wire::LidarData scan(stream, version);
        lidar::Header   header;

	const int32_t  scanArc  = utility::degreesToRadians(270.0) * 1e6; // microradians
	const uint32_t maxRange = 30.0 * 1e3; // mm

        sensorToLocalTime(static_cast<double>(scan.timeStartSeconds) + 
                          1e-6 * static_cast<double>(scan.timeStartMicroSeconds),
                          header.timeStartSeconds, header.timeStartMicroSeconds);

        sensorToLocalTime(static_cast<double>(scan.timeEndSeconds) + 
                          1e-6 * static_cast<double>(scan.timeEndMicroSeconds),
                          header.timeEndSeconds, header.timeEndMicroSeconds);

        header.scanId            = scan.scanCount;
        header.spindleAngleStart = scan.angleStart;
        header.spindleAngleEnd   = scan.angleEnd;
	header.scanArc           = scanArc;
	header.maxRange          = maxRange;
        header.pointCount        = scan.points;

        dispatchLaser(buffer, header, scan.distanceP, scan.intensityP);

        break;
    }
    case MSG_ID(wire::ImageMeta::ID):
    {
        wire::ImageMeta *metaP = new (std::nothrow) wire::ImageMeta(stream, version);

        if (NULL == metaP)
            CRL_EXCEPTION("unable to allocate metadata");

        m_imageMetaCache.insert(metaP->frameId, metaP); // destroys oldest

        break;
    }
    case MSG_ID(wire::Image::ID):
    {
        wire::Image image(stream, version);

        const wire::ImageMeta *metaP = m_imageMetaCache.find(image.frameId);
        if (NULL == metaP)
            CRL_EXCEPTION("no meta cached for frameId %d", image.frameId);

        image::Header header;

        sensorToLocalTime(static_cast<double>(metaP->timeSeconds) + 
                          1e-6 * static_cast<double>(metaP->timeMicroSeconds),
                          header.timeSeconds, header.timeMicroSeconds);

        header.source           = sourceWireToApi(image.source);
        header.bitsPerPixel     = image.bitsPerPixel;
        header.width            = image.width;
        header.height           = image.height;
        header.frameId          = image.frameId;
        header.exposure         = metaP->exposureTime;
        header.gain             = metaP->gain;
        header.framesPerSecond  = metaP->framesPerSecond;
        
        dispatchImage(buffer, header, image.dataP);

        break;
    }
    case MSG_ID(wire::Disparity::ID):
    {
        wire::Disparity image(stream, version);

        const wire::ImageMeta *metaP = m_imageMetaCache.find(image.frameId);
        if (NULL == metaP)
            CRL_EXCEPTION("no meta cached for frameId %d", image.frameId);
        
        image::Header header;

        sensorToLocalTime(static_cast<double>(metaP->timeSeconds) + 
                          1e-6 * static_cast<double>(metaP->timeMicroSeconds),
                          header.timeSeconds, header.timeMicroSeconds);

        header.source           = Source_Disparity;
        header.bitsPerPixel     = wire::Disparity::API_BITS_PER_PIXEL;
        header.width            = image.width;
        header.height           = image.height;
        header.frameId          = image.frameId;
        header.exposure         = metaP->exposureTime;
        header.gain             = metaP->gain;
        header.framesPerSecond  = metaP->framesPerSecond;

        dispatchImage(buffer, header, image.dataP);

        break;
    }
    case MSG_ID(wire::Ack::ID):
        break; // handle below
    case MSG_ID(wire::CamConfig::ID):
        m_messages.store(wire::CamConfig(stream, version));
        break;
    case MSG_ID(wire::CamHistory::ID):
        m_messages.store(wire::CamHistory(stream, version));
        break;
    case MSG_ID(wire::LedStatus::ID):
        m_messages.store(wire::LedStatus(stream, version));
        break;
    case MSG_ID(wire::SysFlashResponse::ID):
        m_messages.store(wire::SysFlashResponse(stream, version));
        break;
    case MSG_ID(wire::SysDeviceInfo::ID):
        m_messages.store(wire::SysDeviceInfo(stream, version));
        break;
    case MSG_ID(wire::SysCameraCalibration::ID):
        m_messages.store(wire::SysCameraCalibration(stream, version));
        break;
    case MSG_ID(wire::SysLidarCalibration::ID):
        m_messages.store(wire::SysLidarCalibration(stream, version));
        break;
    case MSG_ID(wire::SysMtu::ID):
        m_messages.store(wire::SysMtu(stream, version));
        break;
    case MSG_ID(wire::SysNetwork::ID):
        m_messages.store(wire::SysNetwork(stream, version));
        break;
    case MSG_ID(wire::SysDeviceModes::ID):
        m_messages.store(wire::SysDeviceModes(stream, version));
        break;
    case MSG_ID(wire::VersionResponse::ID):
        m_messages.store(wire::VersionResponse(stream, version));
        break;
    case MSG_ID(wire::StatusResponse::ID):
        m_messages.store(wire::StatusResponse(stream, version));   
        break;
    default:

        CRL_DEBUG("unknown message received: id=%d, version=%d\n",
                  id, version);
        return;
    }

    //
    // Signal any listeners (_after_ the message is stored/dispatched)
    //
    // A [n]ack is a special case where we signal
    // the returned status code of the ack'd command, 
    // otherwise we signal valid reception of this message.

    switch(id) {
    case MSG_ID(wire::Ack::ID):
	m_watch.signal(wire::Ack(stream, version));
	break;
    default:	
	m_watch.signal(id);
	break;
    }
}

//
// Get a UDP assembler for this message type. We are given
// the first UDP packet in the stream

impl::UdpAssembler impl::getUdpAssembler(const uint8_t *firstDatagramP,
                                         uint32_t       length)
{
    //
    // Get the message type, it is stored after wire::Header

    utility::BufferStreamReader stream(firstDatagramP, length);
    stream.seek(sizeof(wire::Header));

    wire::IdType messageType; 
    stream & messageType;

    //
    // See if a custom handler has been registered

    UdpAssemblerMap::const_iterator it = m_udpAssemblerMap.find(messageType);

    if (m_udpAssemblerMap.end() != it)
        return it->second;
    else
        return defaultUdpAssembler;
}

//
// Find a suitably sized buffer for the incoming message

utility::BufferStreamWriter& impl::findFreeBuffer(uint32_t messageLength)
{    
    BufferPool *bP = NULL;
    
    if (messageLength <= RX_POOL_SMALL_BUFFER_SIZE)
        bP = &(m_rxSmallBufferPool);
    else if (messageLength <= RX_POOL_LARGE_BUFFER_SIZE)
        bP = &(m_rxLargeBufferPool);
    else
        CRL_EXCEPTION("message too large: %d bytes", messageLength);

    //
    // TODO: re-think the shared() mechanism of the buffers, so we do not
    //       have to walk this vector.

    BufferPool::const_iterator it = bP->begin();
    for(; it != bP->end(); it++)
        if (false == (*it)->shared())
            return *(*it);

    CRL_EXCEPTION("no free RX buffers (%d in use by consumers)\n", bP->size());
}

//
// Unwrap a 16-bit wire sequence ID into a unique 64-bit local ID

const int64_t& impl::unwrapSequenceId(uint16_t wireId)
{
    //
    // Look for a sequence change

    if (wireId != m_lastRxSeqId) {

        const uint16_t ID_MAX    = std::numeric_limits<uint16_t>::max();
        const uint16_t ID_CENTER = ID_MAX / 2;

        //
        // Seed

        if (-1 == m_lastRxSeqId)
            m_lastRxSeqId = m_unWrappedRxSeqId = wireId;

        //
        // Detect 16-bit wrap

        else if (wireId        < ID_CENTER   &&
                 m_lastRxSeqId > ID_CENTER) {

            m_unWrappedRxSeqId += 1 + (ID_MAX - m_lastRxSeqId) + wireId;

        //
        // Normal case

        } else
            m_unWrappedRxSeqId += wireId - m_lastRxSeqId;

        //
        // Remember change

        m_lastRxSeqId = wireId;
    }

    return m_unWrappedRxSeqId;
}

//
// Handles any incoming packets

void impl::handle()
{
    for(;;) {
 
        //
        // Receive the packet
        
        const ssize_t bytesRead = recvfrom(m_serverSocket,
                                           m_incomingBuffer.data(),
                                           m_incomingBuffer.size(),
                                           0, NULL, NULL);
        //
        // Nothing left to read
        
        if (bytesRead < 0)
            break;

        //
        // Check for undersized packets

        else if (bytesRead < (ssize_t) sizeof(wire::Header))
            CRL_EXCEPTION("undersized packet: %d/%d bytes\n",
                          bytesRead, sizeof(wire::Header));

        //
        // For convenience below

        const uint8_t *inP = reinterpret_cast<const uint8_t*>(m_incomingBuffer.data());

        //
        // Validate the header

        const wire::Header& header = *(reinterpret_cast<const wire::Header*>(inP));

        if (wire::HEADER_MAGIC != header.magic)
            CRL_EXCEPTION("bad protocol magic: 0x%x, expecting 0x%x",
                          header.magic, wire::HEADER_MAGIC);
        else if (wire::HEADER_VERSION != header.version)
            CRL_EXCEPTION("bad protocol version: 0x%x, expecting 0x%x",
                          header.version, wire::HEADER_VERSION);
        else if (wire::HEADER_GROUP != header.group)
            CRL_EXCEPTION("bad protocol group: 0x%x, expecting 0x%x",
                          header.group, wire::HEADER_GROUP);

        //
        // Unwrap the sequence identifier

        const int64_t& sequence = unwrapSequenceId(header.sequenceIdentifier);

        //
        // See if we are already tracking this messge ID

        UdpTracker *trP = m_udpTrackerCache.find(sequence);
        if (NULL == trP) {

            //
            // If we drop first packet, we will drop entire message. Currently we 
            // require the first datagram in order to assign an assembler.
            // TODO: re-think this.

            if (0 != header.byteOffset)
                continue;
            else {

                //
                // Create a new tracker for this sequence id. If the tracker cache is
                // full, the oldest tracker will be released automatically.

                m_udpTrackerCache.insert(sequence,
                                         (trP = new UdpTracker(getUdpAssembler(inP, bytesRead),
                                                               findFreeBuffer(header.messageLength))));
            }
        }
     
        //
        // Check for duplicate or out-of-order packets.
        // Out-of-order packets are technically acceptable, however 
        // the cost to check if a packet is out-of-order vs. duplicated is high.
        // Duplicate packets can easily happen on misconfigured network interfaces.

        if (header.byteOffset <= trP->lastByteOffset)
            CRL_EXCEPTION("out-of-order or duplicate packet for sequence %lld",
                          sequence);
        else
            trP->lastByteOffset = header.byteOffset;

        //
        // Assemble the datagram into the message stream

        const uint32_t messageBytes = bytesRead - sizeof(wire::Header);

        trP->assembler(trP->stream,
                       &(inP[sizeof(wire::Header)]),
                       header.byteOffset, messageBytes);
                
        //
        // Have we fully re-assembled this message yet?
        
        trP->bytesAssembled += messageBytes;

        if (header.messageLength == trP->bytesAssembled) {

            //
            // Dispatch to any listeners

            dispatch(trP->stream);

            //
            // Free the tracker

            m_udpTrackerCache.remove(sequence);
        }
    }
}

//
// This thread waits for UDP packets

void *impl::rxThread(void *userDataP)
{
    impl     *selfP  = reinterpret_cast<impl*>(userDataP);
    const int server = selfP->m_serverSocket;
    fd_set    readSet;

    //
    // Loop until shutdown

    while(selfP->m_threadsRunning) {

        //
        // Add the server socket to the read set.

        FD_ZERO(&readSet);
        FD_SET(server, &readSet);

        //
        // Wait for a new packet to arrive, timing out every once in awhile

        struct timeval tv = {0, 200000}; // 5Hz
        const int result  = select(server+1, &readSet, NULL, NULL, &tv);
        if (result <= 0)
            continue;

        //
        // Let the comm object handle decoding

        try {

            selfP->handle();

        } catch (const utility::Exception& e) {
                    
            CRL_DEBUG("exception while decoding packet: %s\n", e.what());

        } catch ( ... ) {
            
            CRL_DEBUG("unknown exception while decoding packet\n");
        }
    }

    return NULL;
}

}}}; // namespaces
