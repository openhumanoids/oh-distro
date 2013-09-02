/**
 * @file LibMultiSense/details/public.cc
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

#include "details/utility/Functional.hh"

#include "details/channel.hh"
#include "details/query.hh"

#include "details/wire/VersionRequestMessage.h"
#include "details/wire/VersionResponseMessage.h"
#include "details/wire/StatusRequestMessage.h"
#include "details/wire/StatusResponseMessage.h"

#include "details/wire/StreamControlMessage.h"

#include "details/wire/CamControlMessage.h"
#include "details/wire/CamSetHdrMessage.h"
#include "details/wire/CamSetResolutionMessage.h"
#include "details/wire/CamGetConfigMessage.h"
#include "details/wire/CamConfigMessage.h"

#include "details/wire/LidarSetMotorMessage.h"

#include "details/wire/LedGetStatusMessage.h"
#include "details/wire/LedSetMessage.h"
#include "details/wire/LedStatusMessage.h"

#include "details/wire/SysMtuMessage.h"
#include "details/wire/SysGetMtuMessage.h"
#include "details/wire/SysFlashOpMessage.h"
#include "details/wire/SysGetNetworkMessage.h"
#include "details/wire/SysNetworkMessage.h"
#include "details/wire/SysGetDeviceInfoMessage.h"
#include "details/wire/SysDeviceInfoMessage.h"
#include "details/wire/SysGetCameraCalibrationMessage.h"
#include "details/wire/SysCameraCalibrationMessage.h"
#include "details/wire/SysGetLidarCalibrationMessage.h"
#include "details/wire/SysLidarCalibrationMessage.h"
#include "details/wire/SysGetDeviceModesMessage.h"
#include "details/wire/SysDeviceModesMessage.h"

namespace crl {
namespace multisense {
namespace details {

//
// The user may "hold on" to the buffer back-end
// of a datum within a callback thread.

__thread utility::BufferStream *dispatchBufferReferenceTP = NULL;

//
//
// Public API follows
//
//

//
// Adds a new image listener

Status impl::addIsolatedCallback(image::Callback callback, 
                                 DataSource     imageSourceMask,
                                 void           *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_imageListeners.push_back(new ImageListener(callback, 
                                                     imageSourceMask, 
                                                     userDataP,
                                                     MAX_USER_IMAGE_QUEUE_SIZE));

    } catch (const utility::Exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Error;
    }
    return Status_Ok;
}

//
// Adds a new laser listener

Status impl::addIsolatedCallback(lidar::Callback callback, 
                                 void           *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_lidarListeners.push_back(new LidarListener(callback, 
                                                     userDataP,
                                                     MAX_USER_LASER_QUEUE_SIZE));

    } catch (const utility::Exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Error;
    }
    return Status_Ok;
}

//
// Removes an image listener

Status impl::removeIsolatedCallback(image::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<ImageListener*>::iterator it;
        for(it  = m_imageListeners.begin();
            it != m_imageListeners.end();
            it ++) {
        
            if ((*it)->callback() == callback) {
                delete *it;
                m_imageListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const utility::Exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
    }

    return Status_Error;
}

//
// Removes an lidar listener

Status impl::removeIsolatedCallback(lidar::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<LidarListener*>::iterator it;
        for(it  = m_lidarListeners.begin();
            it != m_lidarListeners.end();
            it ++) {
        
            if ((*it)->callback() == callback) {
                delete *it;
                m_lidarListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const utility::Exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
    }

    return Status_Error;
}


//
// Reserve the current callback buffer being used in a dispatch thread

void *impl::reserveCallbackBuffer()
{
    if (dispatchBufferReferenceTP)  {

        try {

            return reinterpret_cast<void*>(new utility::BufferStream(*dispatchBufferReferenceTP));
            
        } catch (const utility::Exception& e) {
            
            CRL_DEBUG("exception: %s\n", e.what());
            
        } catch (...) {
            
            CRL_DEBUG("unknown exception\n");
        }
    }

    return NULL;
}

//
// Release a user reserved buffer back to us

Status impl::releaseCallbackBuffer(void *referenceP)
{
    if (referenceP)
        try {

            delete reinterpret_cast<utility::BufferStream*>(referenceP);
            return Status_Ok;

        } catch (const utility::Exception& e) {
            
            CRL_DEBUG("exception: %s\n", e.what());
            
        } catch (...) {
            
            CRL_DEBUG("unknown exception\n");
        }

    return Status_Failed;
}

//
// Get a copy of the histogram for a particular frame ID

const uint32_t *impl::getHistogram(int64_t   frameId,
                                   uint32_t& channels,
                                   uint32_t& bins)
{
    try {
        
        utility::ScopedLock lock(m_imageMetaCache.mutex());

        const wire::ImageMeta *metaP = m_imageMetaCache.find_nolock(frameId);
        if (NULL == metaP)
            CRL_EXCEPTION("no meta cached for frameId %d", frameId);

        if (m_imageMetaCache.take_nolock(frameId)) {

            channels = wire::ImageMeta::HISTOGRAM_CHANNELS;
            bins     = wire::ImageMeta::HISTOGRAM_BINS;

            return metaP->histogramP;
        }

    } catch (const utility::Exception& e) {

        CRL_DEBUG("exception: %s\n", e.what());
    }

    return NULL;
}

//
// Release the memory behind a histogram

Status impl::releaseHistogram(int64_t frameId)
{
    try {

        if (m_imageMetaCache.give(frameId))
            return Status_Ok;
        
    } catch (const utility::Exception& e) {

        CRL_DEBUG("exception: %s\n", e.what());

    } catch (...) {

        CRL_DEBUG("unknown exception\n");
    }

    return Status_Error;
}

//
// Stream control

Status impl::startStreams(DataSource mask)
{
    utility::ScopedLock lock(m_streamLock);

    wire::StreamControl cmd;

    cmd.enable(sourceApiToWire(mask));

    Status status = waitAck(cmd);
    if (Status_Ok == status)
        m_streamsEnabled |= mask;

    return status;
}
Status impl::stopStreams(DataSource mask)
{
    utility::ScopedLock lock(m_streamLock);

    wire::StreamControl cmd;

    cmd.disable(sourceApiToWire(mask));

    Status status = waitAck(cmd);
    if (Status_Ok == status)
        m_streamsEnabled &= ~mask;

    return status;
}
Status impl::getEnabledStreams(DataSource& mask)
{
    utility::ScopedLock lock(m_streamLock);

    mask = m_streamsEnabled;

    return Status_Ok;
}

//
// Set the motor speed

Status impl::setMotorSpeed(float rpm)
{
    return waitAck(wire::LidarSetMotor(rpm));
}

//
// Get/set the lighting configuration

Status impl::getLightingConfig(lighting::Config& c)
{
    Status          status;
    wire::LedStatus data;

    status = waitData(wire::LedGetStatus(), data);
    if (Status_Ok != status)
        return status;
        
    for(uint32_t i=0; i<lighting::MAX_LIGHTS; i++) {
        float duty=0.0f;
        if ((1<<i) & data.available)
            duty = (data.intensity[i] * 100.0f) / 255;
        c.setDutyCycle(i, duty);
    }

    c.setFlash(data.flash != 0);

    return Status_Ok;
}

Status impl::setLightingConfig(const lighting::Config& c)
{
    wire::LedSet msg;

    msg.flash = c.getFlash() ? 1 : 0;
    for(uint32_t i=0; i<lighting::MAX_LIGHTS; i++) {
      
        float duty = c.getDutyCycle(i);
        if (duty >= 0.0f) {  // less than zero == not set
            msg.mask |= (1<<i);
            msg.intensity[i] = 255.0f * (utility::boundValue(duty, 0.0f, 100.0f) / 100.0f);
        }
    }

    return waitAck(msg);
}

//
// Requests version from sensor

Status impl::getSensorVersion(VersionType& version)
{
    Status                status;
    wire::VersionResponse data;

    status = waitData(wire::VersionRequest(), data);
    if (Status_Ok == status)
        version = data.firmwareVersion;

    return status;
}

//
// Version of this API

Status impl::getApiVersion(VersionType& version)
{
    version = API_VERSION;
    return Status_Ok;
}

//
// Requests all versioning information

Status impl::getVersionInfo(system::VersionInfo& v)
{
    Status                status;
    wire::VersionResponse data;

    status = waitData(wire::VersionRequest(), data);
    if (Status_Ok != status)
        return status;

    char dateP[128] = {0};

    snprintf(dateP, 128, "%s, %s", __DATE__, __TIME__);

    v.apiBuildDate            = std::string(dateP);
    v.apiVersion              = API_VERSION;
    v.sensorFirmwareBuildDate = data.firmwareBuildDate;
    v.sensorFirmwareVersion   = data.firmwareVersion;
    v.sensorHardwareVersion   = data.hardwareVersion;
    v.sensorHardwareMagic     = data.hardwareMagic;
    v.sensorFpgaDna           = data.fpgaDna;

    return Status_Ok;
}

//
// Query camera configuration

Status impl::getImageConfig(image::Config& config)
{
    Status          status;
    wire::CamConfig d;

    status = waitData(wire::CamGetConfig(), d);
    if (Status_Ok != status)
        return status;

    // for access to protected calibration members
    class ConfigAccess : public image::Config {
    public:
        void setCal(float fx, float fy, float cx, float cy,
                    float tx, float ty, float tz,
                    float r,  float p,  float w) {
            m_fx = fx; m_fy = fy; m_cx = cx; m_cy = cy;
            m_tx = tx; m_ty = ty; m_tz = tz; 
            m_roll = r; m_pitch = p; m_yaw = w;
        };
    };
      
    // what is the proper c++ cast for this?
    ConfigAccess& a = *((ConfigAccess *) &config);
    
    a.setResolution(d.width, d.height);
    a.setFps(d.framesPerSecond);
    a.setGain(d.gain);
    
    a.setExposure(d.exposure);
    a.setAutoExposure(d.autoExposure != 0);
    a.setAutoExposureMax(d.autoExposureMax);
    a.setAutoExposureDecay(d.autoExposureDecay);
    a.setAutoExposureThresh(d.autoExposureThresh);
    
    a.setWhiteBalance(d.whiteBalanceRed, d.whiteBalanceBlue);
    a.setAutoWhiteBalance(d.autoWhiteBalance != 0);
    a.setAutoWhiteBalanceDecay(d.autoWhiteBalanceDecay);
    a.setAutoWhiteBalanceThresh(d.autoWhiteBalanceThresh);

    a.setCal(d.fx, d.fy, d.cx, d.cy, 
             d.tx, d.ty, d.tz,
             d.roll, d.pitch, d.yaw);
    
    return Status_Ok;
}

//
// Set camera configuration
//
// Currently several sensor messages are combined and presented
// to the user as one.

Status impl::setImageConfig(const image::Config& c)
{
    Status status;

    status = waitAck(wire::CamSetResolution(c.width(), c.height()));
    if (Status_Ok != status)
        return status;

    wire::CamControl cmd;

    cmd.framesPerSecond = c.fps();
    cmd.gain            = c.gain();
    
    cmd.exposure           = c.exposure();
    cmd.autoExposure       = c.autoExposure() ? 1 : 0;
    cmd.autoExposureMax    = c.autoExposureMax();
    cmd.autoExposureDecay  = c.autoExposureDecay();
    cmd.autoExposureThresh = c.autoExposureThresh();

    cmd.whiteBalanceRed        = c.whiteBalanceRed();
    cmd.whiteBalanceBlue       = c.whiteBalanceBlue();
    cmd.autoWhiteBalance       = c.autoWhiteBalance() ? 1 : 0;
    cmd.autoWhiteBalanceDecay  = c.autoWhiteBalanceDecay();
    cmd.autoWhiteBalanceThresh = c.autoWhiteBalanceThresh();

    return waitAck(cmd);
}

//
// Get camera calibration

Status impl::getImageCalibration(image::Calibration& c)
{
    wire::SysCameraCalibration d;

    Status status = waitData(wire::SysGetCameraCalibration(), d);
    if (Status_Ok != status)
        return status;

    CPY_ARRAY_2(c.left.M, d.left.M, 3, 3);
    CPY_ARRAY_1(c.left.D, d.left.D, 8);
    CPY_ARRAY_2(c.left.R, d.left.R, 3, 3);
    CPY_ARRAY_2(c.left.P, d.left.P, 3, 4);

    CPY_ARRAY_2(c.right.M, d.right.M, 3, 3);
    CPY_ARRAY_1(c.right.D, d.right.D, 8);
    CPY_ARRAY_2(c.right.R, d.right.R, 3, 3);
    CPY_ARRAY_2(c.right.P, d.right.P, 3, 4);

    return Status_Ok;
}

//
// Set camera calibration

Status impl::setImageCalibration(const image::Calibration& c)
{
    wire::SysCameraCalibration d;

    CPY_ARRAY_2(d.left.M, c.left.M, 3, 3);
    CPY_ARRAY_1(d.left.D, c.left.D, 8);
    CPY_ARRAY_2(d.left.R, c.left.R, 3, 3);
    CPY_ARRAY_2(d.left.P, c.left.P, 3, 4);

    CPY_ARRAY_2(d.right.M, c.right.M, 3, 3);
    CPY_ARRAY_1(d.right.D, c.right.D, 8);
    CPY_ARRAY_2(d.right.R, c.right.R, 3, 3);
    CPY_ARRAY_2(d.right.P, c.right.P, 3, 4);

    return waitAck(d);
}

//
// Get lidar calibration

Status impl::getLidarCalibration(lidar::Calibration& c)
{
    wire::SysLidarCalibration d;

    Status status = waitData(wire::SysGetLidarCalibration(), d);
    if (Status_Ok != status)
        return status;

    CPY_ARRAY_2(c.laserToSpindle, d.laserToSpindle, 4, 4);
    CPY_ARRAY_2(c.cameraToSpindleFixed, d.cameraToSpindleFixed, 4, 4);

    return Status_Ok;
}

//
// Set lidar calibration

Status impl::setLidarCalibration(const lidar::Calibration& c)
{
    wire::SysLidarCalibration d;

    CPY_ARRAY_2(d.laserToSpindle, c.laserToSpindle, 4, 4);
    CPY_ARRAY_2(d.cameraToSpindleFixed, c.cameraToSpindleFixed, 4, 4);

    return waitAck(d);
}

//
// Get a list of supported image formats / data sources

Status impl::getDeviceModes(std::vector<system::DeviceMode>& modes)
{
    wire::SysDeviceModes d;

    Status status = waitData(wire::SysGetDeviceModes(), d);
    if (Status_Ok != status)
        return Status_Error;

    std::vector<wire::DeviceMode>::const_iterator it;

    modes.clear();
    for(it = d.modes.begin(); it != d.modes.end(); it++)
        modes.push_back(system::DeviceMode((*it).width,
                                           (*it).height,
                                           (*it).supportedDataSources,
                                           (*it).flags));
    return Status_Ok;
}

//
// Set/get the mtu

Status impl::setMtu(int32_t mtu)
{
    Status status = waitAck(wire::SysMtu(mtu));
    if (Status_Ok == status)
        m_sensorMtu = mtu;

    return status;
}

Status impl::getMtu(int32_t& mtu)
{
    wire::SysMtu resp;

    Status status = waitData(wire::SysGetMtu(), resp);
    if (Status_Ok == status)
        mtu = resp.mtu;

    return status;
}

//
// Set/get the network configuration

Status impl::setNetworkConfig(const system::NetworkConfig& c)
{
    return waitAck(wire::SysNetwork(c.ipv4Address,
                                    c.ipv4Gateway,
                                    c.ipv4Netmask));
}

Status impl::getNetworkConfig(system::NetworkConfig& c)
{
    wire::SysNetwork resp;

    Status status = waitData(wire::SysGetNetwork(), resp);
    if (Status_Ok == status) {
        c.ipv4Address = resp.address;
        c.ipv4Gateway = resp.gateway;
        c.ipv4Netmask = resp.netmask;
    }

    return status;
}

//
// Get device info from sensor

Status impl::getDeviceInfo(system::DeviceInfo& info)
{
    wire::SysDeviceInfo w;

    Status status = waitData(wire::SysGetDeviceInfo(), w);
    if (Status_Ok != status)
        return status;

    info.name             = w.name;
    info.buildDate        = w.buildDate;
    info.serialNumber     = w.serialNumber;
    info.hardwareRevision = w.hardwareRevision;
    info.pcbs.clear();

    for(uint8_t i=0; i<w.numberOfPcbs; i++) {
        system::PcbInfo pcb;

        pcb.name     = w.pcbs[i].name;
        pcb.revision = w.pcbs[i].revision;

        info.pcbs.push_back(pcb);
    }
        
    info.imagerName              = w.imagerName;
    info.imagerType              = w.imagerType;
    info.imagerWidth             = w.imagerWidth;
    info.imagerHeight            = w.imagerHeight;
    info.lensName                = w.lensName;
    info.lensType                = w.lensType;
    info.nominalBaseline         = w.nominalBaseline;
    info.nominalFocalLength      = w.nominalFocalLength;
    info.nominalRelativeAperture = w.nominalRelativeAperture;
    info.lightingType            = w.lightingType;
    info.numberOfLights          = w.numberOfLights;
    info.laserName               = w.laserName;
    info.laserType               = w.laserType;
    info.motorName               = w.motorName;
    info.motorType               = w.motorType;
    info.motorGearReduction      = w.motorGearReduction;

    return Status_Ok;
}

//
// Sets the device info

Status impl::setDeviceInfo(const std::string& key,
                           const system::DeviceInfo& info)
{
    wire::SysDeviceInfo w;

    w.key              = key; // must match device firmware key
    w.name             = info.name;
    w.buildDate        = info.buildDate;
    w.serialNumber     = info.serialNumber;
    w.hardwareRevision = info.hardwareRevision;
    w.numberOfPcbs     = std::min((uint32_t) info.pcbs.size(), 
                                  (uint32_t) wire::SysDeviceInfo::MAX_PCBS);
    for(uint32_t i=0; i<w.numberOfPcbs; i++) {
        w.pcbs[i].name     = info.pcbs[i].name;
        w.pcbs[i].revision = info.pcbs[i].revision;
    }
        
    w.imagerName              = info.imagerName;
    w.imagerType              = info.imagerType;
    w.imagerWidth             = info.imagerWidth;
    w.imagerHeight            = info.imagerHeight;
    w.lensName                = info.lensName;
    w.lensType                = info.lensType;
    w.nominalBaseline         = info.nominalBaseline;
    w.nominalFocalLength      = info.nominalFocalLength;
    w.nominalRelativeAperture = info.nominalRelativeAperture;
    w.lightingType            = info.lightingType;
    w.numberOfLights          = info.numberOfLights;
    w.laserName               = info.laserName;
    w.laserType               = info.laserType;
    w.motorName               = info.motorName;
    w.motorType               = info.motorType;
    w.motorGearReduction      = info.motorGearReduction;

    return waitAck(w);
}

//
// Flash the bitstream file (dangerous!)

Status impl::flashBitstream(const std::string& filename)
{
    return doFlashOp(filename,
                     wire::SysFlashOp::OP_PROGRAM,
                     wire::SysFlashOp::RGN_BITSTREAM);
}

//
// Flash the firmware file (dangerous!)

Status impl::flashFirmware(const std::string& filename)
{
    return doFlashOp(filename,
                     wire::SysFlashOp::OP_PROGRAM,
                     wire::SysFlashOp::RGN_FIRMWARE);
}

//
// Verify the bitstream file

Status impl::verifyBitstream(const std::string& filename)
{
    return doFlashOp(filename,
                     wire::SysFlashOp::OP_VERIFY,
                     wire::SysFlashOp::RGN_BITSTREAM);
}

//
// Verify the firmware file

Status impl::verifyFirmware(const std::string& filename)
{
    return doFlashOp(filename,
                     wire::SysFlashOp::OP_VERIFY,
                     wire::SysFlashOp::RGN_FIRMWARE);
}

}}}; // namespaces
