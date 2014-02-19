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

#include <stdlib.h>

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
#include "details/wire/CamSetTriggerSourceMessage.h"

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

#include "details/wire/ImuGetInfoMessage.h"
#include "details/wire/ImuGetConfigMessage.h"
#include "details/wire/ImuInfoMessage.h"
#include "details/wire/ImuConfigMessage.h"

#include "details/wire/SysTestMtuMessage.h"
#include "details/wire/SysTestMtuResponseMessage.h"

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

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
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
                                                     0,
                                                     userDataP,
                                                     MAX_USER_LASER_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Adds a new PPS listener

Status impl::addIsolatedCallback(pps::Callback callback, 
                                 void         *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_ppsListeners.push_back(new PpsListener(callback, 
                                                 0,
                                                 userDataP,
                                                 MAX_USER_PPS_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
    return Status_Ok;
}

//
// Adds a new IMU listener

Status impl::addIsolatedCallback(imu::Callback callback, 
                                 void         *userDataP)
{
    try {

        utility::ScopedLock lock(m_dispatchLock);
        m_imuListeners.push_back(new ImuListener(callback, 
                                                 0,
                                                 userDataP,
                                                 MAX_USER_IMU_QUEUE_SIZE));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
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

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes a lidar listener

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

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes a PPS listener

Status impl::removeIsolatedCallback(pps::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<PpsListener*>::iterator it;
        for(it  = m_ppsListeners.begin();
            it != m_ppsListeners.end();
            it ++) {
        
            if ((*it)->callback() == callback) {
                delete *it;
                m_ppsListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Removes an IMU listener

Status impl::removeIsolatedCallback(imu::Callback callback)
{
    try {
        utility::ScopedLock lock(m_dispatchLock);

        std::list<ImuListener*>::iterator it;
        for(it  = m_imuListeners.begin();
            it != m_imuListeners.end();
            it ++) {
        
            if ((*it)->callback() == callback) {
                delete *it;
                m_imuListeners.erase(it);
                return Status_Ok;
            }
        }

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
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
            
        } catch (const std::exception& e) {
            
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
    if (referenceP) {
        try {

            delete reinterpret_cast<utility::BufferStream*>(referenceP);
            return Status_Ok;

        } catch (const std::exception& e) {
            
            CRL_DEBUG("exception: %s\n", e.what());
            return Status_Exception;
            
        } catch (...) {
            
            CRL_DEBUG("unknown exception\n");
            return Status_Exception;
        }
    }

    return Status_Error;
}

//
// Get a copy of the histogram for a particular frame ID

Status impl::getImageHistogram(int64_t           frameId,
                               image::Histogram& histogram)
{
    try {
        
        utility::ScopedLock lock(m_imageMetaCache.mutex());

        const wire::ImageMeta *metaP = m_imageMetaCache.find_nolock(frameId);
        if (NULL == metaP) {
            CRL_DEBUG("no meta cached for frameId %ld", frameId);
            return Status_Failed;
        }

        histogram.channels = wire::ImageMeta::HISTOGRAM_CHANNELS;
        histogram.bins     = wire::ImageMeta::HISTOGRAM_BINS;

        const int entries   = histogram.channels * histogram.bins;
        const int sizeBytes = entries * sizeof(uint32_t);

        histogram.data.resize(entries);
        memcpy(&(histogram.data[0]), metaP->histogramP, sizeBytes);

        return Status_Ok;

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Error;
}

//
// Enable/disable network-based time synchronization

Status impl::networkTimeSynchronization(bool enabled)
{
    m_networkTimeSyncEnabled = enabled;
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
// Set the trigger source

Status impl::setTriggerSource(TriggerSource s)
{
    uint32_t wireSource;

    switch(s) {
    case Trigger_Internal: 

        wireSource = wire::CamSetTriggerSource::SOURCE_INTERNAL;
        break;

    case Trigger_External:

        wireSource = wire::CamSetTriggerSource::SOURCE_EXTERNAL;
        break;

    default:

        return Status_Error;
    }

    return waitAck(wire::CamSetTriggerSource(wireSource));
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
    version = m_sensorVersion.firmwareVersion;
    return Status_Ok;
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
    char dateP[128] = {0};

    snprintf(dateP, 128, "%s, %s", __DATE__, __TIME__);

    v.apiBuildDate            = std::string(dateP);
    v.apiVersion              = API_VERSION;
    v.sensorFirmwareBuildDate = m_sensorVersion.firmwareBuildDate;
    v.sensorFirmwareVersion   = m_sensorVersion.firmwareVersion;
    v.sensorHardwareVersion   = m_sensorVersion.hardwareVersion;
    v.sensorHardwareMagic     = m_sensorVersion.hardwareMagic;
    v.sensorFpgaDna           = m_sensorVersion.fpgaDna;

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
    if (-1 == d.disparities) { // pre v2.3 firmware
        if (1024 == d.width)   // TODO: check for monocular
            d.disparities = 128;
        else
            d.disparities = 0;
    }
    a.setDisparities(d.disparities);
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
    a.setStereoPostFilterStrength(d.stereoPostFilterStrength);

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

    status = waitAck(wire::CamSetResolution(c.width(), 
                                            c.height(),
                                            c.disparities()));
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

    cmd.whiteBalanceRed          = c.whiteBalanceRed();
    cmd.whiteBalanceBlue         = c.whiteBalanceBlue();
    cmd.autoWhiteBalance         = c.autoWhiteBalance() ? 1 : 0;
    cmd.autoWhiteBalanceDecay    = c.autoWhiteBalanceDecay();
    cmd.autoWhiteBalanceThresh   = c.autoWhiteBalanceThresh();
    cmd.stereoPostFilterStrength = c.stereoPostFilterStrength();

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

    modes.resize(d.modes.size());
    for(uint32_t i=0; i<d.modes.size(); i++) {

        system::DeviceMode&     a = modes[i]; 
        const wire::DeviceMode& w = d.modes[i];

        a.width                = w.width;
        a.height               = w.height;
        a.supportedDataSources = sourceWireToApi(w.supportedDataSources);
        if (m_sensorVersion.firmwareVersion >= 0x0203)
            a.disparities = w.disparities;
        else 
            a.disparities = (a.width == 1024) ? 128 : 0;
    }
                        
    return Status_Ok;
}

//
// Set/get the mtu

Status impl::setMtu(int32_t mtu)
{
    Status status = Status_Ok;

    //
    // Firmware v2.3 or better will send us an MTU-sized
    // response packet, which can be used to verify the
    // MTU setting before we actually make the change.

    if (m_sensorVersion.firmwareVersion <= 0x0202)
        status = waitAck(wire::SysMtu(mtu));
    else {

        wire::SysTestMtuResponse resp;
        status = waitData(wire::SysTestMtu(mtu), resp);
        if (Status_Ok == status)
            status = waitAck(wire::SysMtu(mtu));
    }

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
    info.hardwareRevision = hardwareWireToApi(w.hardwareRevision);
    info.pcbs.clear();

    for(uint8_t i=0; i<w.numberOfPcbs; i++) {
        system::PcbInfo pcb;

        pcb.name     = w.pcbs[i].name;
        pcb.revision = w.pcbs[i].revision;

        info.pcbs.push_back(pcb);
    }
        
    info.imagerName              = w.imagerName;
    info.imagerType              = imagerWireToApi(w.imagerType);
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
    w.hardwareRevision = hardwareApiToWire(info.hardwareRevision);
    w.numberOfPcbs     = std::min((uint32_t) info.pcbs.size(), 
                                  (uint32_t) wire::SysDeviceInfo::MAX_PCBS);
    for(uint32_t i=0; i<w.numberOfPcbs; i++) {
        w.pcbs[i].name     = info.pcbs[i].name;
        w.pcbs[i].revision = info.pcbs[i].revision;
    }
        
    w.imagerName              = info.imagerName;
    w.imagerType              = imagerApiToWire(info.imagerType);
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

//
// Get IMU information

Status impl::getImuInfo(uint32_t& maxSamplesPerMessage,
                        std::vector<imu::Info>& info)
{
    wire::ImuInfo w;

    Status status = waitData(wire::ImuGetInfo(), w);
    if (Status_Ok != status)
        return status;

    //
    // Wire --> API

    maxSamplesPerMessage = w.maxSamplesPerMessage;
    info.resize(w.details.size());
    for(uint32_t i=0; i<w.details.size(); i++) {

        const wire::imu::Details& d = w.details[i];

        info[i].name   = d.name;
        info[i].device = d.device;
        info[i].units  = d.units;
        
        info[i].rates.resize(d.rates.size());
        for(uint32_t j=0; j<d.rates.size(); j++) {
            info[i].rates[j].sampleRate      = d.rates[j].sampleRate;
            info[i].rates[j].bandwidthCutoff = d.rates[j].bandwidthCutoff;
        }
        info[i].ranges.resize(d.ranges.size());
        for(uint32_t j=0; j<d.ranges.size(); j++) {
            info[i].ranges[j].range      = d.ranges[j].range;
            info[i].ranges[j].resolution = d.ranges[j].resolution;
        }
    }

    return Status_Ok;
}

//
// Get IMU configuration

Status impl::getImuConfig(uint32_t& samplesPerMessage,
                          std::vector<imu::Config>& c)
{
    wire::ImuConfig w;

    Status status = waitData(wire::ImuGetConfig(), w);
    if (Status_Ok != status)
        return status;

    //
    // Wire --> API

    samplesPerMessage = w.samplesPerMessage;
    c.resize(w.configs.size());
    for(uint32_t i=0; i<w.configs.size(); i++) {
        c[i].name            = w.configs[i].name;
        c[i].enabled         = (w.configs[i].flags & wire::imu::Config::FLAGS_ENABLED);
        c[i].rateTableIndex  = w.configs[i].rateTableIndex;
        c[i].rangeTableIndex = w.configs[i].rangeTableIndex;
    }

    return Status_Ok;
}

//
// Set IMU configuration

Status impl::setImuConfig(bool storeSettingsInFlash,
                          uint32_t samplesPerMessage,
                          const std::vector<imu::Config>& c)
{
    wire::ImuConfig w;

    //
    // API --> wire

    w.storeSettingsInFlash = storeSettingsInFlash ? 1 : 0;
    w.samplesPerMessage    = samplesPerMessage;
    w.configs.resize(c.size());
    for(uint32_t i=0; i<c.size(); i++) {
        w.configs[i].name            = c[i].name;
        w.configs[i].flags           = c[i].enabled ? wire::imu::Config::FLAGS_ENABLED : 0;
        w.configs[i].rateTableIndex  = c[i].rateTableIndex;
        w.configs[i].rangeTableIndex = c[i].rangeTableIndex;
    }

    return waitAck(w);
}

//
// Get recommended large buffer pool count/size

Status impl::getLargeBufferDetails(uint32_t& bufferCount,
                                   uint32_t& bufferSize)
{
    bufferCount = RX_POOL_LARGE_BUFFER_COUNT;
    bufferSize  = RX_POOL_LARGE_BUFFER_SIZE;
    
    return Status_Ok;
}

//
// Replace internal large buffers with user supplied

Status impl::setLargeBuffers(const std::vector<uint8_t*>& buffers,
                             uint32_t                     bufferSize)
{
    if (buffers.size() < RX_POOL_LARGE_BUFFER_COUNT)
        CRL_DEBUG("WARNING: supplying less than recommended number of large buffers: %ld/%d\n",
                  buffers.size(), RX_POOL_LARGE_BUFFER_COUNT);
    if (bufferSize < RX_POOL_LARGE_BUFFER_SIZE)
        CRL_DEBUG("WARNING: supplying smaller than recommended large buffers: %d/%d bytes\n",
                  bufferSize, RX_POOL_LARGE_BUFFER_SIZE);

    try {

        utility::ScopedLock lock(m_rxLock); // halt potential pool traversal
        
        //
        // Deletion is safe even if the buffer is in use elsewhere
        // (BufferStream is reference counted.)

        BufferPool::const_iterator it;
        for(it  = m_rxLargeBufferPool.begin();
            it != m_rxLargeBufferPool.end();
            ++it)
            delete *it;

        m_rxLargeBufferPool.clear();

        for(uint32_t i=0; i<buffers.size(); i++)
            m_rxLargeBufferPool.push_back(new utility::BufferStreamWriter(buffers[i], bufferSize));

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }

    return Status_Ok;
}

}}}; // namespaces
