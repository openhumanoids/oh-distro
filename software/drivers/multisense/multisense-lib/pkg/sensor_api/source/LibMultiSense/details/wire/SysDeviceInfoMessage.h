/**
 * @file LibMultiSense/SysDeviceInfoMessage.h
 *
 * This message contains general device information
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
 *   2013-05-08, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysDeviceInfoMessage
#define LibMultiSense_SysDeviceInfoMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class PcbInfo {
public:
    static const VersionType VERSION = 1;

    std::string name;
    uint32_t    revision;

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & name;
        message & revision;
    };
};

class SysDeviceInfo {
public:
    static const IdType      ID      = ID_DATA_SYS_DEVICE_INFO;
    static const VersionType VERSION = 1;

    //
    // These constants are stored in flash on the device, do
    // not change these, only add.
    //
    // crl::multisense::DeviceInfo:: has similar constants
    // that can be changed at will (just remember to 
    // map any differences when translating between
    // WIRE and API.)

    static const uint8_t  MAX_PCBS                   = 8;
    static const uint32_t HARDWARE_REV_MULTISENSE_SL = 1;
    static const uint32_t HARDWARE_REV_MULTISENSE_S  = 2;
    static const uint32_t HARDWARE_REV_MULTISENSE_M  = 3;

    static const uint32_t IMAGER_TYPE_CMV2000_GREY   = 1;
    static const uint32_t IMAGER_TYPE_CMV2000_COLOR  = 2;
    static const uint32_t IMAGER_TYPE_CMV4000_GREY   = 3;
    static const uint32_t IMAGER_TYPE_CMV4000_COLOR  = 4;

    std::string key;
    std::string name;
    std::string buildDate;
    std::string serialNumber;
    uint32_t    hardwareRevision;

    uint8_t     numberOfPcbs;
    PcbInfo     pcbs[MAX_PCBS];
    
    std::string imagerName;
    uint32_t    imagerType;
    uint32_t    imagerWidth;
    uint32_t    imagerHeight;
    
    std::string lensName;
    uint32_t    lensType;
    float       nominalBaseline;          // meters
    float       nominalFocalLength;       // meters
    float       nominalRelativeAperture;  // f-stop

    uint32_t    lightingType;
    uint32_t    numberOfLights;

    std::string laserName;
    uint32_t    laserType;

    std::string motorName;
    uint32_t    motorType;
    float       motorGearReduction;

    //
    // Constructors

    SysDeviceInfo(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysDeviceInfo() :
        hardwareRevision(0),
        imagerType(0),
        imagerWidth(0),
        imagerHeight(0),
        lensType(0),
        nominalBaseline(0),
        nominalFocalLength(0),
        nominalRelativeAperture(0.0),
        lightingType(0),
        numberOfLights(0),
        laserType(0),
        motorType(0),
        motorGearReduction(0.0) {};

    //
    // Serialization routine
    
    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & key;
        message & name;
        message & buildDate;
        message & serialNumber;
        message & hardwareRevision;

        message & numberOfPcbs;

        uint8_t num = std::min(numberOfPcbs, (uint8_t) MAX_PCBS);
        for(uint8_t i=0; i<num; i++)
            pcbs[i].serialize(message, version);

        message & imagerName;
        message & imagerType;
        message & imagerWidth;
        message & imagerHeight;
        message & lensName;
        message & lensType;
        message & nominalBaseline;
        message & nominalFocalLength;
        message & nominalRelativeAperture;
        message & lightingType;
        message & numberOfLights;
        message & laserName;
        message & laserType;
        message & motorName;
        message & motorType;
        message & motorGearReduction;
    }
};

}}}}; // namespaces

#endif
