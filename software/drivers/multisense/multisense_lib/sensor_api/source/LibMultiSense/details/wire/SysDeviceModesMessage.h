/**
 * @file LibMultiSense/SysDeviceModesMessage.h
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
 *   2013-06-17, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysDeviceModesMessage
#define LibMultiSense_SysDeviceModesMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class DeviceMode {
public:
    uint32_t width;
    uint32_t height;
    uint32_t supportedDataSources;
    uint32_t disparities;

    DeviceMode(uint32_t w=0,
               uint32_t h=0,
               uint32_t s=0,
               uint32_t d=0) : 
        width(w),
        height(h),
        supportedDataSources(s),
        disparities(d) {};
};

class SysDeviceModes {
public:
    static const IdType      ID      = ID_DATA_SYS_DEVICE_MODES;
    static const VersionType VERSION = 2;

    //
    // Available formats

    std::vector<DeviceMode> modes;

    //
    // Constructors

    SysDeviceModes(utility::BufferStreamReader& r, 
                   VersionType                  v) {serialize(r,v);};
    SysDeviceModes() {};
        
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        uint32_t length = modes.size();
        message & length;
        modes.resize(length);

        //
        // Serialize by hand here to maintain backwards compatibility with
        // pre-v2.3 firmware.

        for(uint32_t i=0; i<length; i++) {

            DeviceMode& m = modes[i];

            message & m.width;
            message & m.height;
            message & m.supportedDataSources;
            message & m.disparities; // was 'flags' in pre v2.3
        }
    }
};

}}}}; // namespaces

#endif
