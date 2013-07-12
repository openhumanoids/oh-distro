/**
 * @file LibMultiSense/SysGetDeviceInfoMessage.h
 *
 * This message contains a request for device info
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
 *   2013-05-22, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysGetDeviceInfoMessage
#define LibMultiSense_SysGetDeviceInfoMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysGetDeviceInfo {
public:
    static const IdType      ID      = ID_CMD_SYS_GET_DEVICE_INFO;
    static const VersionType VERSION = 1;

    //
    // Constructors

    SysGetDeviceInfo(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysGetDeviceInfo() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        // nothing yet
    }
};

}}}}; // namespaces

#endif
