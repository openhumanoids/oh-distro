/**
 * @file LibMultiSense/SysGetNetworkMessage.h
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
 *   2013-06-13, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_SysGetNetworkMessage
#define LibMultiSense_SysGetNetworkMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysGetNetwork {
public:
    static const IdType      ID      = ID_CMD_SYS_GET_NETWORK;
    static const VersionType VERSION = 1;

    //
    // Constructors

    SysGetNetwork(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysGetNetwork() {};

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
