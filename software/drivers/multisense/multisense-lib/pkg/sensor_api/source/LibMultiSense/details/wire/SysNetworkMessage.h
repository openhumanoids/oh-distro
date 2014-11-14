/**
 * @file LibMultiSense/SysNetworkMessage.h
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

#ifndef LibMultiSense_SysNetworkMessage
#define LibMultiSense_SysNetworkMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysNetwork {
public:
    static const IdType      ID      = ID_CMD_SYS_SET_NETWORK;
    static const VersionType VERSION = 1;

    //
    // Configurable interfaces

    static const uint8_t Interface_Unknown   = 0;
    static const uint8_t Interface_Primary   = 1;  // external GigE
    static const uint8_t Interface_Secondary = 2;  // internal 100Mb

    //
    // IPV4 parameters

    uint8_t     interface;
    std::string address;
    std::string gateway;
    std::string netmask;

    //
    // Constructors

    SysNetwork(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysNetwork(const std::string& a,
               const std::string& g,
               const std::string& n) : 
        interface(Interface_Primary),
        address(a),
        gateway(g),
        netmask(n) {};
    SysNetwork() :
        interface(Interface_Unknown),
        address(),
        gateway(),
        netmask() {};
        
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & interface;
        message & address;
        message & gateway;
        message & netmask;
    }
};

}}}}; // namespaces

#endif
