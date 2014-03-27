/**
 * @file LibMultiSense/SysTestMtuResponseMessage.h
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
 *   2013-11-19, ekratzer@carnegierobotics.com, PR1044, Significant rewrite.
 **/

#ifndef LibMultiSense_SysTestMtuResponseMessage
#define LibMultiSense_SysTestMtuResponseMessage

#include <typeinfo>

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class SysTestMtuResponse {
public:
    static const IdType      ID          = ID_DATA_SYS_TEST_MTU_RESPONSE;
    static const VersionType VERSION     = 1;
    static const uint32_t    HEADER_SIZE = sizeof(uint32_t);

    uint32_t payloadSize;

    //
    // Constructors

    SysTestMtuResponse(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    SysTestMtuResponse(uint32_t s=0) : payloadSize(s) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & payloadSize;
        for(uint32_t i=0; i<payloadSize; ++i) {
            uint8_t dummy = 0;
            message & dummy;
        }
    }
};

}}}}; // namespaces

#endif
