/**
 * @file LibMultiSense/StatusResponseMessage.h
 *
 * This message contains status information.
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
 *   2013-05-08, ekratzer@carnegierobotics.com, PR1044, Significant rewrite.
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_StatusResponseMessage
#define LibMultiSense_StatusResponseMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class StatusResponse {
public:
    static const IdType      ID      = ID_DATA_STATUS;
    static const VersionType VERSION = 1;

    //
    // Subsytem status

    static const uint32_t STATUS_GENERAL_OK     = (1<<0);
    static const uint32_t STATUS_LASER_OK       = (1<<1);
    static const uint32_t STATUS_LASER_MOTOR_OK = (1<<2);
    static const uint32_t STATUS_CAMERAS_OK     = (1<<3);

    //
    // The reported uptime for the system
    //

    utility::TimeStamp uptime;
    uint32_t           status;
    float              temperature0; // celsius
    float              temperature1;

    //
    // Constructors

    StatusResponse(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    StatusResponse() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & uptime;
        message & status;
        message & temperature0;
        message & temperature1;
    }
};

}}}}; // namespaces

#endif
