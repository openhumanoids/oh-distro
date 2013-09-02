/**
 * @file LibMultiSense/LedGetStatusMessage.h
 *
 * This message contains a request for camera configuration.
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
 *   2012-07-17, dstrother@carnegierobotics.com, RD1020, Changed to be LedGetStatus
 *   2012-04-12, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_LedGetStatusMessage
#define LibMultiSense_LedGetStatusMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class LedGetStatus {
public:
    static const IdType      ID      = ID_CMD_LED_GET_STATUS;
    static const VersionType VERSION = 1;

    //
    // Constructors

    LedGetStatus(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    LedGetStatus() {};

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
