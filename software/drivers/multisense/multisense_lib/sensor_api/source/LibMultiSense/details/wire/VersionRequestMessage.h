/**
 * @file LibMultiSense/VersionRequestMessage.h
 *
 * This message contains a request for versioning information.
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
 *   2012-04-11, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_VersionRequestMessage
#define LibMultiSense_VersionRequestMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class VersionRequest {
public:
    static const IdType      ID      = ID_CMD_GET_VERSION;
    static const VersionType VERSION = 1;

    //
    // Constructors

    VersionRequest(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    VersionRequest() {};

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
