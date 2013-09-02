/**
 * @file LibMultiSense/VersionResponseMessage.h
 *
 * This message contains versioning information.
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

#ifndef LibMultiSense_VersionResponseMessage
#define LibMultiSense_VersionResponseMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class VersionResponse {
public:
    static const IdType      ID      = ID_DATA_VERSION;
    static const VersionType VERSION = 1;

    std::string firmwareBuildDate;
    VersionType firmwareVersion;
    uint64_t    hardwareVersion;
    uint64_t    hardwareMagic;
    uint64_t    fpgaDna;

    //
    // Constructors

    VersionResponse(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    VersionResponse() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & firmwareBuildDate;
        message & firmwareVersion;
        message & hardwareVersion;
        message & hardwareMagic;
        message & fpgaDna;
    }
};

}}}}; // namespaces

#endif
