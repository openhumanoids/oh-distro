/**
 * @file LibMultiSense/CamGetHistoryMessage.h
 *
 * This message contains a request for a history of what camera frames
 * were sent recently.
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

#ifndef LibMultiSense_CamGetHistoryMessage
#define LibMultiSense_CamGetHistoryMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class CamGetHistory {
public:
    static const IdType      ID      = ID_CMD_CAM_GET_HISTORY;
    static const VersionType VERSION = 1;

    //
    // Constructors

    CamGetHistory(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    CamGetHistory() {};

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
