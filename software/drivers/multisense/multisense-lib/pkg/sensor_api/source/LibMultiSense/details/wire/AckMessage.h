/**
 * @file LibMultiSense/AckMessage.h
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
 *   2013-05-07, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_AckMessage
#define LibMultiSense_AckMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class Ack {
public:
    static const IdType      ID      = ID_ACK;
    static const VersionType VERSION = 1;

    IdType command; // the command being [n]ack'd
    Status status;

    //
    // Constructors

    Ack(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    Ack(IdType c=0, Status s=Status_Ok) : command(c), status(s) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & command;
        message & status;
    }
};

}}}}; // namespaces

#endif
