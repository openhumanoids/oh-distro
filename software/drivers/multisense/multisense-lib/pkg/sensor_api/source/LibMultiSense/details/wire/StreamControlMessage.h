/**
 * @file LibMultiSense/StreamControlMessage.h
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

#ifndef LibMultiSense_StreamControlMessage
#define LibMultiSense_StreamControlMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class StreamControl {
public:
    static const IdType      ID      = ID_CMD_STREAM_CONTROL;
    static const VersionType VERSION = 1;

    //
    // Set modify mask bit high to have the device
    // accept the corresponding bit in the control mask.

    SourceType modifyMask;
    SourceType controlMask;

    //
    // Convenience functions

    void enable(SourceType mask) {
        modifyMask = controlMask = mask;
    };
    void disable(SourceType mask) {
        modifyMask = mask; controlMask = 0;
    };

    //
    // Constructor

    StreamControl(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    StreamControl()
        : modifyMask(0), controlMask(0) {};

    //
    // Serialization routine.

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & modifyMask;
        message & controlMask;
    }
};

}}}}; // namespaces

#endif
