/**
 * @file LibMultiSense/CamSetTriggerSourceMessage.h
 *
 * This message sets the output resolution of the camera.
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
 *   2013-07-15, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_CamSetTriggerSourceMessage
#define LibMultiSense_CamSetTriggerSourceMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class CamSetTriggerSource {
public:
    static const IdType      ID      = ID_CMD_CAM_SET_TRIGGER_SOURCE;
    static const VersionType VERSION = 1;

    static const uint32_t    SOURCE_INTERNAL = 0;
    static const uint32_t    SOURCE_EXTERNAL = 1; // OPTO_RX

    //
    // Parameters

    uint32_t source;

    //
    // Constructors

    CamSetTriggerSource(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    CamSetTriggerSource(uint32_t s=SOURCE_INTERNAL) : source(s) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & source;
    }
};

}}}}; // namespaces

#endif
