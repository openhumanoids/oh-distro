/**
 * @file LibMultiSense/CamSetResolutionMessage.h
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
 *   2013-05-08, ekratzer@carnegierobotics.com, PR1044, Significant rewrite.
 *   2012-10-19, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_CamSetResolutionMessage
#define LibMultiSense_CamSetResolutionMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class CamSetResolution {
public:
    static const IdType      ID      = ID_CMD_CAM_SET_RESOLUTION;
    static const VersionType VERSION = 1;

    //
    // Parameters

    uint32_t width;
    uint32_t height;

    //
    // Constructors

    CamSetResolution(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    CamSetResolution(uint32_t w=0, uint32_t h=0) :
        width(w), height(h) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & width;
        message & height;
    }
};

}}}}; // namespaces

#endif
