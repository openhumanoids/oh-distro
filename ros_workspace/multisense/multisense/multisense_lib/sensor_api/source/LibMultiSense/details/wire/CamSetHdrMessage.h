/**
 * @file LibMultiSense/CamSetHdrMessage.h
 *
 * This message sets HDR parameters on the image sensor.
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
 *   2012-10-03, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_CamSetHdrMessage
#define LibMultiSense_CamSetHdrMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class CamSetHdr {
public:
    static const IdType      ID      = ID_CMD_CAM_SET_HDR;
    static const VersionType VERSION = 1;

    //
    // Parameters

    // TODO: Currently specific to CMV2000; abstract it

    float   exp_kp2;    // relative to overall exposure length (0.0-1.0)
    float   exp_kp1;    // ""
    uint8_t vlow3;      // 64~128
    uint8_t vlow2;      // ""
    uint8_t nr_slopes;  // 1-3

    //
    // Constructors

    CamSetHdr(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    CamSetHdr() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & exp_kp2;
        message & exp_kp1;
        message & vlow3;
        message & vlow2;
        message & nr_slopes;
    }
};

}}}}; // namespaces

#endif
