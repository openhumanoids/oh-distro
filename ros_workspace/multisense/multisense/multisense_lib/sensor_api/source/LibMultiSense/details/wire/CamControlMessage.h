/**
 * @file LibMultiSense/CamControlMessage.h
 *
 * This message contains the current camera configuration.
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
 *   2012-04-14, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_CamControlMessage
#define LibMultiSense_CamControlMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class CamControl {
public:
    static const IdType      ID      = ID_CMD_CAM_CONTROL;
    static const VersionType VERSION = 1;

    //
    // Parameters representing the current camera configuration

    float    framesPerSecond;
    float    gain;
    
    uint32_t exposure;
    uint8_t  autoExposure;
    uint32_t autoExposureMax;
    uint32_t autoExposureDecay;
    float    autoExposureThresh;

    float    whiteBalanceRed;
    float    whiteBalanceBlue;
    uint8_t  autoWhiteBalance;
    uint32_t autoWhiteBalanceDecay;
    float    autoWhiteBalanceThresh;

    //
    // Constructors

    CamControl(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    CamControl() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & framesPerSecond;
        message & gain;

        message & exposure;
        message & autoExposure;
        message & autoExposureMax;
        message & autoExposureDecay;
        message & autoExposureThresh;

        message & whiteBalanceRed;
        message & whiteBalanceBlue;
        message & autoWhiteBalance;
        message & autoWhiteBalanceDecay;
        message & autoWhiteBalanceThresh;
    }
};

}}}}; // namespaces

#endif
