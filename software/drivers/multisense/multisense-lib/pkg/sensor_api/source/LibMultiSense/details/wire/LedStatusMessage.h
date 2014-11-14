/**
 * @file LibMultiSense/LedStatusMessage.h
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
 *   2012-07-17, dstrother@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibMultiSense_LedStatusMessage
#define LibMultiSense_LedStatusMessage

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class LedStatus {
public:
    static const IdType      ID      = ID_DATA_LED_STATUS;
    static const VersionType VERSION = 1;

    //
    // Bit mask indicating which LEDs are implemented

    uint8_t available;

    //
    // Current LED duty cycles; 0 = off; 255 = 100%

    uint8_t intensity[lighting::MAX_LIGHTS];

    //
    // If non-zero, LEDs are only on while sensors are exposing

    uint8_t flash;

    //
    // Constructors

    LedStatus(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    LedStatus() : available(0), flash(0) {};

    //
    // Serialization routine
    
    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & available;
        for(uint32_t i=0; i<lighting::MAX_LIGHTS; i++)
            message & intensity[i];
        message & flash;
    }
};

}}}}; // namespaces

#endif
