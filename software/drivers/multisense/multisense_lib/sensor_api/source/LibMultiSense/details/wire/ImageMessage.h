/**
 * @file LibMultiSense/ImageMessage.h
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
 *   2013-06-12, ekratzer@carnegierobotics.com, PR1044, created file.
 **/

#ifndef LibMultiSense_ImageMessage
#define LibMultiSense_ImageMessage

#include <typeinfo>
#include <cmath>

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ ImageHeader {
public:

static const IdType      ID      = ID_DATA_IMAGE;
static const VersionType VERSION = 1;

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    uint32_t source;
    uint32_t bitsPerPixel;
    int64_t  frameId;
    uint16_t width;
    uint16_t height;

    ImageHeader() 
        : 
#ifdef SENSORDPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        source(0),
        bitsPerPixel(0),
        frameId(0),
        width(0),
        height(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class Image : public ImageHeader {
public:

    void *dataP;

    //
    // Constructors

    Image(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    Image() : dataP(NULL) {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & source;
        message & bitsPerPixel;
        message & frameId;
        message & width;
        message & height;

        const uint32_t imageSize = std::ceil(((double) bitsPerPixel / 8.0) * width * height);

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {
          
            message.write(dataP, imageSize);

        } else {

            dataP = message.peek();
            message.seek(message.tell() + imageSize);
        }
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}}; // namespaces

#endif
