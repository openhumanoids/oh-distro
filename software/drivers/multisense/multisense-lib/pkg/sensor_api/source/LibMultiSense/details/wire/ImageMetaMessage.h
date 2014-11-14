/**
 * @file LibMultiSense/ImageMetaMessage.h
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

#ifndef LibMultiSense_ImageMetaMessage
#define LibMultiSense_ImageMetaMessage

#include <typeinfo>

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ ImageMetaHeader {
public:

    static const IdType      ID      = ID_DATA_IMAGE_META;
    static const VersionType VERSION = 1;

    static const uint32_t HISTOGRAM_CHANNELS = 4; // g0, r, b, g1
    static const uint32_t HISTOGRAM_BINS     = 256;
    static const uint32_t HISTOGRAM_LENGTH   = (HISTOGRAM_CHANNELS * HISTOGRAM_BINS *
                                                sizeof(uint32_t));

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    int64_t            frameId;
    float              framesPerSecond;
    float              gain;
    uint32_t           exposureTime;
    uint32_t           timeSeconds;
    uint32_t           timeMicroSeconds;
    int32_t            angle; // microradians

    ImageMetaHeader() 
        :
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        frameId(0),
        framesPerSecond(0),
        gain(0.0),
        exposureTime(0),
        timeSeconds(0),
        timeMicroSeconds(0),    
        angle(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class ImageMeta : public ImageMetaHeader {
public:

    uint32_t histogramP[HISTOGRAM_BINS * HISTOGRAM_CHANNELS];

    //
    // Constructors

    ImageMeta(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    ImageMeta() {};
  
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & frameId;
        message & framesPerSecond;
        message & gain;
        message & exposureTime;
        message & timeSeconds;
        message & timeMicroSeconds;
        message & angle;
        
        if (typeid(Archive) == typeid(utility::BufferStreamWriter))
            message.write(histogramP, HISTOGRAM_LENGTH);
        else
            message.read(histogramP, HISTOGRAM_LENGTH);
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}}; // namespaces

#endif
