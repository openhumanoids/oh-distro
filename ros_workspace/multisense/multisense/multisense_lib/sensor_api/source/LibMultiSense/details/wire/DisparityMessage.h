/**
 * @file LibMultiSense/DisparityMessage.h
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

#ifndef LibMultiSense_DisparityMessage
#define LibMultiSense_DisparityMessage

#include <typeinfo>
#include <cmath>

#define DISPARITY_32F

namespace crl {
namespace multisense {
namespace details {
namespace wire {

class WIRE_HEADER_ATTRIBS_ DisparityHeader {
public:

    static const IdType      ID      = ID_DATA_DISPARITY;
    static const VersionType VERSION = 1;

    static const uint8_t  WIRE_BITS_PER_PIXEL = 12;
    static const uint8_t  WIRE_BYTE_ALIGNMENT = 3;
    static const uint8_t  API_BITS_PER_PIXEL  = 16; // after custom assemble()
    static const uint32_t META_LENGTH         = 16; // packed, includes type/version

#ifdef SENSORPOD_FIRMWARE
    IdType      id;
    VersionType version;
#endif // SENSORPOD_FIRMWARE

    int64_t  frameId;
    uint16_t width;
    uint16_t height;

    DisparityHeader() 
        : 
#ifdef SENSORPOD_FIRMWARE
        id(ID),
        version(VERSION),
#endif // SENSORPOD_FIRMWARE
        frameId(0),
        width(0),
        height(0) {};
};

#ifndef SENSORPOD_FIRMWARE

class Disparity : public DisparityHeader {
public:

    void *dataP;

    //
    // Constructors

    Disparity(utility::BufferStreamReader&r, VersionType v) {serialize(r,v);};
    Disparity() : dataP(NULL) {};
  
    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const VersionType version)
    {
        message & frameId;
        message & width;
        message & height;

        const uint32_t imageSize = std::ceil(((double) API_BITS_PER_PIXEL / 8.0) * width * height);

        if (typeid(Archive) == typeid(utility::BufferStreamWriter)) {
          
            message.write(dataP, imageSize);

        } else {

            dataP = message.peek();
            message.seek(message.tell() + imageSize);
        }
    }

    //
    // UDP assembler 

    static void assembler(utility::BufferStreamWriter& stream,
                          const uint8_t               *dataP,
                          uint32_t                     offset,
                          uint32_t                     length)
    {   
        //
        // Special case, 1st packet contains header only. Firmware
        // does not have to worry about the header length being
        // byte-aligned on a WIRE_BITS_PER_PIXEL boundary

        if (0 == offset) {
            stream.seek(0);
            stream.write(dataP, META_LENGTH);
            return;
        }
    
        //
        // The data section of each incoming packet is byte-aligned 
        // on a WIRE_BITS_PER_PIXEL boundary

        const uint8_t *sP           = dataP;
        const uint32_t sourceOffset = offset - META_LENGTH;
        const uint32_t count        = (8 * length) / WIRE_BITS_PER_PIXEL;
        const uint32_t destOffset   = META_LENGTH + (((8 * sourceOffset) / WIRE_BITS_PER_PIXEL) * 
                                                     (API_BITS_PER_PIXEL / 8));
        //
        // Seek to the proper location

        stream.seek(destOffset);
       
        //
        // This conversion is for (WIRE == 12bits), (API == 16bits, 1/16th pixel, unsigned integer)

        if (12 == WIRE_BITS_PER_PIXEL && 16 == API_BITS_PER_PIXEL) {

            uint16_t *dP = reinterpret_cast<uint16_t*>(stream.peek());

            for(uint32_t i=0; i<count; i+=2, sP+=3) {
                dP[i]   = ((sP[0]     ) | ((sP[1] & 0x0F) << 8));
                dP[i+1] = ((sP[1] >> 4) |  (sP[2] << 4)        );
            }

        //
        // This conversion is for (WIRE == 12bits), (API == 32bits, floating point)

        } else if (12 == WIRE_BITS_PER_PIXEL && 32 == API_BITS_PER_PIXEL) {

            float *dP = reinterpret_cast<float*>(stream.peek());
        
            for(uint32_t i=0; i<count; i+=2, sP+=3) {
                
                dP[i]   = (float) ((sP[0]     ) | ((sP[1] & 0x0F) << 8)) / 16.0f;
                dP[i+1] = (float) ((sP[1] >> 4) |  (sP[2] << 4)        ) / 16.0f;
            }

        } else 
            CRL_EXCEPTION("(wire==%dbits, api=%dbits) not supported",
                          WIRE_BITS_PER_PIXEL, API_BITS_PER_PIXEL);
    }
};

#endif // !SENSORPOD_FIRMWARE

}}}}; // namespaces

#endif
