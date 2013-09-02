/**
 * @file LibMultiSense/details/wire/Protocol.h
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

#ifndef LibMultiSense_details_wire_protocol
#define LibMultiSense_details_wire_protocol

namespace crl {
namespace multisense {
namespace details {
namespace wire {

//
// Some message headers are directly used by sensor firmware

#ifdef SENSORPOD_FIRMWARE
#define WIRE_HEADER_ATTRIBS_ __attribute__ ((__packed__))
#else
#define WIRE_HEADER_ATTRIBS_
#endif // SENSORPOD_FIRMWARE

//
// The size of the combined headers

static const uint8_t COMBINED_HEADER_LENGTH = 60;

//
// The magic number and version

static const uint16_t HEADER_MAGIC   = 0xadad;
static const uint16_t HEADER_VERSION = 0x0100;

//
// The protocol group (TODO: define CRL-wide)

static const uint16_t HEADER_GROUP   = 0x0001;

//
// The packet header structure

typedef struct __attribute__ ((__packed__)) {
    
    //
    // The magic number

    uint16_t magic;

    //
    // The protocol version

    uint16_t version;

    //
    // The protocol group

    uint16_t group;

    //
    // Protocol flags

    uint16_t flags;

    //
    // The message sequence identifier

    uint16_t sequenceIdentifier;

    //
    // The total size of the message

    uint32_t messageLength;

    //
    // Offset of this packet's payload

    uint32_t byteOffset;

} Header;

//
// Types for message IDs and versions

typedef uint16_t IdType;
typedef uint16_t VersionType;

//
// Every command responsds with an ID_ACK message, 
// regardless if a data message is also following.

//
// TODO: this message set is still awkward in places:
//       - Missing 1:1 get/set 
//       - Some "Data" messages are also commands
//       - Duplicated information (CAM_GET_CONFIG, SYS_GET_CAMERA_CAL, etc.)

//
// [N]acks

static const IdType ID_ACK                        = 0x0001;

//
// Commands 

static const IdType ID_CMD_GET_VERSION            = 0x0002;
static const IdType ID_CMD_GET_STATUS             = 0x0003;

static const IdType ID_CMD_CAM_GET_CONFIG         = 0x0004;
static const IdType ID_CMD_CAM_CONTROL            = 0x0007;
static const IdType ID_CMD_CAM_GET_HISTORY        = 0x0008;
static const IdType ID_CMD_CAM_SET_HDR            = 0x000b;
static const IdType ID_CMD_CAM_SET_RESOLUTION     = 0x000c;

static const IdType ID_CMD_LIDAR_GET_CONFIG       = 0x000d;
static const IdType ID_CMD_LIDAR_SET_MOTOR        = 0x0010;

static const IdType ID_CMD_LED_GET_STATUS         = 0x0012;
static const IdType ID_CMD_LED_SET                = 0x0013;

static const IdType ID_CMD_SYS_MTU                = 0x0014;
static const IdType ID_CMD_SYS_FLASH_OP           = 0x0015;
static const IdType ID_CMD_SYS_SET_NETWORK        = 0x0016;
static const IdType ID_CMD_SYS_GET_DEVICE_INFO    = 0x0017;
static const IdType ID_CMD_SYS_GET_CAMERA_CAL     = 0x0018;
static const IdType ID_CMD_SYS_GET_LIDAR_CAL      = 0x0019;

static const IdType ID_CMD_SYS_GET_MTU            = 0x001a;
static const IdType ID_CMD_SYS_GET_NETWORK        = 0x001b;

static const IdType ID_CMD_STREAM_CONTROL         = 0x001c;

static const IdType ID_CMD_SYS_GET_DEVICE_MODES   = 0x001d;

//
// Data 

static const IdType ID_DATA_VERSION               = 0x0102;
static const IdType ID_DATA_STATUS                = 0x0103;

static const IdType ID_DATA_CAM_CONFIG            = 0x0104;
static const IdType ID_DATA_CAM_HISTORY           = 0x0105;

static const IdType ID_DATA_LIDAR_CONFIG          = 0x0108;
static const IdType ID_DATA_LIDAR_SCAN            = 0x0109;

static const IdType ID_DATA_LED_STATUS            = 0x010a;

static const IdType ID_DATA_SYS_FLASH_RESPONSE    = 0x010b;
static const IdType ID_DATA_SYS_DEVICE_INFO       = 0x010c;
static const IdType ID_DATA_SYS_CAMERA_CAL        = 0x010d;
static const IdType ID_DATA_SYS_LIDAR_CAL         = 0x010e;

static const IdType ID_DATA_IMAGE_META            = 0x010f;
static const IdType ID_DATA_IMAGE                 = 0x0110;
static const IdType ID_DATA_DISPARITY             = 0x0111;

static const IdType ID_DATA_SYS_DEVICE_MODES      = 0x0112;

//
// Data sources

typedef uint32_t SourceType;

static const SourceType SOURCE_UNKNOWN           = 0;
static const SourceType SOURCE_RAW_LEFT          = (1<<0);
static const SourceType SOURCE_RAW_RIGHT         = (1<<1);
static const SourceType SOURCE_LUMA_LEFT         = (1<<2);
static const SourceType SOURCE_LUMA_RIGHT        = (1<<3);
static const SourceType SOURCE_LUMA_RECT_LEFT    = (1<<4);
static const SourceType SOURCE_LUMA_RECT_RIGHT   = (1<<5);
static const SourceType SOURCE_CHROMA_LEFT       = (1<<6);
static const SourceType SOURCE_CHROMA_RIGHT      = (1<<7);
static const SourceType SOURCE_DISPARITY         = (1<<10);
static const SourceType SOURCE_LIDAR_SCAN        = (1<<24);

static const SourceType SOURCE_IMAGES            = 0x04FF;

//
// Some helper macros

#define MSG_ID(x)  ((wire::IdType)(x))
#define MSG_VER(x) ((wire::VersionType)(x))

#define SER_ARRAY_1(a_,n_)                    \
    for(uint32_t i_=0; i_<(n_); i_++)         \
        message & (a_)[i_];                   \
    
#define SER_ARRAY_2(a_,n_,m_)                 \
    for(uint32_t i_=0; i_<(n_); i_++)         \
        for(uint32_t j_=0; j_<(m_); j_++)     \
            message & (a_)[(i_)][(j_)];       \

#define CPY_ARRAY_1(d_,s_,n_)                   \
    for(uint32_t i_=0; i_<(n_); i_++)           \
        (d_)[i_] = (s_)[i_];                    \

#define CPY_ARRAY_2(d_,s_,n_,m_)                \
    for(uint32_t i_=0; i_<(n_); i_++)           \
        for(uint32_t j_=0; j_<(m_); j_++)       \
            (d_)[i_][j_] = (s_)[i_][j_];        \

}}}}; // namespaces

#endif
