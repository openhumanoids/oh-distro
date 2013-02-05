/**
 * @file LibSensorPodCommunications/MessageIdentifiers.h
 *
 * Defines all of the message identifiers we support.
 *
 * Copyright 2012
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2012-04-11, dtascione@carnegierobotics.com, RD1020, Created file.
 **/

#ifndef LibSensorPodCommunications_MessageIdentifiers
#define LibSensorPodCommunications_MessageIdentifiers

//
// Status message types.
//

#define CP_VERSION_REQUEST             0x80
#define SP_VERSION_RESPONSE            0x00

#define CP_STATUS_REQUEST              0x81
#define SP_STATUS_RESPONSE             0x01

//
// Camera message types.
//

#define CP_CAM_GET_CONFIG              0x10
#define SP_CAM_CONFIG                  0x90

#define CP_CAM_START_STREAM            0x11
#define SP_CAM_START_STREAM_ACK        0x91

#define CP_CAM_STOP_STREAM             0x12
#define SP_CAM_STOP_STREAM_ACK         0x92

#define CP_CAM_CONTROL                 0x13
#define SP_CAM_CONTROL_ACK             0x93

#define CP_CAM_GET_HISTORY             0x14
#define SP_CAM_HISTORY                 0x94

#define CP_CAM_START_IMAGE_STREAM      0x15
#define SP_CAM_START_IMAGE_STREAM_ACK  0x95

#define CP_CAM_STOP_IMAGE_STREAM       0x16
#define SP_CAM_STOP_IMAGE_STREAM_ACK   0x96

#define SP_CAM_DATA                    0x98
#define SP_CAM_IMAGE_DATA              0x99

#define CP_CAM_SET_HDR                 0x1A
#define SP_CAM_SET_HDR_ACK             0x9A

#define CP_CAM_SET_RESOLUTION          0x1B
#define SP_CAM_SET_RESOLUTION_ACK      0x9B

//
// LIDAR messages.
//

#define CP_LIDAR_GET_CONFIG            0x20
#define SP_LIDAR_CONFIG                0xa0

#define CP_LIDAR_START_SCAN            0x21
#define SP_LIDAR_START_SCAN_ACK        0xa1

#define CP_LIDAR_STOP_SCAN             0x22
#define SP_LIDAR_STOP_SCAN_ACK         0xa2

#define CP_LIDAR_SET_MOTOR             0x23
#define SP_LIDAR_MOTOR_ACK             0xa3

#define CP_LIDAR_HOME                  0x24
#define SP_LIDAR_HOME_ACK              0xa4

#define SP_LIDAR_DATA                  0xa8

//
// LED messages.
//

#define CP_LED_GET_STATUS              0x30
#define SP_LED_STATUS                  0xb0

#define CP_LED_SET                     0x31
#define SP_LED_SET_ACK                 0xb1

//
// System messages.
//

#define CP_SYS_SET_MTU                 0x70
#define SP_SYS_SET_MTU_ACK             0xF0

#define CP_SYS_FLASH_OP                0x71
#define SP_SYS_FLASH_OP_ACK            0xF1

#endif
