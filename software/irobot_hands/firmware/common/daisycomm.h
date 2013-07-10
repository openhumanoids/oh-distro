/****************************************************

 // Author:            Zachary Clifford
 // File Name:        C1482-SRC-COMMON-0-daisycomm.c
 // Creation Date:    22 February, 2012
 // Revision:        00
 // Hardware:        ATxmega32A4U
 // Description:    Common routines for daisy chaining

****************************************************/

/******************************************************************************
    File Revision History:
-------------------------------------------------------------------------------
Revision    Date        Engineer    Description
--------    --------    --------    -------------------------------------------
00            MM/DD/YY    ZAC            Initial Release
-------------------------------------------------------------------------------

******************************************************************************/

//
// NOTE: see "Firmware/palm/router.h" for similar defines for palm board,
// and "Software/handle_lib/handleTypes.hpp" for Overo.
//

#ifndef C1482_SRC_COMMON_0_DAISYCOMM_H_
#define C1482_SRC_COMMON_0_DAISYCOMM_H_

#define MAX_PACKET_SIZE 200
#define MIN_PACKET_SIZE 3
#define COMMAND_PACKET_SIZE 7

#define RESPONSE_PACKETSIZE_OFFSET 0
#define RESPONSE_REFLECTEDOPCODE_OFFSET 1
#define RESPONSE_STATUSCODE_OFFSET 2
#define RESPONSE_PAYLOAD_OFFSET 3

#define STATUS_OK 0x00
#define CHECKSUM_ERROR 0x01
#define UNKNOWN_COMMAND 0x02
#define BAD_ADDRESS 0x03
#define TIMEOUT_ERROR 0x04
#define DELAYED_ERROR 0x80
#define MANDATORY_COOLDOWN 0x81
#define OUT_OF_RANGE 0x82

#define DESTINATION_HEADER_OFFSET 0
#define COMMAND_OFFSET 1
#define PAYLOAD_OFFSET 2
#define CHECKSUM_OFFSET 6

#define DESTINATION_BROADCAST 0xFF

#define CHAINADDRESS_BITMASK 0xF0
#define PALM_CHAINADDRESS 0x00
#define FINGER1_CHAINADDRESS 0x10
#define FINGER2_CHAINADDRESS 0x20
#define FINGER3_CHAINADDRESS 0x30
#define MOTOR1_CHAINADDRESS 0x40
#define MOTOR2_CHAINADDRESS 0x50
#define TACTILE_CHAINADDRESS 0x60

#define CHAININDEX_BITMASK 0x0F

#define OPCODE_BITMASK 0xF0

#define DATA_COLLECTION_OPCODE 0x00
#define SET_SAMPLE_PERIOD_OPCODE 0x10
#define SET_SAMPLE_ARGS_OPCODE 0x20
#define START_COLLECTION_OPCODE 0x30
#define STOP_COLLECTION_OPCODE 0x40
#define FINGER_COMMAND_OPCODE 0x50
#define MOTOR_PARAMETER_RE_L_OPCODE 0x60
#define MOTOR_PARAMETER_RE_H_OPCODE 0x70
#define MOTOR_PARAMETER_WR_L_OPCODE 0x80
#define MOTOR_PARAMETER_WR_H_OPCODE 0x90
#define MOTOR_COMMAND_OPCODE 0xA0
#define BOOTLOADER_OPCODE 0xB0
#define CALIBRATION_OPCODE 0xC0

#define PARAMETER_ADDRESS_BITMASK 0x1F

#define DATA_COLLECTION_ACCELERATION_BITMASK     0x8000
#define DATA_COLLECTION_DYNAMIC_BITMASK          0x4000
#define DATA_COLLECTION_TENSION_BITMASK          0x2000
#define DATA_COLLECTION_DISTALJOINT_BITMASK      0x1000
#define DATA_COLLECTION_PROXIMALJOINT_BITMASK    0x0800
#define DATA_COLLECTION_FINGERROTATION_BITMASK   0x0400
#define DATA_COLLECTION_TACTILE_BITMASK          0x0200
#define DATA_COLLECTION_MOTORCURRENT_BITMASK     0x0100
#define DATA_COLLECTION_MOTORSTATORTEMP_BITMASK  0x0080
#define DATA_COLLECTION_MOTORVELOCITY_BITMASK    0x0040
#define DATA_COLLECTION_EXTERNALSUPPLY_BITMASK   0x0020
#define DATA_COLLECTION_AIRTEMPERATURE_BITMASK   0x0010
#define DATA_COLLECTION_MOTORWINDINGTEMP_BITMASK 0x0008
#define DATA_COLLECTION_MOTORHALL_BITMASK        0x0004
#define DATA_COLLECTION_DEBUG_BITMASK            0x0002
#define DATA_COLLECTION_TACTILE_TEMP_BITMASK     0x0001

#define MOTOR_COMMAND_DIRECTION_BITMASK 0x0C
#define MOTOR_COMMAND_STOP 0x00
#define MOTOR_COMMAND_REVERSE 0x04
#define MOTOR_COMMAND_FORWARD 0x08

#define MOTOR_COMMAND_SCHEME_BITMASK 0x03
#define MOTOR_COMMAND_POSITION       0x00
#define MOTOR_COMMAND_CURRENT        0x01
#define MOTOR_COMMAND_VOLTAGE        0x02
#define MOTOR_COMMAND_VELOCITY       0x03

#define EEPROM_ADDRESS_ENCODER_OFFSET   28
#define EEPROM_ADDRESS_FIRMWARE_VERSION 29
#define EEPROM_ADDRESS_ID               30
#define EEPROM_ADDRESS_LED              31


extern volatile uint8_t notifyDaisy;
extern uint16_t RxCheckSumErrCnt[2];


int computeChecksum(uint8_t *packetBuffer, int packetSize);
void configureDaisyUSART(void);
void doDaisyTask(void);
void handleTC(void);

int processCommand(uint8_t *commandPacket,uint8_t *outputBuffer);

#endif /* C1482_SRC_COMMON_0_DAISYCOMM_H_ */
