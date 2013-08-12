#ifndef HAND_PACKETS_H
#define HAND_PACKETS_H

#include <stdint.h>

// this file is compiled into both the C++ driver library and the C firmware
static const uint32_t CMD_ID_SET_FINGER_POWER_STATE        =  1;
static const uint32_t CMD_ID_SET_FINGER_CONTROL_MODE       =  2;
static const uint32_t CMD_ID_SET_FINGER_JOINT_POS          =  3;
static const uint32_t CMD_ID_CONFIGURE_CAMERA_STREAM       =  4;
static const uint32_t CMD_ID_FINGER_RAW_TX                 =  5; // pass-through
static const uint32_t CMD_ID_SET_MOBO_STATUS_RATE          =  6;
static const uint32_t CMD_ID_MOBO_STATUS                   =  7;
static const uint32_t CMD_ID_SET_FINGER_AUTOPOLL           =  8;
static const uint32_t CMD_ID_SET_ALL_FINGER_POWER_STATES   =  9;
static const uint32_t CMD_ID_ENABLE_LOWVOLT_REGULATOR      = 10;
static const uint32_t CMD_ID_READ_FPGA_FLASH_PAGE          = 11;
static const uint32_t CMD_ID_FPGA_FLASH_PAGE               = 12;
static const uint32_t CMD_ID_FPGA_FLASH_ERASE_SECTOR       = 13;
static const uint32_t CMD_ID_FPGA_FLASH_ERASE_SECTOR_ACK   = 14;
static const uint32_t CMD_ID_BL_MOBO_MCU_FLASH_PAGE        = 15;
static const uint32_t CMD_ID_MOBO_BOOT_CTRL                = 16;
static const uint32_t CMD_ID_MOBO_PING                     = 17;
static const uint32_t CMD_ID_HAND_JOINT_COMMANDS           = 18;
static const uint32_t CMD_ID_MOBO_SET_CURRENT_LIMIT        = 19;
static const uint32_t CMD_ID_HAND_RELATIVE_JOINT_COMMANDS  = 20;
static const uint32_t CMD_ID_MOBO_GET_HW_VERSION           = 21;
static const uint32_t CMD_ID_MOBO_SET_DEST_PORT            = 22;

typedef struct 
{
  uint8_t finger_idx;
  uint8_t finger_power_state;
} __attribute__((packed)) set_finger_power_state_t;
static const uint8_t FINGER_POWER_STATE_OFF  = 0;
static const uint8_t FINGER_POWER_STATE_LOW  = 1;
static const uint8_t FINGER_POWER_STATE_FULL = 2;

typedef struct
{
  uint8_t fps[4];
} __attribute__((packed)) set_all_finger_power_states_t;

typedef struct
{
  uint8_t finger_idx;
  uint8_t finger_control_mode;
} __attribute__((packed)) set_finger_control_mode_t;
static const uint8_t FINGER_CONTROL_MODE_IDLE      = 0;
static const uint8_t FINGER_CONTROL_MODE_JOINT_POS = 1;

typedef struct
{
  uint8_t finger_idx;
  uint8_t pad_0; // pad 3 bytes so the floats are aligned
  uint8_t pad_1;
  uint8_t pad_2;
  float joint_0_radians;
  float joint_1_radians;
  float joint_2_radians;
} __attribute__((packed)) set_finger_joint_pos_t;

typedef struct
{
  uint8_t cam_0_stream;
  uint8_t cam_1_stream;
} __attribute__((packed)) configure_camera_stream_t;
static const uint8_t CAMERA_STREAM_OFF = 0;
static const uint8_t CAMERA_STREAM_ON  = 1;

#define FINGER_RAW_TX_MAX_LEN 500
typedef struct
{
  uint8_t finger_idx;
  uint8_t pad;
  uint16_t tx_data_len;
  uint8_t tx_data[FINGER_RAW_TX_MAX_LEN];
} __attribute__((packed)) finger_raw_tx_t;

typedef struct
{
  uint8_t mobo_state_hz;
} __attribute__((packed)) set_mobo_state_rate_t;

typedef struct
{
  uint32_t mobo_time_ms;
  float finger_currents[4];
  float logic_currents[3];
  uint16_t mobo_raw_temperatures[3];
  uint8_t mobo_max_effort;
} __attribute__((packed)) mobo_state_t;

typedef struct
{
  uint16_t finger_autopoll_hz;
} __attribute__((packed)) set_finger_autopoll_t;

typedef struct
{
  uint8_t enable;
} __attribute__((packed)) enable_lowvolt_regulator_t;

typedef struct
{
  uint32_t page_num;
} __attribute__((packed)) read_fpga_flash_page_t;

#define FPGA_FLASH_PAGE_SIZE 256
typedef struct
{
  uint32_t page_num;
  uint32_t page_status;
  uint8_t page_data[FPGA_FLASH_PAGE_SIZE];
} __attribute__((packed)) fpga_flash_page_t;
static const uint8_t FPGA_FLASH_PAGE_STATUS_READ      = 1;
static const uint8_t FPGA_FLASH_PAGE_STATUS_WRITE_REQ = 2;
static const uint8_t FPGA_FLASH_PAGE_STATUS_WRITE_ACK = 3;

typedef struct
{
  uint32_t sector_page_num; // erases the _entire_ sector holding this page!
} __attribute__((packed)) fpga_flash_erase_sector_t;

typedef struct
{
  uint32_t sector_page_num; 
} __attribute__((packed)) fpga_flash_erase_sector_ack_t;
#define MOBO_MCU_FLASH_PAGE_SIZE 256
typedef struct
{
  uint32_t page_num;
  uint32_t page_status;
  uint8_t page_data[MOBO_MCU_FLASH_PAGE_SIZE];
} __attribute__((packed)) mobo_mcu_flash_page_t;
static const uint8_t MOBO_MCU_FLASH_PAGE_STATUS_READ_REQ  = 0;
static const uint8_t MOBO_MCU_FLASH_PAGE_STATUS_READ_RES  = 1;
static const uint8_t MOBO_MCU_FLASH_PAGE_STATUS_WRITE_REQ = 2;
static const uint8_t MOBO_MCU_FLASH_PAGE_STATUS_WRITE_RES = 3;

typedef struct
{
  uint32_t boot_cmd;
} __attribute__((packed)) mobo_boot_ctrl_t;
static const uint32_t MOBO_BOOT_CTRL_RESET_REQUEST  = 0;
static const uint32_t MOBO_BOOT_CTRL_RESET_RESPONSE = 1;
static const uint32_t MOBO_BOOT_CTRL_BL_AUTOBOOT_HALT_REQUEST  = 2;
static const uint32_t MOBO_BOOT_CTRL_BL_AUTOBOOT_HALT_RESPONSE = 3;
static const uint32_t MOBO_BOOT_CTRL_BL_BOOT_REQUEST  = 4;
static const uint32_t MOBO_BOOT_CTRL_BL_BOOT_RESPONSE = 5;

typedef struct
{
  uint32_t state;
} __attribute__((packed)) mobo_ping_t;
static const uint32_t MOBO_PING_REQUEST  = 0;
static const uint32_t MOBO_PING_RESPONSE = 1;

typedef struct
{
  float   joint_angles[12];
  uint8_t max_efforts [12];
} __attribute__((packed)) hand_joint_commands_t;

typedef struct
{
  uint8_t pkt_state;
  float   current_limit;
} __attribute__((packed)) set_mobo_current_limit_t;
static const uint8_t MOBO_CURRENT_LIMIT_STATE_REQUEST  = 0;
static const uint8_t MOBO_CURRENT_LIMIT_STATE_RESPONSE = 1;

typedef struct
{
  float   relative_joint_angles[12];
  uint8_t max_efforts [12];
} __attribute__((packed)) relative_joint_commands_t;

typedef struct
{
  uint8_t pkt_state;
  uint32_t version;
} __attribute__((packed)) get_hw_version_t;
static const uint8_t MOBO_GET_HW_VERSION_REQUEST  = 0;
static const uint8_t MOBO_GET_HW_VERSION_RESPONSE = 1;

typedef struct
{
  uint8_t pkt_state;
  uint16_t port;
} __attribute__((packed)) set_dest_port_t;
static const uint8_t MOBO_SET_DEST_PORT_REQUEST  = 0;
static const uint8_t MOBO_SET_DEST_PORT_RESPONSE = 1;

#endif

