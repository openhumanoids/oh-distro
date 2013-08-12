#ifndef STATUS_H
#define STATUS_H

#include <stdint.h>

#define STATUS_IMU_LEN 6
#define PP_NUM_TAXELS  6
#define DP_NUM_TAXELS 12
typedef struct 
{
  uint32_t pp_tactile_time;
  uint32_t dp_tactile_time;
  uint32_t fmcb_time;
  uint16_t pp_tactile[PP_NUM_TAXELS];
  uint16_t dp_tactile[DP_NUM_TAXELS];
  uint16_t pp_imu_data[STATUS_IMU_LEN];
  uint16_t dp_imu_data[STATUS_IMU_LEN];
  uint16_t fmcb_imu_data[STATUS_IMU_LEN];
  uint16_t pp_temp[4];
  uint16_t dp_temp[4];
  uint16_t fmcb_temp[3];
  uint16_t fmcb_voltage;
  uint16_t fmcb_pb_current;
  uint32_t pp_strain;
  int32_t  fmcb_hall_tgt[3];
  int32_t  fmcb_hall_pos[3];
  int16_t  fmcb_effort[3];
} status_t;
extern status_t g_status;
#define STATUS_PAYLOAD_LEN  (sizeof(status_t))

void status_init();

#endif

