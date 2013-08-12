#include <cstdio>
#include "sandia_hand/motor_module.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ros/console.h>
using namespace sandia_hand;
using std::vector;

MotorModule::MotorModule(const uint8_t addr)
: SerialMessageProcessor(addr)
{
  registerRxHandler(PKT_FINGER_STATUS, 
                    boost::bind(&MotorModule::rxFingerState, this, _1, _2));
  registerRxHandler(PKT_PHALANGE_TXRX,
                    boost::bind(&MotorModule::rxPhalangeTxRx, this, _1, _2));
}

MotorModule::~MotorModule()
{
}

bool MotorModule::setPhalangeBusPower(bool on)
{
  getTxBuffer()[0] = on ? 1 : 0;
  if (!sendTxBuffer(PKT_PHALANGE_POWER, 1))
    return false;
  return listenFor(PKT_PHALANGE_POWER, 0.5);
}

bool MotorModule::setPhalangeAutopoll(bool on)
{
  getTxBuffer()[0] = on ? 1 : 0;
  if (!sendTxBuffer(PKT_PHALANGE_AUTOPOLL, 1))
    return false;
  return listenFor(PKT_PHALANGE_AUTOPOLL, 0.5);
}

typedef struct
{
  uint32_t pp_tactile_time;
  uint32_t dp_tactile_time;
  uint32_t fmcb_time;
  uint16_t pp_tactile[6];
  uint16_t dp_tactile[12];
  uint16_t pp_imu[6];
  uint16_t dp_imu[6];
  uint16_t fmcb_imu[6];
  uint16_t pp_temp[4];
  uint16_t dp_temp[4];
  uint16_t fmcb_temp[3];
  uint16_t fmcb_voltage;
  uint16_t fmcb_pb_current;
  uint32_t pp_strain;
  int32_t  fmcb_hall_tgt[3];
  int32_t  fmcb_hall_pos[3];
} finger_state_t;
/*
static void print_uint16_array(const char *name, 
                               const uint16_t *p, const uint16_t len)
{
  printf("%s:\n  ", name);
  for (int i = 0; i < len; i++)
    printf("%06d ", p[i]);
  printf("\n");
}
*/
void MotorModule::rxFingerState(const uint8_t *payload, 
                                const uint16_t payload_len)
{
  /*
  printf("rx finger status %d bytes\n  ", payload_len);
  for (int i = 0; i < payload_len; i++)
    printf("%02x ", payload[i]);
  printf("\n");
  */
  finger_state_t *p = (finger_state_t *)payload;
  /*
  print_uint16_array("pp tactile", p->pp_tactile,  6);
  print_uint16_array("dp tactile", p->dp_tactile, 12);
  print_uint16_array("pp imu"    , p->pp_imu    ,  6);
  print_uint16_array("dp imu"    , p->dp_imu    ,  6);
  print_uint16_array("fmcb imu"  , p->fmcb_imu  ,  6);
  print_uint16_array("pp temp"   , p->pp_temp   ,  4);
  print_uint16_array("dp temp"   , p->dp_temp   ,  4);
  print_uint16_array("fmcb temp" , p->fmcb_temp ,  3);
  const float fmcb_v_adc = (float)p->fmcb_voltage / 4095.0f * 3.3f;
  const float fmcb_v_cal = (100.0f + 10.0f) / 10.0f * fmcb_v_adc;
  printf("fmcb voltage: %d -> %.4f volts\n", p->fmcb_voltage, fmcb_v_cal);
  float cal_phalange_current = (float)p->fmcb_pb_current/4095.0f*3.3f/2.7;
  printf("fmcb phalange current: %d -> %.4f amps\n", 
         p->fmcb_pb_current, cal_phalange_current);
  printf("pp strain: %d\n", p->pp_strain);
  printf("hall targets: %d %d %d\n", 
         p->fmcb_hall_tgt[0], p->fmcb_hall_tgt[1], p->fmcb_hall_tgt[2]);
  printf("hall positions: %d %d %d\n", 
         p->fmcb_hall_pos[0], p->fmcb_hall_pos[1], p->fmcb_hall_pos[2]);
  */
  printf("MotorModule::rxFingerState fmcb time: %.6f\n", 
         p->fmcb_time * 1.0e-6);
  printf("\n");
}

bool MotorModule::pollFingerState()
{
  if (!sendTxBuffer(PKT_FINGER_STATUS, 0))
    return false;
  return listenFor(PKT_FINGER_STATUS, 0.5);
}

bool MotorModule::phalangeTxRx(const uint8_t *data, const uint16_t data_len)
{
  if (!data || data_len > MAX_PACKET_LENGTH - 10)
    return false;
  serializeUint16(data_len, getTxBuffer());
  serializeUint16(10, getTxBuffer()+2);
  memcpy(getTxBuffer()+4, data, data_len);
  return sendTxBuffer(PKT_PHALANGE_TXRX, data_len + 4);
}

bool MotorModule::setControlMode(const uint8_t  control_mode,
                                 const float   *pos,
                                 const uint8_t *effort)
{
  // this is a private function because it's easy to make the hand go bonkers
  // if you send wild inputs here.
  getTxBuffer()[0] = control_mode;
  for (int i = 0; i < 3; i++)
  {
    serializeFloat32(pos ? pos[i] : 0, getTxBuffer() + 1 + 4*i);
    getTxBuffer()[13+i] = effort ? effort[i] : 50;
  }
  if (!sendTxBuffer(PKT_CONTROL_MODE, 1 + 4*3 + 3))
    return false;
  return true;
}

bool MotorModule::setMotorsIdle()
{
  return setControlMode(CM_IDLE, NULL, NULL);
}

bool MotorModule::setMotorPos(const int16_t *motor_pos)
{
  // caution! bypasses joint limits!
  if (!motor_pos)
    return false;
  getTxBuffer()[0] = CM_MOTOR_SPACE;
  for (int i = 0; i < 3; i++)
    serializeInt16(motor_pos[i], getTxBuffer() + 1 + 2*i);
  if (!sendTxBuffer(PKT_CONTROL_MODE, 1 + 4*2))
    return false;
  return true;
}

bool MotorModule::setJointPosHome()
{
  return setControlMode(CM_JOINT_SPACE_WITH_MAX_EFFORT, NULL, NULL);
}

bool MotorModule::setJointPos(const float *pos, const uint8_t *effort)
{
  if (!pos || !effort)
    return false;
  return setControlMode(CM_JOINT_SPACE_WITH_MAX_EFFORT, pos, effort);
}

bool MotorModule::setRelativeJointPos(const float *pos, const uint8_t *effort)
{
  if (!pos || !effort)
    return false;
  return setControlMode(CM_JOINT_SPACE_RELATIVE, pos, effort);
}

/*  
                               const uint16_t timeout_ms)
  if (listenFor(PKT_PHALANGE_TXRX, 0.1f + 0.001f * timeout_ms))
  {
    printf("phalange last payload: %d bytes\n", (int)phalange_rx.size());
    return true;
  }
  printf("no response to phalange txrx\n");
  return false;
}
*/

void MotorModule::rxPhalangeTxRx(const uint8_t *data, const uint16_t data_len)
{
  uint16_t pkt_load = deserializeUint16(data);
  if (pkt_load >= MAX_PACKET_LENGTH || pkt_load > data_len - 2) // sanity check
    return;
  for (vector<RxFunctor>::iterator it = phalange_rx_functors.begin();
       it != phalange_rx_functors.end(); ++it)
    (*it)(data + 2, pkt_load); // neato
}

void MotorModule::addPhalangeRxFunctor(RxFunctor f)
{
  phalange_rx_functors.push_back(f);
}

bool MotorModule::setJointLimits(const float *lower, const float *upper)
{
  bool all_ok = true;
  all_ok &= setParamFloat("j0_lower_limit", lower[2]);
  all_ok &= setParamFloat("j1_lower_limit", lower[1]);
  all_ok &= setParamFloat("j2_lower_limit", lower[0]);
  all_ok &= setParamFloat("j0_upper_limit", upper[2]);
  all_ok &= setParamFloat("j1_upper_limit", upper[1]);
  all_ok &= setParamFloat("j2_upper_limit", upper[0]);
  return all_ok;
}

bool MotorModule::setHallOffsets(const int32_t *offsets)
{
  bool all_ok = true;
  all_ok &= setParamInt("m0_offset", offsets[2]);
  all_ok &= setParamInt("m1_offset", offsets[1]);
  all_ok &= setParamInt("m2_offset", offsets[0]);
  return all_ok;
}

