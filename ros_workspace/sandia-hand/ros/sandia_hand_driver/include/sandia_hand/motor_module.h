#ifndef SANDIA_HAND_MOTOR_MODULE_H
#define SANDIA_HAND_MOTOR_MODULE_H

#include "sandia_hand/serial_message_processor.h"

namespace sandia_hand
{

class MotorModule : public SerialMessageProcessor
{
public:
  MotorModule(const uint8_t addr);
  virtual ~MotorModule();
  bool setPhalangeBusPower(bool on);
  bool setPhalangeAutopoll(bool on); 
  bool pollFingerState();
  bool phalangeTxRx(const uint8_t *data, const uint16_t data_len);
                    //const uint16_t timeout_ms);
  void addPhalangeRxFunctor(RxFunctor f);
  bool setJointLimits(const float *lower, const float *upper);
  bool setHallOffsets(const int32_t *offsets);

  bool setMotorsIdle();
  bool setJointPosHome();
  bool setJointPos(const float *joint_pos, const uint8_t *max_efforts);
  bool setMotorPos(const int16_t *motor_pos); // caution! no joint limits!
  bool setRelativeJointPos(const float *joint_pos, const uint8_t *max_efforts);
  static const uint8_t PKT_FINGER_STATUS     = 0x21;
private:
  static const uint8_t PKT_CONTROL_MODE      = 0x1d;
  static const uint8_t PKT_PHALANGE_POWER    = 0x1e;
  static const uint8_t PKT_PHALANGE_TXRX     = 0x1f;
  static const uint8_t PKT_PHALANGE_AUTOPOLL = 0x20; // broken
  enum ControlMode { CM_IDLE = 0, CM_MOTOR_SPACE, 
                     CM_JOINT_SPACE, CM_JOINT_SPACE_FP,
                     CM_JOINT_SPACE_WITH_MAX_EFFORT,
                     CM_JOINT_SPACE_RELATIVE };
  void rxFingerState(const uint8_t *payload, const uint16_t payload_len);
  void rxPhalangeTxRx(const uint8_t *data, const uint16_t data_len);
  bool setControlMode(const uint8_t control_mode,
                      const float *joint_pos, 
                      const uint8_t *max_efforts);
  std::vector<uint8_t> phalange_rx;
  std::vector<RxFunctor> phalange_rx_functors;
};

}

#endif

