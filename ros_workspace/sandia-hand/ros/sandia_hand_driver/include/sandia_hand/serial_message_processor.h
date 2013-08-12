#ifndef SANDIA_HAND_SERIAL_MESSAGE_PROCESSOR
#define SANDIA_HAND_SERIAL_MESSAGE_PROCESSOR

#include <stdint.h>
#include <boost/function.hpp>
#include <vector>
#include <map>
#include "sandia_hand/param.h"

namespace sandia_hand
{

class SerialMessageProcessor
{
public:
  SerialMessageProcessor(const uint8_t addr);
  virtual ~SerialMessageProcessor();
  bool rx(const uint8_t *data, const uint16_t data_len);

  typedef boost::function<void(const uint8_t *, const uint16_t)> RawTxFunctor;
  inline void setRawTx(RawTxFunctor f) { raw_tx_ = f; }
  typedef boost::function<void(const uint8_t *, const uint16_t)> RxFunctor;
  void registerRxHandler(uint8_t msg_id, RxFunctor f);
  bool ping();
  typedef boost::function<void(const float)> ListenFunctor;
  void registerListenHandler(ListenFunctor functor);
  bool blHaltAutoboot();
  bool blBoot();
  bool blReadFlashPage(const uint16_t page_num, uint8_t *page_buf);
  bool blWriteFlashPage(const uint16_t page_num, const uint8_t *page_buf, 
                        bool chop);
  bool reset();
  uint32_t getHardwareVersion();

  typedef boost::function<bool()> PowerFunctor;
  bool programAppFile(FILE *bin_file, 
                      PowerFunctor power_on, PowerFunctor power_off);
  bool setParamFloat(const std::string &name, const float val);
  bool setParamInt(const std::string &name, const int32_t val);
  bool getParamNames(std::vector<std::string> &names);
  const std::vector<sandia_hand::Param> &getParams();

protected:
  static const uint32_t MAX_PACKET_LENGTH = 512;
  RawTxFunctor raw_tx_;
  bool print_parser_debris_;
  inline uint8_t *getTxBuffer() { return (uint8_t *)(&outgoing_packet_[5]); }
  bool sendTxBuffer(const uint8_t pkt_id, uint16_t payload_len = 0);
  bool listenFor(const uint8_t listen_pkt_type, const float max_seconds);
  bool listenFor(float seconds);
  void stopListening();

  uint16_t deserializeUint16(const uint8_t *p);
  uint32_t deserializeUint32(const uint8_t *p);
  float    deserializeFloat32(const uint8_t *p);
  void serializeUint16(const uint16_t x, uint8_t *p);
  void serializeInt16 (const  int16_t x, uint8_t *p);
  void serializeUint32(const uint32_t x, uint8_t *p);
  void serializeInt32(const int32_t x, uint8_t *p);
  void serializeFloat32(const float x, uint8_t *p);
  void resetParser();
  bool retrieveParams();

private:
  uint8_t addr_;
  uint8_t rx_pkt_addr_, rx_pkt_type_;
  uint16_t rx_pkt_write_idx_, rx_pkt_len_, rx_pkt_crc_;
  enum { ST_IDLE=0, ST_ADDRESS=1, ST_LEN_1=2, ST_LEN_2=3, 
         ST_TYPE=4, ST_DATA=5, ST_CRC_1=6, ST_CRC_2=7 } rx_pkt_parser_state_;
  std::vector<uint8_t> rx_pkt_data_;
  std::map<uint8_t, RxFunctor> rx_map_;
  std::vector<uint8_t> outgoing_packet_;
  static const uint8_t PKT_PING                  = 0x01;
  static const uint8_t PKT_BL_HALT_AUTOBOOT      = 0x0c;
  static const uint8_t PKT_BL_BOOT               = 0x08;
  static const uint8_t PKT_RESET                 = 0x09;
  static const uint8_t PKT_BL_READ_FLASH_PAGE    = 0x0a;
  static const uint8_t PKT_BL_WRITE_FLASH_PAGE   = 0x0b;
  static const uint8_t PKT_BL_SET_FLASH_BUF_WORD = 0x0d;
  static const uint8_t PKT_BL_WRITE_FLASH_BUF    = 0x0e;
  static const uint8_t PKT_READ_NUM_PARAMS       = 0x16;
  static const uint8_t PKT_READ_PARAM_NAME       = 0x17;
  static const uint8_t PKT_READ_PARAM_VALUE      = 0x18;
  static const uint8_t PKT_WRITE_PARAM_VALUE     = 0x19;
  static const uint8_t PKT_READ_HW_VERSION       = 0xfa;
  void rxPing(const uint8_t *payload, const uint16_t payload_len);
  void rxByte(const uint8_t b);
  ListenFunctor listen_functor_;
  bool done_listening_;
  uint8_t listen_pkt_type_;
  std::vector<sandia_hand::Param> params_;
};

}

#endif
