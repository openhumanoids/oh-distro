#ifndef SANDIA_HAND_H
#define SANDIA_HAND_H

#include <vector>
#include <map>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include "hand_packets.h"
#include <boost/function.hpp>
#include <sandia_hand/finger.h>
#include <sandia_hand/palm.h>
#include <ros/time.h>

namespace sandia_hand
{

class Hand
{
public:
  static const int NUM_FINGERS = 4;
  Finger fingers[NUM_FINGERS];
  Palm palm;
  enum Side { RIGHT, LEFT, UNKNOWN };
  
  Hand();
  ~Hand();
  static const uint16_t DEFAULT_HAND_BASE_PORT = 12321; // i love palindromes
  bool init(const char *ip = "10.66.171.23", 
            const uint16_t base_port = DEFAULT_HAND_BASE_PORT); // right hand

  enum FingerPowerState { FPS_OFF  = FINGER_POWER_STATE_OFF, 
                          FPS_LOW  = FINGER_POWER_STATE_LOW,
                          FPS_FULL = FINGER_POWER_STATE_FULL }; 
  bool setFingerPower(const uint8_t finger_idx, const FingerPowerState fps);
  bool setAllFingerPowers(const FingerPowerState fps);
  bool enableLowvoltRegulator(const bool on);

  enum FingerControlMode { FCM_IDLE      = FINGER_CONTROL_MODE_IDLE,
                           FCM_JOINT_POS = FINGER_CONTROL_MODE_JOINT_POS };
  bool setFingerControlMode(const uint8_t finger_idx, 
                            const FingerControlMode fcm);
  bool setFingerJointPos(const uint8_t finger_idx,
                         float joint_0, float joint_1, float joint_2);
  bool setAllFingerJointPos(const float   *joint_pos,
                            const uint8_t *joint_max_effort);
  bool setAllRelativeFingerJointPos(const float   *relative_joint_pos,
                                    const uint8_t *joint_max_effort);
  bool listen(const float max_seconds);
  bool setCameraStreaming(const bool cam_0_streaming, 
                          const bool cam_1_streaming);
  static const int IMG_WIDTH = 720, IMG_HEIGHT = 480, NUM_CAMS = 2;
  typedef boost::function<void(const uint8_t, const uint32_t, 
                               const uint8_t *)> ImageCallback;
  typedef boost::function<void(const uint8_t *, const uint16_t)> RxFunctor;
  void registerRxHandler(const uint32_t msg_id, RxFunctor f);
  void setImageCallback(ImageCallback callback);
  bool pingFinger(const uint8_t finger_idx);
  bool setMoboStateHz(const uint16_t mobo_state_hz);
  bool setFingerAutopollHz(const uint16_t finger_autopoll_hz);
  // todo: bake all of these SAM3S bootloader burns into a single function
  bool programMotorModuleAppFile(const uint8_t finger_idx, FILE *bin_file);
  bool programDistalPhalangeAppFile(const uint8_t finger_idx, FILE *bin_file);
  bool programProximalPhalangeAppFile(const uint8_t finger_idx, FILE *bin_file);
  bool programPalmAppFile(FILE *bin_file);
  bool programFPGAGoldenFile(FILE *bin_file);
  bool programFPGAAppFile(FILE *bin_file);
  bool programMoboMCUAppFile(FILE *bin_file);
  bool readMoboFlashPage(const uint32_t page_num, std::vector<uint8_t> &page);
  bool writeMoboFlashPage(const uint32_t page_num, std::vector<uint8_t> &page);
  bool eraseMoboFlashSector(const uint32_t page_num); // erases ENTIRE sector!
  bool readMoboMCUPage(const uint32_t page_num, std::vector<uint8_t> &page);
  bool writeMoboMCUPage(const uint32_t page_num, std::vector<uint8_t> &page);
  bool resetMoboMCU();
  bool haltMoboMCUAutoboot();
  bool bootMoboMCU();
  bool pingMoboMCU();
  bool setMoboCurrentLimit(const float limit);
  bool getHwVersion(uint32_t &version);
  Side getSide(); 
private:
  Side side_;
  static const int NUM_SOCKS = 4;
  int control_sock, cam_socks[NUM_CAMS], rs485_sock;
  sockaddr_in control_saddr, cam_saddrs[NUM_CAMS], rs485_saddr;
  int *socks[NUM_SOCKS];
  sockaddr_in *saddrs[NUM_SOCKS];
  bool tx_udp(uint8_t *pkt, uint16_t pkt_len);
  bool rx_data(const int sock_idx, const uint8_t *data, const int data_len);
  uint8_t *img_data[NUM_CAMS]; // camera image buffers
  bool *img_rows_recv[NUM_CAMS]; // keep track of completeness
  ImageCallback img_cb;
  bool fingerRawTx(const uint8_t finger_idx, 
                   const uint8_t *data, const uint16_t data_len);
  std::map<uint32_t, RxFunctor> rx_map_;
  std::map<uint32_t, std::vector< std::vector< uint8_t > > > rx_flags_;
  std::map<uint8_t, uint8_t> rx_rs485_map_; // changes in right vs left hands
  bool listenForDuration(float seconds);

  // this feels really inelegant. goal is just to reduce repetitive code...
  uint32_t last_packet_id_;
  std::vector<uint8_t> last_packet_data_;
  template <class T> bool listenForPacketId(const uint32_t id,
                                            const float max_seconds,
                                            T &response)
  {
    last_packet_id_ = 0;
    for (ros::Time t_start(ros::Time::now()); 
         (ros::Time::now() - t_start).toSec() < max_seconds;)
    {
      if (!this->listen(0.01))
        return false;
      if (last_packet_id_ != id)
        continue;
      if (last_packet_data_.size() != sizeof(T))
        continue;
      memcpy(&response, &last_packet_data_[0], sizeof(T)); // populate response
      return true;
    }
    return false;
  }

  // feels inelegant to put it here, but templates make the calling code nice
  template <class T> bool txPacket(const uint32_t id, const T &pkt)
  {
    static std::vector<uint8_t> s_txBuf; // ugly. the idea is to save alloc time
    s_txBuf.resize(4 + sizeof(T));
    *((uint32_t *)&s_txBuf[0]) = id;
    memcpy(&s_txBuf[4], &pkt, sizeof(T));
    if (-1 == sendto(control_sock, &s_txBuf[0], sizeof(T) + 4, 0, 
                     (sockaddr *)&control_saddr, sizeof(sockaddr)))
      return false;
    return true;
  }
  
  bool programFPGAFile(const int start_page, FILE *bin_file);
};

}

#endif

