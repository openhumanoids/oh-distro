#include "sandia_hand/hand.h" 
#include <cstdio>
#include <cstring>
#include <cmath>
#include <boost/bind.hpp>
#include "sandia_hand/hand_packets.h"
using namespace sandia_hand;
using std::vector;

Hand::Hand()
: side_(Hand::UNKNOWN), last_packet_id_(0)
{
  socks[0] = &control_sock;
  socks[1] = &cam_socks[0];
  socks[2] = &cam_socks[1];
  socks[3] = &rs485_sock;
  saddrs[0] = &control_saddr;
  saddrs[1] = &cam_saddrs[0];
  saddrs[2] = &cam_saddrs[1];
  saddrs[3] = &rs485_saddr;
  for (int i = 0; i < NUM_SOCKS; i++)
    *socks[i] = 0;
  for (int i = 0; i < NUM_CAMS; i++)
  {
    img_data[i] = new uint8_t[IMG_WIDTH * IMG_HEIGHT];
    img_rows_recv[i] = new bool[IMG_HEIGHT];
    for (int j = 0; j < IMG_HEIGHT; j++)
      img_rows_recv[i][j] = false;
  }
  for (int i = 0; i < NUM_FINGERS; i++)
  {
    fingers[i].mm.setRawTx(boost::bind(&Hand::fingerRawTx, this, i, _1, _2));
    fingers[i].registerListenHandler(boost::bind(&Hand::listen, this, _1));
  }
  palm.setRawTx(boost::bind(&Hand::fingerRawTx, this, 4, _1, _2));
  palm.registerListenHandler(boost::bind(&Hand::listen, this, _1));
}

Hand::~Hand()
{
  for (int i = 0; i < NUM_SOCKS; i++)
    if (*socks[i])
      close(*socks[i]);
}

bool Hand::init(const char *ip, const uint16_t base_port)
{
  for (int i = 0; i < NUM_SOCKS; i++)
  {
    if (-1 == (*socks[i] = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)))
    {
      perror("could't create udp socket");
      return false;
    }
    bzero(saddrs[i], sizeof(saddrs[i]));
    saddrs[i]->sin_family = AF_INET;
    saddrs[i]->sin_port = htons(base_port + i);
    saddrs[i]->sin_addr.s_addr = INADDR_ANY;
    if (bind(*socks[i], (struct sockaddr *)saddrs[i], sizeof(*saddrs[i])) != 0)
    {
      perror("couldn't bind udp socket");
      return false;
    }
    // recycle the saddr structs to easily send future outgoing datagrams
    bzero(saddrs[i], sizeof(*saddrs[i]));
    saddrs[i]->sin_family = AF_INET;
    if (0 == inet_aton(ip, &saddrs[i]->sin_addr))
    {
      perror("inet_aton");
      return false;
    }
    saddrs[i]->sin_port = htons(DEFAULT_HAND_BASE_PORT + i);
  }
  // set the port range we want
  set_dest_port_t request, response;
  request.pkt_state = MOBO_SET_DEST_PORT_REQUEST;
  request.port = base_port;
  if (!txPacket(CMD_ID_MOBO_SET_DEST_PORT, request))
    return false;
  if (!listenForPacketId(CMD_ID_MOBO_SET_DEST_PORT, 0.25, response))
  {
    perror("didn't hear back from hand after setting base port.");
    // don't necessarily bail here; the hand could be in bootloader still.
  }
  // now, we can query the hand for its hardware version.
  side_ = getSide();
  if (side_ == Hand::LEFT)
  {
    rx_rs485_map_[0] = 1;
    rx_rs485_map_[1] = 0;
    rx_rs485_map_[2] = 3;
    rx_rs485_map_[3] = 2;
    rx_rs485_map_[4] = 4;
  }
  else
  {
    // otherwise, assume right hand
    rx_rs485_map_[0] = 4;
    rx_rs485_map_[1] = 1;
    rx_rs485_map_[2] = 2;
    rx_rs485_map_[3] = 3;
    rx_rs485_map_[4] = 0;
  }
  return true;
}

bool Hand::enableLowvoltRegulator(const bool on)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_ENABLE_LOWVOLT_REGULATOR;
  enable_lowvolt_regulator_t *p = (enable_lowvolt_regulator_t *)(pkt+4);
  p->enable = on ? 1 : 0;
  if (!tx_udp(pkt, 4 + sizeof(enable_lowvolt_regulator_t)))
    return false;
  return true;
}

bool Hand::setFingerPower(const uint8_t finger_idx, const FingerPowerState fps)
{
  if (finger_idx >= NUM_FINGERS)
    return false;
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_FINGER_POWER_STATE;
  set_finger_power_state_t *sfps = (set_finger_power_state_t *)(pkt+4);
  sfps->finger_idx = finger_idx;
  sfps->finger_power_state = (uint8_t)fps;
  if (!tx_udp(pkt, 4 + sizeof(set_finger_power_state_t)))
    return false;
  return true;
}

bool Hand::setAllFingerPowers(const FingerPowerState fps)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_ALL_FINGER_POWER_STATES;
  set_all_finger_power_states_t *p = 
                              (set_all_finger_power_states_t *)(pkt+4);
  for (int i = 0; i < 4; i++)
    p->fps[i] = fps;
  if (!tx_udp(pkt, 4 + sizeof(set_all_finger_power_states_t)))
    return false;
  return true;
}

bool Hand::setFingerControlMode(const uint8_t finger_idx,
                                const FingerControlMode fcm)
{
  if (finger_idx >= NUM_FINGERS)
    return false;
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_FINGER_CONTROL_MODE;
  set_finger_control_mode_t *p = (set_finger_control_mode_t *)(pkt + 4);
  p->finger_idx = finger_idx;
  p->finger_control_mode = (uint8_t)fcm;
  if (!tx_udp(pkt, 4 + sizeof(set_finger_control_mode_t)))
    return false;
  return true;
}

bool Hand::setFingerJointPos(const uint8_t finger_idx,
                             float joint_0, float joint_1, float joint_2)
{
  if (finger_idx >= NUM_FINGERS)
    return false;
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_FINGER_JOINT_POS;
  set_finger_joint_pos_t *p = (set_finger_joint_pos_t *)(pkt + 4);
  p->finger_idx = finger_idx;
  // permute joints as needed... numbering scheme in schematics is different.
  p->joint_0_radians =  joint_2;
  p->joint_1_radians =  joint_1;
  p->joint_2_radians = (side_ == RIGHT ? -joint_0 : joint_0);
  if (!tx_udp(pkt, 4 + sizeof(set_finger_joint_pos_t)))
    return false;
  return true;
}

bool Hand::setAllFingerJointPos(const float   *joint_pos,
                                const uint8_t *joint_max_effort)
{
  uint8_t pkt[100];
  *((uint32_t *)pkt) = CMD_ID_HAND_JOINT_COMMANDS;
  hand_joint_commands_t *p = (hand_joint_commands_t *)(pkt + 4);
  for (int finger_idx = 0; finger_idx < NUM_FINGERS; finger_idx++)
  {
    p->joint_angles[finger_idx*3  ] =  joint_pos[finger_idx*3+2];
    p->joint_angles[finger_idx*3+1] =  joint_pos[finger_idx*3+1];
    p->joint_angles[finger_idx*3+2] = 
       (side_ == RIGHT ? -joint_pos[finger_idx*3] : joint_pos[finger_idx*3]);
    p->max_efforts[finger_idx*3  ] =   joint_max_effort[finger_idx*3+2];
    p->max_efforts[finger_idx*3+1] =   joint_max_effort[finger_idx*3+1];
    p->max_efforts[finger_idx*3+2] =   joint_max_effort[finger_idx*3  ];
  }
  if (!tx_udp(pkt, 4 + sizeof(hand_joint_commands_t)))
    return false;
  return true;
}

// todo: factor this method and the previous one somehow
bool Hand::setAllRelativeFingerJointPos(const float   *relative_joint_pos,
                                        const uint8_t *joint_max_effort)
{
  uint8_t pkt[100];
  *((uint32_t *)pkt) = CMD_ID_HAND_RELATIVE_JOINT_COMMANDS;
  relative_joint_commands_t *p = (relative_joint_commands_t *)(pkt + 4);
  for (int i = 0; i < NUM_FINGERS; i++)
  {
    p->relative_joint_angles[i*3  ] =  relative_joint_pos[i*3+2];
    p->relative_joint_angles[i*3+1] =  relative_joint_pos[i*3+1];
    p->relative_joint_angles[i*3+2] = 
      (side_ == RIGHT ? -relative_joint_pos[i*3] : relative_joint_pos[i*3]);
    p->max_efforts[i*3  ] =   joint_max_effort[i*3+2];
    p->max_efforts[i*3+1] =   joint_max_effort[i*3+1];
    p->max_efforts[i*3+2] =   joint_max_effort[i*3  ];
  }
  if (!tx_udp(pkt, 4 + sizeof(relative_joint_commands_t)))
    return false;
  return true;
}

bool Hand::setCameraStreaming(const bool cam_0_streaming, 
                              const bool cam_1_streaming)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_CONFIGURE_CAMERA_STREAM;
  configure_camera_stream_t *p = (configure_camera_stream_t *)(pkt + 4);
  p->cam_0_stream = cam_0_streaming ? CAMERA_STREAM_ON : CAMERA_STREAM_OFF;
  p->cam_1_stream = cam_1_streaming ? CAMERA_STREAM_ON : CAMERA_STREAM_OFF;
  return tx_udp(pkt, 4 + sizeof(configure_camera_stream_t));
}

bool Hand::setMoboStateHz(const uint16_t mobo_status_hz)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_MOBO_STATUS_RATE;
  set_mobo_state_rate_t *p = (set_mobo_state_rate_t *)(pkt+4);
  p->mobo_state_hz = mobo_status_hz;
  return tx_udp(pkt, 4 + sizeof(set_mobo_state_rate_t));
}

bool Hand::setFingerAutopollHz(const uint16_t autopoll_hz)
{
  uint8_t pkt[50];
  *((uint32_t *)pkt) = CMD_ID_SET_FINGER_AUTOPOLL;
  set_finger_autopoll_t *p = (set_finger_autopoll_t *)(pkt+4);
  p->finger_autopoll_hz = autopoll_hz;
  return tx_udp(pkt, 4 + sizeof(set_finger_autopoll_t));
}

bool Hand::fingerRawTx(const uint8_t finger_idx, 
                       const uint8_t *data, const uint16_t data_len)
{
  /*
  printf("Hand::fingerRawTx(%d, x, %d)\n", finger_idx, data_len);
  for (int i = 0; i < data_len; i++)
    printf("  %02d:0x%02x\n", i, data[i]);
  */
  // note: finger_idx of 4 means the palm
  uint8_t pkt[FINGER_RAW_TX_MAX_LEN+20];
  *((uint32_t *)pkt) = CMD_ID_FINGER_RAW_TX;
  finger_raw_tx_t *p = (finger_raw_tx_t *)(pkt + 4);
  p->finger_idx = finger_idx;
  p->pad = 0;
  p->tx_data_len = data_len;
  for (int i = 0; i < data_len && i < FINGER_RAW_TX_MAX_LEN; i++)
    p->tx_data[i] = data[i]; // todo: speed this up someday if it ever matters
  return tx_udp(pkt, 4 + sizeof(finger_raw_tx_t) - 
                     FINGER_RAW_TX_MAX_LEN + data_len);  // ugly
}

bool Hand::tx_udp(uint8_t *pkt, uint16_t pkt_len)
{
  //printf("tx_udp %d bytes to socket %d\n", pkt_len, control_sock);
  if (-1 == sendto(control_sock, pkt, pkt_len, 0, 
                   (sockaddr *)&control_saddr, sizeof(sockaddr)))
  {
    perror("couldn't send udp packet");
    return false;
  }
  return true;
}

bool Hand::listen(const float max_seconds)
{
  //printf("listen()\n");
  timeval timeout;
  timeout.tv_sec  = (time_t)trunc(max_seconds);
  timeout.tv_usec = (suseconds_t)((max_seconds - timeout.tv_sec) * 1e6);
  fd_set rdset;
  FD_ZERO(&rdset);
  for (int i = 0; i < NUM_SOCKS; i++)
    FD_SET(*socks[i], &rdset);
  int rv = select(*socks[NUM_SOCKS-1]+1, &rdset, NULL, NULL, &timeout);
  if (rv < 0)
    return false; // select returned some error. need better way to bubble up
  if (rv == 0)
    return true; // no errors, but nothing happened. bail.
  for (int i = 0; i < NUM_SOCKS; i++)
    if (FD_ISSET(*socks[i], &rdset)) 
    {
      //printf("%d set\n", i);
      int bytes_recv;
      sockaddr_in recv_addr;
      socklen_t addr_len = sizeof(recv_addr);
      uint8_t recv_buf[2000];
      if ((bytes_recv = recvfrom(*socks[i], recv_buf, sizeof(recv_buf), 0,
                                 (struct sockaddr *)&recv_addr, 
                                 &addr_len)) == -1) 
      {
        perror("recvfrom"); // find a better way to report this...
        return false;
      }
      if (!rx_data(i, recv_buf, bytes_recv)) // just to reduce indenting...
        return false;
    }
  return true; // no errors
}

bool Hand::rx_data(const int sock_idx, const uint8_t *data, const int data_len)
{
  //printf("received %d bytes on sock %d\n", data_len, sock_idx);
  if (sock_idx == 1 || sock_idx == 2)
  {
    const int cam_idx = sock_idx - 1;
    const uint32_t frame_count = *((uint32_t *)data); // maybe use sometime?
    const uint16_t row_count = *((uint16_t *)(data+4));
    const uint8_t *pixels = data + 8;
    if (row_count == 0)
      for (int i = 0; i < IMG_HEIGHT; i++)
        img_rows_recv[cam_idx][i] = false;
    if (row_count < IMG_HEIGHT)
    {
      memcpy(img_data[cam_idx] + row_count * IMG_WIDTH, pixels, IMG_WIDTH);
      img_rows_recv[cam_idx][row_count] = true;
    }
    if (row_count == IMG_HEIGHT - 1)
    {
      // only counts if it's a complete image...
      bool all_recv = true;
      for (int i = 0; i < IMG_HEIGHT-1 && all_recv; i++)
        if (!img_rows_recv[cam_idx][i])
          all_recv = false;
      if (all_recv && img_cb)
        img_cb(cam_idx, frame_count, img_data[cam_idx]);
    }
  }
  else if (sock_idx == 0)
  {
    const uint32_t pkt_id = *((uint32_t *)(data));
    if (rx_map_.find(pkt_id) != rx_map_.end())
      rx_map_[pkt_id](data + 4, (uint16_t)data_len - 4); // neat.
    //else
    //  printf("unhandled packet id: %d\n", pkt_id);
    // this feels inelegant. revisit at some point.
    last_packet_id_ = pkt_id;
    last_packet_data_.resize(data_len-4);
    memcpy(&last_packet_data_[0], data + 4, data_len - 4);
  }
  else if (sock_idx == 3) // rs485 sock
  {
    //printf("rs485 sock received %d bytes\n", data_len);
    // bytes come in pairs on this socket
    for (int i = 0; i < data_len / 2; i++)
    {
      const uint8_t rs485_sender_byte = data[i * 2]; 
      if (!(rs485_sender_byte & 0x80)) // hardware sets high bit. verify.
      {
        //printf("WOAH THERE PARTNER. unexpected byte on rs485 sock: 0x%02x\n",
        //       rs485_sender_byte);
        // actually... this happens as needed to align the UDP payloads on 
        // quadword boundaries, for easier checksum generation in the fpga.
        continue;
      }
      const uint8_t rs485_sender = rs485_sender_byte & ~0x80;
      if (rx_rs485_map_.find(rs485_sender) == rx_rs485_map_.end())
        continue; // bogus sender address. maybe garbled packet somehow.
      const uint8_t remapped_sender = rx_rs485_map_[rs485_sender];
      const uint8_t rs485_byte = data[i * 2 + 1];
      // printf("rs485 sender = %d mapped to %d\n", 
      //        rs485_sender, remapped_sender);
      if (remapped_sender < 4)
        fingers[remapped_sender].mm.rx(&rs485_byte, 1); // todo: batch this
      else if (remapped_sender == 4)
        palm.rx(&rs485_byte, 1);
    }
  }
  return true;
}

void Hand::setImageCallback(ImageCallback callback)
{
  img_cb = callback;
}

bool Hand::pingFinger(const uint8_t finger_idx)
{
  return false; // todo
}

void Hand::registerRxHandler(const uint32_t msg_id, RxFunctor f)
{
  rx_map_[msg_id] = f;
}

bool Hand::programMotorModuleAppFile(const uint8_t finger_idx, FILE *bin_file)
{
  if (finger_idx >= 4 || !bin_file)
    return false; // sanity check
  Finger *finger = &fingers[finger_idx];
  return finger->mm.programAppFile(bin_file,
            boost::bind(&Hand::setFingerPower, this, finger_idx, FPS_OFF),
            boost::bind(&Hand::setFingerPower, this, finger_idx, FPS_LOW));
}

bool Hand::programDistalPhalangeAppFile(const uint8_t finger_idx, 
                                        FILE *bin_file)
{
  if (finger_idx >= 4 || !bin_file)
    return false; // sanity check
  Finger *finger = &fingers[finger_idx];
  return finger->programDistalPhalangeAppFile(bin_file);
}

bool Hand::programProximalPhalangeAppFile(const uint8_t finger_idx, 
                                          FILE *bin_file)
{
  if (finger_idx >= 4 || !bin_file)
    return false; // sanity check
  Finger *finger = &fingers[finger_idx];
  return finger->programProximalPhalangeAppFile(bin_file);
}

bool Hand::programPalmAppFile(FILE *bin_file)
{
  return palm.programAppFile(bin_file,
               boost::bind(&Hand::enableLowvoltRegulator, this, false),
               boost::bind(&Hand::enableLowvoltRegulator, this, true));
}

bool Hand::programMoboMCUAppFile(FILE *bin_file)
{
  if (!resetMoboMCU())
    printf("couldn't reset mobo mcu. continuing anyway...\n");
  sleep(4);
  bool autoboot_halted = false;
  for (int attempt = 0; !autoboot_halted && attempt < 50; attempt++)
  {
    printf("autoboot halt attempt %d / %d...\n", attempt, 50);
    sleep(0.25);
    if (haltMoboMCUAutoboot())
      autoboot_halted = true;
  }
  if (!autoboot_halted)
  {
    printf("couldn't halt mobo autoboot. fail.\n");
    return false;
  }
  printf("mobo autoboot halted successfully.\n");
  // todo: ensure we are in bootloader mode by trying to halt autoboot.
  // this assumes that now we are in bootloader mode.
  for (int page_num = 128; !feof(bin_file) && page_num < 2048; page_num++)
  {
    vector<uint8_t> page_buf;
    page_buf.resize(256);
    size_t nread = 0;
    nread = fread(&page_buf[0], 1, 256, bin_file);
    if (nread == 0)
    {
      printf("couldn't read a flash page from FILE: returned %d\n",
             (int)nread);
      return false;
    }
    else if (nread < 256)
      printf("partial page: %d bytes, hopefully last flash page?\n",
             (int)nread);
    if (!writeMoboMCUPage(page_num, page_buf)) // todo: try a few times?
    {
      printf("couldn't write page %d\n", page_num);
      return false;
    }
  }
  if (!bootMoboMCU())
  {
    printf("couldn't boot mobo mcu.\n");
    return false;
  }
  return true;
  // todo: buffer the file and verify it at the end.
}

bool Hand::programFPGAGoldenFile(FILE *bin_file)
{
  return programFPGAFile(32768, bin_file); // midpoint of flash
}

bool Hand::programFPGAAppFile(FILE *bin_file)
{
  return programFPGAFile(0, bin_file); // put application image at root
}

bool Hand::programFPGAFile(const int start_page, FILE *bin_file)
{
  vector<uint8_t> page;
  page.resize(256);
  // as we go along, erase sectors as needed.
  for (int page_num = 0 + start_page; 
       page_num < 30000 + start_page && !feof(bin_file); 
       page_num++)
  {
    if (page_num % 256 == 0) // first page of this sector, erase first
    {
      printf("erasing sector starting at page 0x%x...\n", page_num);
      if (!eraseMoboFlashSector(page_num))
        return false;
    }
    //printf("programming page %x...\n", page_num);
    memset(&page[0], 0, 256);
    size_t nread = fread(&page[0], 1, 256, bin_file);
    if (nread == 0)
    {
      printf("couldn't read a flash page, fread returned %d\n", (int)nread);
      return false;
    }
    else if (nread < 256)
      printf("partial read (%d bytes) of page %d. hopefully last page?\n",
             (int)nread, page_num);
    if (!writeMoboFlashPage(page_num, page))
    {
      printf("couldn't write page %d\n", page_num);
      return false;
    }
  }
  printf("done.\n");
  return true;
  // todo: buffer the file and verify it at the end.
}

bool Hand::listenForDuration(float seconds)
{
  for (ros::Time t_start(ros::Time::now()); 
       (ros::Time::now() - t_start).toSec() < seconds;)
    if (!this->listen(0.01))
      return false;
  return true;
}

bool Hand::readMoboMCUPage(const uint32_t page_num, std::vector<uint8_t> &page)
{
  mobo_mcu_flash_page_t req, res;
  req.page_status = MOBO_MCU_FLASH_PAGE_STATUS_READ_REQ;
  req.page_num = page_num;
  if (!txPacket(CMD_ID_BL_MOBO_MCU_FLASH_PAGE, req))
    return false;
  if (!listenForPacketId(CMD_ID_BL_MOBO_MCU_FLASH_PAGE, 0.5, res))
    return false;
  if (res.page_status != MOBO_MCU_FLASH_PAGE_STATUS_READ_RES ||
      res.page_num != page_num)
  {
    printf("wrong page came back from mcu bootloader read request\r\n");
    return false;
  }
  page.resize(MOBO_MCU_FLASH_PAGE_SIZE);
  memcpy(&page[0], &res.page_data[0], MOBO_MCU_FLASH_PAGE_SIZE);
  return true;
}


bool Hand::readMoboFlashPage(const uint32_t page_num, 
                             vector<uint8_t> &page)
{
  read_fpga_flash_page_t req;
  req.page_num = page_num;
  if (!txPacket(CMD_ID_READ_FPGA_FLASH_PAGE, req))
    return false;
  fpga_flash_page_t p;
  if (!listenForPacketId(CMD_ID_FPGA_FLASH_PAGE, 0.5, p))
    return false;
  if (p.page_status != FPGA_FLASH_PAGE_STATUS_READ || p.page_num != page_num)
  {
    printf("wrong page came back from read request\n");
    return false;
  }
  /*
  printf("read flash page %d:\n", page_num);
  for (int i = 0; i < FPGA_FLASH_PAGE_SIZE; i++)
  {
    printf("0x%02x  ", p.page_data[i]);
    if (i % 8 == 7)
      printf("\n");
  }
  printf("\n");
  */
  page.resize(FPGA_FLASH_PAGE_SIZE);
  memcpy(&page[0], &p.page_data[0], FPGA_FLASH_PAGE_SIZE);
  return true;
}

bool Hand::resetMoboMCU()
{
  mobo_boot_ctrl_t req, res;
  req.boot_cmd = MOBO_BOOT_CTRL_RESET_REQUEST;
  if (!txPacket(CMD_ID_MOBO_BOOT_CTRL, req))
    return false;
  if (!listenForPacketId(CMD_ID_MOBO_BOOT_CTRL, 0.5, res))
  {
    printf("didn't hear back from reset command\n");
    return false;
  }
  if (res.boot_cmd != MOBO_BOOT_CTRL_RESET_RESPONSE)
  {
    printf("wrong response to reset command\n");
    return false;
  }
  return true;
}

bool Hand::writeMoboMCUPage(const uint32_t page_num, vector<uint8_t> &page)
{
  if (page.size() != 256)
    return false;
  mobo_mcu_flash_page_t req, res;
  req.page_num = page_num;
  req.page_status = MOBO_MCU_FLASH_PAGE_STATUS_WRITE_REQ;
  memcpy(req.page_data, &page[0], 256);
  if (!txPacket(CMD_ID_BL_MOBO_MCU_FLASH_PAGE, req))
    return false;
  if (!listenForPacketId(CMD_ID_BL_MOBO_MCU_FLASH_PAGE, 0.5, res))
    return false;
  if (res.page_status != MOBO_MCU_FLASH_PAGE_STATUS_WRITE_RES ||
      res.page_num != page_num)
  {
    printf("wrong page came back from write request\n");
    return false;
  }
  //printf("wrote flash page %d\n", page_num);
  return true;
}

bool Hand::writeMoboFlashPage(const uint32_t page_num, 
                              vector<uint8_t> &page)
{
  if (page.size() != 256)
    return false;
  fpga_flash_page_t req;
  req.page_num = page_num;
  req.page_status = FPGA_FLASH_PAGE_STATUS_WRITE_REQ;
  memcpy(req.page_data, &page[0], 256);
  if (!txPacket(CMD_ID_FPGA_FLASH_PAGE, req))
    return false;
  fpga_flash_page_t p;
  if (!listenForPacketId(CMD_ID_FPGA_FLASH_PAGE, 0.5, p))
    return false;
  if (p.page_status != FPGA_FLASH_PAGE_STATUS_WRITE_ACK || 
      p.page_num != page_num)
  {
    printf("wrong page came back from write request\n");
    return false;
  }
  //printf("wrote flash page %d\n", page_num);
  return true;
}

bool Hand::eraseMoboFlashSector(const uint32_t page_num)
{
  fpga_flash_erase_sector_t req;
  req.sector_page_num = page_num;
  if (!txPacket(CMD_ID_FPGA_FLASH_ERASE_SECTOR, req))
    return false;
  fpga_flash_erase_sector_ack_t p;
  if (!listenForPacketId(CMD_ID_FPGA_FLASH_ERASE_SECTOR_ACK, 2.0, p))
  {
    printf("no response in eraseMoboFlashSector\n");
    return false;
  }
  if (p.sector_page_num != page_num)
  {
    printf("in eraseMoboFlashSector: p.sector_page_num = %d, page_num = %d\n",
           p.sector_page_num, page_num);
    return false;
  }
  return true;
}

bool Hand::haltMoboMCUAutoboot()
{
  mobo_boot_ctrl_t request, response;
  request.boot_cmd = MOBO_BOOT_CTRL_BL_AUTOBOOT_HALT_REQUEST;
  if (!txPacket(CMD_ID_MOBO_BOOT_CTRL, request))
    return false;
  if (!listenForPacketId(CMD_ID_MOBO_BOOT_CTRL, 0.25, response))
  {
    printf("no response to MCU autoboot halt\n");
    return false;
  }
  if (response.boot_cmd != MOBO_BOOT_CTRL_BL_AUTOBOOT_HALT_RESPONSE)
  {
    printf("unexpected response\n");
    return false;
  }
  return true;
}

bool Hand::bootMoboMCU()
{
  mobo_boot_ctrl_t request, response;
  request.boot_cmd = MOBO_BOOT_CTRL_BL_BOOT_REQUEST;
  if (!txPacket(CMD_ID_MOBO_BOOT_CTRL, request))
    return false;
  if (!listenForPacketId(CMD_ID_MOBO_BOOT_CTRL, 0.25, response))
  {
    printf("no response to MCU boot command\n");
    return false;
  }
  if (response.boot_cmd != MOBO_BOOT_CTRL_BL_BOOT_RESPONSE)
  {
    printf("unexpected response\n");
    return false;
  }
  return true;
}

bool Hand::pingMoboMCU()
{
  mobo_ping_t request, response;
  request.state = MOBO_PING_REQUEST;
  if (!txPacket(CMD_ID_MOBO_PING, request))
    return false;
  if (!listenForPacketId(CMD_ID_MOBO_PING, 0.25, response))
    return false;
  if (response.state != MOBO_PING_RESPONSE)
    return false;
  return true;
}

bool Hand::setMoboCurrentLimit(const float limit)
{
  if (limit < 0)
    return false;
  if (limit > 10)
    return false;
  set_mobo_current_limit_t request, response;
  request.pkt_state = MOBO_CURRENT_LIMIT_STATE_REQUEST;
  request.current_limit = limit;
  if (!txPacket(CMD_ID_MOBO_SET_CURRENT_LIMIT, request))
    return false;
  if (!listenForPacketId(CMD_ID_MOBO_SET_CURRENT_LIMIT, 0.25, response))
    return false;
  if (response.pkt_state != MOBO_CURRENT_LIMIT_STATE_RESPONSE)
    return false;
  return true;
}

bool Hand::getHwVersion(uint32_t &version)
{
  get_hw_version_t request, response;
  request.pkt_state = MOBO_GET_HW_VERSION_REQUEST;
  request.version = 0;
  if (!txPacket(CMD_ID_MOBO_GET_HW_VERSION, request))
    return false;
  if (!listenForPacketId(CMD_ID_MOBO_GET_HW_VERSION, 0.25, response))
    return false;
  if (response.pkt_state != MOBO_GET_HW_VERSION_RESPONSE)
    return false;
  version = response.version;
  return true;
}

Hand::Side Hand::getSide()
{
  uint32_t hw_ver = 0;
  if (!getHwVersion(hw_ver))
    return Hand::UNKNOWN;
  if (((hw_ver >> 16) & 0xffff) != 0xbeef)
    return Hand::UNKNOWN;
  char side_ascii = (char)((hw_ver >> 8) & 0xff);
  if (side_ascii == 'R')
    return Hand::RIGHT;
  else if (side_ascii == 'L')
    return Hand::LEFT;
  return Hand::UNKNOWN;
}

