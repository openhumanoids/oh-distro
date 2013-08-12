#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include "sandia_hand/hand.h"
#include "ros/time.h"
#include <string>
#include "sandia_hand/palm_state.h"
using namespace sandia_hand;
using std::vector;
using std::string;
using std::map;

static bool g_done = false;
void signal_handler(int signum)
{
  if (signum == SIGINT)
    g_done = true;
}

//////////////////////////////////////////////////////////////////////////////
// some helper functions to condense everything....

int b2e(bool b) // convert boolean function return values to exit codes
{
  return b ? 0 : 1;
}

void parse_finger_idx(uint8_t &finger_idx, const char *s)
{
  finger_idx = atoi(s);
  if (finger_idx >= 4)
  {
    printf("finger_idx must be in {0,1,2,3}\n");
    exit(1);
  }
}

void listen_hand(const float duration, Hand &hand)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    hand.listen(0.01);
    if ((ros::Time::now() - t_start).toSec() > duration)
      break;
  }
}

bool verify_argc(const int argc, const int min_argc, const char *usage_text)
{
  if (argc < min_argc)
  {
    printf("%s\n", usage_text);
    exit(1);
    return false; // never gets here... but feels good to write it still
  }
  return true;
}

//////////////////////////////////////////////////////////////////////////////
// command handlers

// set all finger powers
int p(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 3, "usage: hand_cli p POWER_STATE\n"
                       "  where POWER_STATE = {off, low, on}\n");
  const char *fp_str = argv[2];
  Hand::FingerPowerState fps;
  if (!strcmp(fp_str, "off"))
    fps = Hand::FPS_OFF;
  else if (!strcmp(fp_str, "low"))
    fps = Hand::FPS_LOW;
  else if (!strcmp(fp_str, "on"))
    fps = Hand::FPS_FULL;
  else
  {
    printf("unrecognized power state [%s]\n", fp_str);
    printf("power_state must be in {off, low, on}\n");
    return 1;
  }
  for (int i = 0; i < 4; i++)
  {
    hand.setFingerPower(i, fps);
    usleep(250000); // wait 250ms so we don't thrash power supply too much
  }
  return 0;
}

// fping
int fping(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 3, "usage: fping FINGER_IDX\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  printf("pinging finger %d:\n", finger_idx);
  printf("  motor module: ");
  if (hand.fingers[finger_idx].mm.ping())
    printf("OK\n");
  else
    printf("fail\n");
  printf("  proximal phalange: ");
  if (hand.fingers[finger_idx].pp.ping())
    printf("OK\n");
  else
    printf("fail\n");
  printf("  distal phalange: ");
  if (hand.fingers[finger_idx].dp.ping())
    printf("OK\n");
  else
    printf("fail\n");
  return 0;
}

int palm_ping(int argc, char **argv, Hand &hand)
{
  printf("pinging palm\n");
  if (hand.palm.ping())
    printf("   OK\n");
  else
    printf("   fail\n");
  listen_hand(0.5, hand);
  return 0;
}

// set single finger power
int fp(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 4, "usage: hand_cli fp FINGER_IDX POWER_STATE\n"
                       "  where POWER_STATE = {off, low, on}\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  const char *fp_str = argv[3];
  Hand::FingerPowerState fps;
  if (!strcmp(fp_str, "off"))
    fps = Hand::FPS_OFF;
  else if (!strcmp(fp_str, "low"))
    fps = Hand::FPS_LOW;
  else if (!strcmp(fp_str, "on"))
    fps = Hand::FPS_FULL;
  else
  {
    printf("unrecognized power state [%s]\n", fp_str);
    printf("power_state must be in {off, low, on}\n");
    return 1;
  }
  hand.setFingerPower(finger_idx, fps);
  return 0;
}

int pp(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 4, "usage: hand_cli pp FINGER_IDX POWER_STATE\n"
                       "  where POWER_STATE = {off, on}\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  const char *fp_str = argv[3];
  bool bus_on;
  if (!strcmp(fp_str, "off"))
    bus_on = false;
  else if (!strcmp(fp_str, "on"))
    bus_on = true;
  else
  {
    printf("unrecognized phalange power state [%s]\n", fp_str);
    printf("power_state must be in {off, on}\n");
    return 1;
  }
  if (!hand.fingers[finger_idx].mm.setPhalangeBusPower(bus_on))
    printf("couldn't set bus power to %d\n", (int)bus_on);
  else
    printf("set bus power to %d\n", (int)bus_on);
  return 0;
}

// set finger control mode
int fcm(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 4, "usage: hand_cli fcm FINGER_IDX CONTROL_MODE\n"
                       "  where CONTROL_MODE = {idle, joint_pos}\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  const char *fcm_str = argv[3];
  Hand::FingerControlMode fcm;
  if (!strcmp(fcm_str, "idle"))
    fcm = Hand::FCM_IDLE;
  else if (!strcmp(fcm_str, "joint_pos"))
    fcm = Hand::FCM_JOINT_POS;
  else
  {
    printf("unrecognized finger control state [%s]\n", fcm_str);
    printf("finger control state must be in {idle, joint_pos}\n");
    return 1;
  }
  hand.setFingerControlMode(finger_idx, fcm);
  return 0;
}

// set joint position
int jp(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 6, "usage: hand_cli jp FINGER_IDX J0 J1 J2\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  float j0 = atof(argv[3]), j1 = atof(argv[4]), j2 = atof(argv[5]);
  hand.setFingerJointPos(finger_idx, j0, j1, j2);
  return 0;
}

void cam_pgm_cb(const uint8_t cam_idx, const uint32_t frame_count, 
                const uint8_t *img_data)
{
  printf("cam_pgm_cb\n");
  FILE *f = NULL;
  char fname_buf[100];
  snprintf(fname_buf, sizeof(fname_buf), "cam_%d_img_%06d.pgm", 
           cam_idx, frame_count);
  f = fopen(fname_buf, "w");
  fprintf(f, "P5\n%d %d\n255\n", Hand::IMG_WIDTH, Hand::IMG_HEIGHT);
  fwrite(img_data, 1, Hand::IMG_WIDTH * Hand::IMG_HEIGHT, f);
  g_done = true; // got the image. bail now from spin loop
  printf("wrote %s\n", fname_buf);
  fclose(f);
}

int cam_pgm(int argc, char **argv, Hand &hand)
{
  printf("saving one pgm image from the hand...\n");
  hand.setImageCallback(&cam_pgm_cb); 
  hand.setCameraStreaming(true, true);
  ros::Time t_start(ros::Time::now());
  while (!g_done && (ros::Time::now() - t_start).toSec() < 10)
    if (!hand.listen(1.0))
      break;
  hand.setCameraStreaming(false, false);
  return 0;
}

int test_finger_currents(int argc, char **argv, Hand &hand)
{
  printf("testing finger currents during boot cycle...\n");
  hand.setMoboStateHz(1);
  ros::Time t_start(ros::Time::now());
  bool finger_states[4] = {false, false, false, false};
  int next_finger_powerup = 0;
  while (!g_done)
  {
    hand.listen(1.0);
    if (((ros::Time::now() - t_start).toSec() > next_finger_powerup * 5) &&
        !finger_states[next_finger_powerup])
    {
      // turn on a finger to observe current ramp
    }
  }
  hand.setMoboStateHz(0);
  return 0;
}

void mobo_state_rx(const uint8_t *data, const uint16_t data_len)
{
  static FILE *f_log = NULL;
  if (!f_log)
    f_log = fopen("current_log.txt", "w");
  //printf("mobo status\n");
  const mobo_state_t *p = (mobo_state_t *)data;
  printf("\n\n  mobo time: %d\n", p->mobo_time_ms);
  fprintf(f_log, "%d ", p->mobo_time_ms);
  for (int i = 0; i < 4; i++)
  {
    printf("  %d current: %.4f\n", i, p->finger_currents[i]);
    fprintf(f_log, "%.6f ", p->finger_currents[i]);
  }
  for (int i = 0; i < 3; i++)
  {
    printf("  logic %d current: %.4f\n", i, p->logic_currents[i]);
    fprintf(f_log, "%.6f ", p->logic_currents[i]);
  }
  for (int i = 0; i < 3; i++)
  {
    printf("  %d raw temperature: %d\n", i, p->mobo_raw_temperatures[i]);
    fprintf(f_log, "%d ", (int16_t)p->mobo_raw_temperatures[i]);
  }
  printf("  mobo max effort: %d\n", p->mobo_max_effort);
  fprintf(f_log, "\n");
}

int test_finger_stream(int argc, char **argv, Hand &hand)
{
  hand.registerRxHandler(CMD_ID_MOBO_STATUS, mobo_state_rx);
  printf("turning on mobo state streaming...\n");
  hand.setMoboStateHz(100);
  //listen_hand(1.0, hand);
  /*
  printf("powering finger sockets...\n");
  hand.setAllFingerPowers(Hand::FPS_LOW);
  listen_hand(0.5, hand);
  hand.setAllFingerPowers(Hand::FPS_FULL);
  listen_hand(4.0, hand);
  printf("turning on phalange bus...\n");
  hand.fingers[0].mm.setPhalangeBusPower(true);
  listen_hand(4.0, hand);
  hand.fingers[0].mm.setPhalangeAutopoll(true);

  printf("turning on finger streaming...\n");
  hand.setFingerAutopollHz(1);
  printf("turning off finger power...\n");
  hand.setAllFingerPowers(Hand::FPS_OFF);
  hand.setFingerAutopollHz(0);
  */
  while (!g_done)
    listen_hand(0.1, hand);
  hand.setMoboStateHz(0);
  usleep(200000);
  printf("bye\n");
  return 0;
}

void palmStateRx(const uint8_t *data, const uint16_t data_len)
{
  printf("palmStateRx\n");
  palm_state_t *p = (palm_state_t *)data;
  printf("  time: %d\n", p->palm_time);
  printf("  tactile: %d %d\n", p->palm_tactile[0], p->palm_tactile[1]);
}

int palm_stream(int argc, char **argv, Hand &hand)
{
  //hand.registerRxHandler(CMD_ID_MOBO_STATUS, mobo_status_rx);
  hand.palm.registerRxHandler(Palm::PKT_PALM_STATE, palmStateRx);
  printf("turning on palm state streaming...\n");
  hand.setFingerAutopollHz(2);
  //listen_hand(1.0, hand);
  /*
  printf("powering finger sockets...\n");
  hand.setAllFingerPowers(Hand::FPS_LOW);
  listen_hand(0.5, hand);
  hand.setAllFingerPowers(Hand::FPS_FULL);
  listen_hand(4.0, hand);
  printf("turning on phalange bus...\n");
  hand.fingers[0].mm.setPhalangeBusPower(true);
  listen_hand(4.0, hand);
  hand.fingers[0].mm.setPhalangeAutopoll(true);

  printf("turning on finger streaming...\n");
  hand.setFingerAutopollHz(1);
  printf("turning off finger power...\n");
  hand.setAllFingerPowers(Hand::FPS_OFF);
  hand.setFingerAutopollHz(0);
  */
  while (!g_done)
    listen_hand(0.1, hand);
  hand.setFingerAutopollHz(0);
  usleep(200000);
  printf("bye\n");
  return 0;
}


int f3burn(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 4, "usage: f3burn FINGER_IDX F3_BIN_FILE\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  const char *fn = argv[3];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open application image %s\n", fn);
    return 1;
  }
  if (!hand.programDistalPhalangeAppFile(finger_idx, f))
  {
    printf("failed to program with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed image %s\n", fn);
  return 0;
}

int f2burn(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 4, "usage: f2burn FINGER_IDX F2_BIN_FILE\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  const char *fn = argv[3];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open application image %s\n", fn);
    return 1;
  }
  if (!hand.programProximalPhalangeAppFile(finger_idx, f))
  {
    printf("failed to program with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed image %s\n", fn);
  return 0;
}

int mmburn(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 4, "usage: mmburn FINGER_IDX MM_BIN_FILE\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  const char *fn = argv[3];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open motor module application image %s\n", fn);
    return 1;
  }
  if (!hand.programMotorModuleAppFile(finger_idx, f))
  {
    printf("failed to program motor module with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed motor module %d with image %s\n", 
         finger_idx, fn);
  return 0;
}

int mmburn_all(int argc, char **argv, Hand &hand) // todo: merge with previous
{
  verify_argc(argc, 3, "usage: mmburn MM_BIN_FILE\n");
  const char *fn = argv[2];
  for (uint8_t finger_idx = 0; finger_idx < 4; finger_idx++)
  {
    FILE *f = fopen(fn, "rb");
    if (!f)
    {
      printf("couldn't open motor module application image %s\n", fn);
      return 1;
    }
    printf("programming finger %d...\n", finger_idx);
    if (!hand.programMotorModuleAppFile(finger_idx, f))
    {
      printf("failed to program motor module %d with image %s\n", 
             finger_idx, fn);
      return 1;
    }
    printf("successfully programmed motor module %d with image %s\n", 
           finger_idx, fn);
    fclose(f);
  }
  return 0;
}

int palmburn(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 3, "usage: palmburn PALM_BIN_FILE\n");
  const char *fn = argv[2];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open application image %s\n", fn);
    return 1;
  }
  if (!hand.programPalmAppFile(f))
  {
    printf("failed to program with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed image %s\n", fn);
  return 0;
}

void print_page(const vector<uint8_t> &v)
{
  if (v.size() != 256)
  {
    printf("invalid page size: %d\n", (int)v.size());
    return;
  }
  for (int i = 0; i < 256; i++)
  {
    printf("0x%02x  ", v[i]);
    if (i % 8 == 7)
      printf("\n");
  }
}

int mflash_read(int argc, char **argv, Hand &hand)
{
  if (argc != 3)
  {
    printf("usage: mflash_read PAGE_NUM\n");
    return 1;
  }
  vector<uint8_t> page_data;
  if (!hand.readMoboFlashPage(atoi(argv[2]), page_data))
  {
    printf("Hand::readMoboFlashPage failed\n");
    return 1;
  }
  printf("Hand::readMoboFlashPage succeeded\n");
  print_page(page_data);
  return 0;
}

int mmcu_read(int argc, char **argv, Hand &hand)
{
  if (argc != 3)
  {
    printf("usage: mmcu_read PAGE_NUM\n");
    return 1;
  }
  vector<uint8_t> page_data;
  if (!hand.readMoboMCUPage(atoi(argv[2]), page_data))
  {
    printf("Hand::readMoboMCUPage failed\n");
    return 1;
  }
  printf("Hand::readMoboMCUPage succeeded\n");
  print_page(page_data);
  return 0;
}

int mflash_test(int argc, char **argv, Hand &hand)
{
  vector<uint8_t> page_data;
  const uint32_t page_num = 32768;
  if (!hand.readMoboFlashPage(page_num, page_data))
  {
    printf("initial page read fail\n");
    return 1;
  }
  printf("page contents at start:\n");
  print_page(page_data);
  if (!hand.eraseMoboFlashSector(page_num))
  {
    printf("couldn't erase sector\n");
    return 1;
  }
  if (!hand.readMoboFlashPage(page_num, page_data))
  {
    printf("post-erase page read fail\n");
    return 1;
  }
  printf("page contents after erase:\n");
  print_page(page_data);
  for (int i = 0; i < 256; i++)
    page_data[i] = i;
  printf("about to write page:\n");
  print_page(page_data);
  if (!hand.writeMoboFlashPage(page_num, page_data))
  {
    printf("page write fail\n");
    return 1;
  }
  if (!hand.readMoboFlashPage(page_num, page_data))
  {
    printf("post-write page read fail\n");
    return 1;
  }
  printf("page contents readback:\n");
  print_page(page_data);
  return 0;
}

int mmcu_test(int argc, char **argv, Hand &hand)
{
  vector<uint8_t> page_data;
  const uint32_t page_num = 1512; // land in bank 1
  if (!hand.readMoboMCUPage(page_num, page_data))
  {
    printf("initial page read fail\n");
    return 1;
  }
  printf("page contents at start:\n");
  print_page(page_data);
  for (int i = 0; i < 256; i++)
    page_data[i] = i;
  printf("about to write page %d:\n", page_num);
  print_page(page_data);
  if (!hand.writeMoboMCUPage(page_num, page_data))
  {
    printf("page write fail\n");
    return 1;
  }
  if (!hand.readMoboMCUPage(page_num, page_data))
  {
    printf("post-write page read fail\n");
    return 1;
  }
  printf("page contents readback:\n");
  print_page(page_data);
  return 0;
}

int mflash_burn_golden_fpga(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 3, "usage: mflash_burn_golden_fpga FPGA_BIN_FILE\n");
  const char *fn = argv[2];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open golden image %s\n", fn);
    return 1;
  }
  if (!hand.programFPGAGoldenFile(f))
  {
    printf("failed to program with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed golden image %s\n", fn);
  return 0;
}

int mflash_burn_fpga(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 3, "usage: mflash_burn_fpga FPGA_BIN_FILE\n");
  const char *fn = argv[2];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open application image %s\n", fn);
    return 1;
  }
  if (!hand.programFPGAAppFile(f))
  {
    printf("failed to program with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed application image %s\n", fn);
  return 0;
}

int mmcu_dump(int argc, char **argv, Hand &hand)
{
  if (argc != 4)
  {
    printf("usage: mmcu_dump START_PAGE NUMBER_OF_PAGES\n");
    return 1;
  }
  vector<uint8_t> page_data;
  page_data.resize(256);
  const int start_page = atoi(argv[2]), n_pages = atoi(argv[3]);
  const char *fn = "dump.bin";
  FILE *f = fopen(fn, "wb");
  for (int page_num = start_page; page_num < start_page + n_pages; page_num++)
  {
    if (!hand.readMoboMCUPage(page_num, page_data))
    {
      printf("couldn't read page %d\n", page_num);
      return 1;
    }
    int n_written = fwrite(&page_data[0], 1, 256, f);
    if (n_written != 256)
    {
      printf("error writing page %d to %s\n", page_num, fn);
      return 1;
    }
  }
  fclose(f);
  printf("%d pages (%d bytes) written to %s\n", 
         n_pages, n_pages * 256, fn);
  return 0;
}

int mflash_dump(int argc, char **argv, Hand &hand)
{
  if (argc != 4)
  {
    printf("usage: mflash_read START_PAGE NUMBER_OF_PAGES\n");
    return 1;
  }
  vector<uint8_t> page_data;
  page_data.resize(256);
  const int start_page = atoi(argv[2]), n_pages = atoi(argv[3]);
  const char *fn = "dump.bin";
  FILE *f = fopen(fn, "wb");
  for (int page_num = start_page; page_num < start_page + n_pages; page_num++)
  {
    if (!hand.readMoboFlashPage(page_num, page_data))
    {
      printf("couldn't read page %d\n", page_num);
      return 1;
    }
    int n_written = fwrite(&page_data[0], 1, 256, f);
    if (n_written != 256)
    {
      printf("error writing page %d to %s\n", page_num, fn);
      return 1;
    }
  }
  fclose(f);
  printf("%d pages (%d bytes) written to %s\n", 
         n_pages, n_pages * 256, fn);
  return 0;

}

int mmcu_burn(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 3, "usage: mmcu_burn BIN_FILE\n");
  const char *fn = argv[2];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open application image %s\n", fn);
    return 1;
  }
  if (!hand.programMoboMCUAppFile(f))
  {
    printf("failed to program with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed mobo MCU with application image %s\n", fn);
  return 0;
}

int mmcu_ping(int argc, char **argv, Hand &hand)
{
  if (!hand.pingMoboMCU())
  {
    printf("couldn't ping mobo MCU\r\n");
    return 1;
  }
  printf("successfully pinged mobo MCU\r\n");
  return 0;
}

int mm_param_dump(int argc, char **argv, Hand &hand)
{
  verify_argc(argc, 3, "usage: mm_param_dump FINGER_IDX\n");
  uint8_t finger_idx = 0;
  parse_finger_idx(finger_idx, argv[2]);
  vector<string> param_names;
  if (!hand.fingers[finger_idx].mm.getParamNames(param_names))
  {
    printf("couldn't get param names\n");
    return 1;
  }
  printf("found %d parameters:\n", (int)param_names.size());
  for (int i = 0; i < (int)param_names.size(); i++)
  {
    printf("  %02d: %s\n", i, param_names[i].c_str());
  }
  return 0;
}

int mobo_version(int argc, char **argv, Hand &hand)
{
  uint32_t version = 0;
  if (!hand.getHwVersion(version))
  {
    printf("couldn't get hardware version\n");
    return 1;
  }
  printf("mobo hardware version: 0x%08x\n", version);
  printf("  hardware rev: %c\n", (char)(version & 0xff));
  printf("  side: %c\n", (char)((version >> 8) & 0xff));
  printf("  enum: %d\n", (int)hand.getSide());
  return 0;
}

///////////////////////

#define CLI_FUNC(x) do { cmds[string(#x)] = x; } while (0)

void usage()
{
  printf("usage: sandia_hand_cli SIDE CMD [OPTS}\n");
  printf("   where SIDE = {left, right}\n");
  exit(1);
}

const char *magic_env_name  = "I_KNOW_WHAT_I_AM_DOING";
const char *magic_env_value = "YES";

int main(int argc, char **argv)
{
  if (!getenv(magic_env_name) ||
      strcasecmp(getenv(magic_env_name), magic_env_value))
  {
    printf("\nWoah there partner. This program can do a lot of things to the\n"
           "hand, including brick it, which would require recovery via a \n"
           "JTAG dongle. If you know what you are doing in here, please \n"
           "set I_KNOW_WHAT_I_AM_DOING to \"YES\" in the environment and \n"
           "try again.\n\n");
    exit(1);
  }
  ros::Time::init();
  Hand hand;
  if (argc < 2)
    usage();
  string ip("10.10.1.2");
  if (!strcmp(argv[1], "left"))
    ip = string("10.66.171.22");
  else if (!strcmp(argv[1], "right"))
    ip = string("10.66.171.23");
  else
    usage();

  if (!hand.init(ip.c_str()))
  {
    printf("bogus. couldn't init hand.\n");
    return false;
  }
  signal(SIGINT, signal_handler);
  typedef boost::function<int(int, char **, Hand &) > cli_cmd_t;
  map<string, cli_cmd_t> cmds;
  // todo: fun challenge: make the function declaration macro automatically
  // do this, instead of having to instantiate macros here
  CLI_FUNC(p);
  CLI_FUNC(fp);
  CLI_FUNC(pp);
  CLI_FUNC(fcm);
  CLI_FUNC(jp);
  CLI_FUNC(cam_pgm);
  CLI_FUNC(fping);
  CLI_FUNC(palm_ping);
  CLI_FUNC(test_finger_currents);
  CLI_FUNC(test_finger_stream);
  CLI_FUNC(mmburn);
  CLI_FUNC(mmburn_all);
  CLI_FUNC(f2burn);
  CLI_FUNC(f3burn);
  CLI_FUNC(palmburn);
  CLI_FUNC(mflash_read);
  CLI_FUNC(mflash_test);
  CLI_FUNC(mflash_burn_golden_fpga);
  CLI_FUNC(mflash_burn_fpga);
  CLI_FUNC(mflash_dump);
  CLI_FUNC(mmcu_read);
  CLI_FUNC(mmcu_dump);
  CLI_FUNC(mmcu_test);
  CLI_FUNC(mmcu_burn);
  CLI_FUNC(mmcu_ping);
  CLI_FUNC(mm_param_dump);
  CLI_FUNC(palm_stream);
  CLI_FUNC(mobo_version);
  if (argc <= 2)
  {
    printf("available commands:\n");
    std::pair<string, cli_cmd_t> cmd;
    BOOST_FOREACH(cmd, cmds)
      printf("  %s\n", cmd.first.c_str());
    return 0;
  }
  string cli_cmd = string(argv[2]);
  if (cmds.find(cli_cmd) == cmds.end())
  {
    printf("unknown command: %s\n", cli_cmd.c_str());
    return 1;
  }
  return cmds[cli_cmd](argc-1, argv+1, hand);
}

