#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "sandia_hand/loose_finger.h"
#include "ros/time.h"
using namespace sandia_hand;

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

void listen_loose_finger(const float duration, LooseFinger &lf)
{
  ros::Time t_start(ros::Time::now());
  while (!g_done)
  {
    lf.listen(0.01);
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

int ping(int argc, char **argv, LooseFinger &lf)
{
  if (lf.mm.ping())
    printf("   motor module ping OK\n");
  else
    printf("   motor module ping fail\n");
  return 0;
}

int status(int argc, char **argv, LooseFinger &lf)
{
  if (lf.mm.pollFingerState())
    printf("finger state poll ok\n");
  else
    printf("finger state poll fail\n");
  return 0;
}

int pb(int argc, char **argv, LooseFinger &lf)
{
  const char *usage = "usage: loose_finger_cli SERIAL_DEV pb { ON | OFF }\n";
  verify_argc(argc, 4, usage);
  const char *s = argv[3];
  if (!strcmp(s, "on"))
    lf.mm.setPhalangeBusPower(true);
  else if (!strcmp(s, "off"))
    lf.mm.setPhalangeBusPower(false);
  else
    printf("%s\n", usage);
  return 0;
}

typedef struct
{
  uint32_t pp_tactile_time;
  uint32_t dp_tactile_time;
  uint32_t fmcb_time;
  uint16_t pp_tactile[6];
  uint16_t dp_tactile[12];
  int16_t  pp_imu[6];
  int16_t  dp_imu[6];
  int16_t  fmcb_imu[6];
  uint16_t pp_temp[4];
  uint16_t dp_temp[4];
  uint16_t fmcb_temp[3];
  uint16_t fmcb_voltage;
  uint16_t fmcb_pb_current;
  uint32_t pp_strain;
  int32_t  fmcb_hall_tgt[3];
  int32_t  fmcb_hall_pos[3];
  int16_t  fmcb_effort[3];
} finger_state_t;

void rxFingerState(const uint8_t *payload, const uint16_t payload_len)
{
  finger_state_t *fst = (finger_state_t *)payload;
  printf("rxFingerState\n");
  printf("  motor module imu: ");
  for (int i = 0; i < 6; i++)
    printf("%06d ", fst->fmcb_imu[i]);
  printf("\n  distal imu: ");
  for (int i = 0; i < 6; i++)
    printf("%06d ", fst->dp_imu[i]);
  printf("\n  distal tactile: ");
  for (int i = 0; i < 12; i++)
    printf("%06d ", fst->dp_tactile[i]);
  printf("\n  proximal imu: ");
  for (int i = 0; i < 6; i++)
    printf("%06d ", fst->pp_imu[i]);
  printf("\n  proximal tactile: ");
  for (int i = 0; i < 6; i++)
    printf("%06d ", fst->pp_tactile[i]);
  printf("\npb current: %d\n", fst->fmcb_pb_current);
  printf("\n\n");
}

int stream(int argc, char **argv, LooseFinger &lf)
{
  lf.mm.registerRxHandler(MotorModule::PKT_FINGER_STATUS,
                          boost::bind(rxFingerState, _1, _2));
  if (!lf.mm.setPhalangeAutopoll(true))
  {
    printf("couldn't start phalange autopoll\n");
    return 1;
  }
  while (!g_done)
  {
    listen_loose_finger(0.01, lf);
    lf.mm.pollFingerState();
  }
  lf.mm.setPhalangeAutopoll(false);
  return 0;
}

int pping(int argc, char **argv, LooseFinger &lf)
{
  if (lf.pp.ping())
    printf("proximal phalange ping ok\n");
  else
    printf("proximal phalange ping fail\n");
  return 0;
}

int pdump(int argc, char **argv, LooseFinger &lf)
{
  printf("powering down phalange bus...\n");
  lf.mm.setPhalangeBusPower(false);
  listen_loose_finger(1.0, lf);
  printf("powering up phalange bus...\n");
  lf.mm.setPhalangeBusPower(true);
  listen_loose_finger(2.0, lf);
  if (lf.pp.blHaltAutoboot())
    printf("autoboot halted\n");
  else
    printf("couldn't halt autoboot\n");
  FILE *f = fopen("image.bin", "wb");
  for (int page_num = 0; page_num < 16; page_num++)
  {
    bool page_read = false;
    for (int attempt = 0; !page_read && attempt < 10; attempt++)
    {
      printf("reading page %d attempt %d...\n", page_num, attempt);
      uint8_t page_buf[1024] = {0};
      if (lf.pp.blReadFlashPage(page_num, page_buf))
      {
        printf("read page 32:\n");
        page_read = true;
        if (256 != fwrite(page_buf, 1, 256, f))
        {
          printf("weird fwrite result\n");
        }
      }
      /*
      for (int i = 0; i < 256; i++)
      {
        printf("%02x  ", page_buf[i]);
        if (i % 8 == 7)
          printf("\n");
      }
      printf("\n");
      */
    }
    if (!page_read)
    {
      printf("couldn't read page %d\n", page_num);
      break;
    }
  }
  if (lf.pp.blBoot())
    printf("booted proximal phalange\n");
  else
    printf("failed to boot proximal phalange\n");
  return 0;
}

int jp(int argc, char **argv, LooseFinger &lf)
{
  verify_argc(argc, 6, "usage: jp J0 J1 J2");
  float j0 = atof(argv[3]), j1 = atof(argv[4]), j2 = atof(argv[5]);
  float joint_pos[3] = {j0, j1, j2};
  uint8_t max_efforts[3] = {50, 50, 50};
  lf.mm.setJointPos(joint_pos, max_efforts);
  return 0;
}


int pburn(int argc, char **argv, LooseFinger &lf)
{
  verify_argc(argc, 4, "usage: pburn FILENAME");
  const char *fn = argv[3];
  printf("burning binary image %s to proximal phalange...\n", fn);
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open file %s\n", fn);
    return 1;
  }
  printf("powering down phalange bus...\n");
  lf.mm.setPhalangeBusPower(false);
  listen_loose_finger(1.0, lf);
  printf("powering up phalange bus...\n");
  lf.mm.setPhalangeBusPower(true);
  listen_loose_finger(2.0, lf);
  if (!lf.pp.blHaltAutoboot())
  {
    printf("couldn't halt autoboot\n");
    return 1;
  }
  printf("autoboot halted\n");
  for (int page_num = 32; !g_done && !feof(f) && page_num < 256; page_num++)
  {
    bool page_written = false;
    uint8_t page_buf[1024] = {0};
    size_t nread = 0;
    nread = fread(page_buf, 1, 256, f);
    if (nread <= 0)
    {
      printf("couldn't read a flash page from %s: returned %d\n", 
             fn, (int)nread);
      break;
    }
    else if (nread < 256)
      printf("partial page: %d bytes, hopefully last flash page?\n", 
             (int)nread);
    if (lf.pp.blWriteFlashPage(page_num, page_buf, false))
      page_written = true;
    if (!page_written)
    {
      printf("couldn't write page %d\n", page_num);
      break;
    }
  }
  if (lf.pp.blBoot())
    printf("booted proximal phalange\n");
  else
    printf("failed to boot proximal phalange\n");
  return 0;
}

int dping(int argc, char **argv, LooseFinger &lf)
{
  if (lf.dp.ping())
    printf("distal phalange ping ok\n");
  else
    printf("distal phalange ping fail\n");
  return 0;
}

int ddump(int argc, char **argv, LooseFinger &lf)
{
  printf("powering down phalange bus...\n");
  lf.mm.setPhalangeBusPower(false);
  listen_loose_finger(1.0, lf);
  printf("powering up phalange bus...\n");
  lf.mm.setPhalangeBusPower(true);
  listen_loose_finger(2.0, lf);
  if (lf.dp.blHaltAutoboot())
    printf("distal autoboot halted\n");
  else
    printf("couldn't halt distal autoboot\n");
  FILE *f = fopen("image.bin", "wb");
  for (int page_num = 0; page_num < 16; page_num++)
  {
    bool page_read = false;
    for (int attempt = 0; !page_read && attempt < 10; attempt++)
    {
      printf("reading page %d attempt %d...\n", page_num, attempt);
      uint8_t page_buf[1024] = {0};
      if (lf.dp.blReadFlashPage(page_num, page_buf))
      {
        printf("read page 32:\n");
        page_read = true;
        if (256 != fwrite(page_buf, 1, 256, f))
        {
          printf("weird fwrite result\n");
        }
      }
      /*
      for (int i = 0; i < 256; i++)
      {
        printf("%02x  ", page_buf[i]);
        if (i % 8 == 7)
          printf("\n");
      }
      printf("\n");
      */
    }
    if (!page_read)
    {
      printf("couldn't read page %d\n", page_num);
      break;
    }
  }
  if (lf.dp.blBoot())
    printf("booted distal phalange\n");
  else
    printf("failed to boot distal phalange\n");
  return 0;
}

int dburn(int argc, char **argv, LooseFinger &lf)
{
  verify_argc(argc, 4, "usage: dburn FILENAME");
  const char *fn = argv[3];
  printf("burning binary image %s to distal phalange...\n", fn);
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open file %s\n", fn);
    return 1;
  }
  printf("powering down phalange bus...\n");
  lf.mm.setPhalangeBusPower(false);
  listen_loose_finger(1.0, lf);
  printf("powering up phalange bus...\n");
  lf.mm.setPhalangeBusPower(true);
  listen_loose_finger(2.0, lf);
  if (!lf.dp.blHaltAutoboot())
  {
    printf("couldn't halt distal autoboot\n");
    return 1;
  }
  printf("distal autoboot halted\n");
  for (int page_num = 32; !g_done && !feof(f) && page_num < 256; page_num++)
  {
    bool page_written = false;
    uint8_t page_buf[1024] = {0};
    size_t nread = 0;
    nread = fread(page_buf, 1, 256, f);
    if (nread <= 0)
    {
      printf("couldn't read a flash page from %s: returned %d\n", 
             fn, (int)nread);
      break;
    }
    else if (nread < 256)
      printf("partial page: %d bytes, hopefully last flash page?\n", 
             (int)nread);
    if (lf.dp.blWriteFlashPage(page_num, page_buf, false))
      page_written = true;
    if (!page_written)
    {
      printf("couldn't write page %d\n", page_num);
      break;
    }
  }
  if (lf.dp.blBoot())
    printf("booted distal phalange\n");
  else
    printf("failed to boot distal phalange\n");
  return 0;
}

int dump(int argc, char **argv, LooseFinger &lf)
{
  printf("resetting motor module...\n");
  lf.mm.reset();
  listen_loose_finger(2.0, lf);
  if (lf.mm.blHaltAutoboot())
    printf("motor module autoboot halted\n");
  else
    printf("couldn't halt motor module autoboot\n");
  FILE *f = fopen("image.bin", "wb");
  for (int page_num = 0; page_num < 256; page_num++)
  {
    bool page_read = false;
    for (int attempt = 0; !page_read && attempt < 10; attempt++)
    {
      printf("reading page %d attempt %d...\n", page_num, attempt);
      uint8_t page_buf[1024] = {0};
      if (lf.mm.blReadFlashPage(page_num, page_buf))
      {
        printf("read page 32:\n");
        page_read = true;
        if (256 != fwrite(page_buf, 1, 256, f))
        {
          printf("weird fwrite result\n");
        }
      }
      /*
      for (int i = 0; i < 256; i++)
      {
        printf("%02x  ", page_buf[i]);
        if (i % 8 == 7)
          printf("\n");
      }
      printf("\n");
      */
    }
    if (!page_read)
    {
      printf("couldn't read page %d\n", page_num);
      break;
    }
  }
  /*
  if (lf.mm.blBoot())
    printf("booted motor module\n");
  else
    printf("failed to boot motor module\n");
  */
  return 0;
}

int ver(int argc, char **argv, LooseFinger &lf)
{
  const uint32_t mm_ver = (uint32_t)lf.mm.getHardwareVersion();
  printf("motor module version: %08x\n", mm_ver);
  return 0;
}

bool reset_finger(LooseFinger &lf)
{
  lf.mm.reset();
  return true; // return true even if we couldn't ack the reset request
}

bool fake_set_finger_power()
{
  return true;
}


int burn(int argc, char **argv, LooseFinger &lf)
{
  verify_argc(argc, 3, "usage: mmburn MM_BIN_FILE\n");
  const char *fn = argv[3];
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open motor module application image %s\n", fn);
    return 1;
  }
  if (!lf.mm.programAppFile(f, boost::bind(reset_finger, boost::ref(lf)), 
                            fake_set_finger_power))
  {
    printf("failed to program motor module with image %s\n", fn);
    return 1;
  }
  printf("successfully programmed motor module with image %s\n", fn);
  if (lf.mm.blBoot())
    printf("booted motor module\n");
  else
    printf("failed to boot motor module\n");
  return 0;
#if 0
  verify_argc(argc, 4, "usage: burn FILENAME");
  const char *fn = argv[3];
  printf("burning binary image %s to motor module...\n", fn);
  FILE *f = fopen(fn, "rb");
  if (!f)
  {
    printf("couldn't open file %s\n", fn);
    return 1;
  }
  if (fseek(f, 32 * 256, SEEK_SET))
  {
    printf("couldn't seek to application image in %s\n", fn);
    return 1;
  }
  printf("resetting motor module...\n");
  lf.mm.reset();
  listen_loose_finger(2.0, lf);
  if (!lf.mm.blHaltAutoboot())
  {
    printf("couldn't halt motor module autoboot\n");
    return 1;
  }
  printf("distal autoboot halted\n");
  for (int page_num = 32; !g_done && !feof(f) && page_num < 256; page_num++)
  {
    bool page_written = false;
    uint8_t page_buf[1024] = {0};
    size_t nread = 0;
    nread = fread(page_buf, 1, 256, f);
    if (nread <= 0)
    {
      printf("couldn't read a flash page from %s: returned %d\n", 
             fn, (int)nread);
      break;
    }
    else if (nread < 256)
      printf("partial page: %d bytes, hopefully last flash page?\n", 
             (int)nread);
    if (lf.mm.blWriteFlashPage(page_num, page_buf, false))
      page_written = true;
    if (!page_written)
    {
      printf("couldn't write page %d\n", page_num);
      break;
    }
  }
  if (lf.mm.blBoot())
    printf("booted motor module\n");
  else
    printf("failed to boot motor module\n");
  return 0;
#endif
}



int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: loose_finger_cli SERIAL_DEVICE COMMAND [OPTIONS]\n");
    return 1;
  }
  ros::Time::init();
  LooseFinger lf;
  if (!lf.init(argv[1]))
  {
    printf("couldn't init hand\n");
    return false;
  }
  signal(SIGINT, signal_handler);
  const char *cmd = argv[2];
  if (!strcmp(cmd, "ping"))
    return ping(argc, argv, lf);
  if (!strcmp(cmd, "status"))
    return status(argc, argv, lf);
  if (!strcmp(cmd, "stream"))
    return stream(argc, argv, lf);
  if (!strcmp(cmd, "pb"))
    return pb(argc, argv, lf);
  if (!strcmp(cmd, "pping"))
    return pping(argc, argv, lf);
  if (!strcmp(cmd, "pdump"))
    return pdump(argc, argv, lf);
  if (!strcmp(cmd, "pburn"))
    return pburn(argc, argv, lf);
  if (!strcmp(cmd, "dping"))
    return dping(argc, argv, lf);
  if (!strcmp(cmd, "ddump"))
    return ddump(argc, argv, lf);
  if (!strcmp(cmd, "dburn"))
    return dburn(argc, argv, lf);
  if (!strcmp(cmd, "dump"))
    return dump(argc, argv, lf);
  if (!strcmp(cmd, "burn"))
    return burn(argc, argv, lf);
  if (!strcmp(cmd, "jp"))
    return jp(argc, argv, lf);
  if (!strcmp(cmd, "ver"))
    return ver(argc, argv, lf);

  printf("unknown command: [%s]\n", cmd);
  return 1;
}

