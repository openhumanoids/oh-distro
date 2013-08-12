#include "config.h"
static uint32_t g_bl_hw_version = 0; // bogus, will overwrite at startup

uint8_t config_init()
{
  g_bl_hw_version = *((uint32_t *)(0x00087ffc));
}

uint8_t config_is_left_hand()
{
  return ((char)((g_bl_hw_version >> 8) & 0xff)) == 'L';
}

uint32_t config_get_hw_version()
{
  return g_bl_hw_version;
}

const uint8_t *config_get_hand_mac()
{
  return (const uint8_t *)0x00087fec;
}

uint32_t config_get_hand_ip()
{
  return *((uint32_t *)(0x00087ff8));
}

uint32_t config_get_master_ip()
{
  return *((uint32_t *)(0x00087ff4));
}

