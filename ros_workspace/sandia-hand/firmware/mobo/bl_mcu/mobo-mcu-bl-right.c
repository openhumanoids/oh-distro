#include "stdint.h"
const uint32_t __attribute__((section(".bl_hw_version")))
                                g_bl_hw_version = 0xbeef5241; // 0x52 = 'R'
const uint32_t __attribute__((section(".bl_master_ip")))
                                g_bl_master_ip = 0x0a42ab14; // 10.66.171.20
const uint32_t __attribute__((section(".bl_hand_ip")))
                                g_bl_hand_ip   = 0x0a42ab17; // 10.66.171.23
const uint8_t __attribute__((section(".bl_hand_mac")))
                   g_bl_hand_mac[6] = { 0xa4, 0xf3, 0xc1, 0x00, 0x00, 0x01 };
