/*  Software License Agreement (Apache License)
 *
 *  Copyright 2012 Open Source Robotics Foundation
 *  Author: Morgan Quigley
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

// lots of this originally snarfed from an Atmel sam3s distro, and much changed
#include "stdint.h"
#include "common_sam3x/sam3x.h"
#include "enet.h"
#include "clocks.h"

/* Stack Configuration */  
#define STACK_SIZE       0x1000     /** Stack size (in DWords) */
__attribute__ ((aligned(8),section(".stack")))
uint32_t pdwStack[STACK_SIZE] ;     
#define IRAM_ADDR 0x20000000

/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;

extern void main( void ) ;
extern void systick_vector(void);

void reset_vector() ;
extern void __libc_init_array( void ) ;

void unmapped_vector()
{
  while (1) { } // spin here to allow JTAG trap
}

typedef void (*IntFunc)(void);

__attribute__((section(".vectors")))
IntFunc exception_table[] = {
    /* Configure Initial Stack Pointer, using linker-generated symbols */
    (IntFunc)(&pdwStack[STACK_SIZE-1]),
    reset_vector,  //0x004000d1,     //_bootloader_start,
    unmapped_vector, // NMI_Handler,
    unmapped_vector, // HardFault_Handler,
    unmapped_vector, // MemManage_Handler,
    unmapped_vector, // BusFault_Handler,
    unmapped_vector, // UsageFault_Handler,
    0, 0, 0, 0,      // Reserved
    unmapped_vector, // SVC_Handler,
    unmapped_vector, // DebugMon_Handler,
    0,               // Reserved
    unmapped_vector, // PendSV_Handler,
    systick_vector,  // SysTick_Handler,
    // SAM3X configurable interrupts begin here...
    // (add 16 to the numbers below to get the ARM ISR number)
    unmapped_vector, // 0  SUPC  Supply Controller 
    unmapped_vector, // 1  RSTC  Reset Controller 
    unmapped_vector, // 2  RTC   Real Time Clock 
    unmapped_vector, // 3  RTT   Real Time Timer 
    unmapped_vector, // 4  WDT   Watchdog Timer 
    unmapped_vector, // 5  PMC 
    unmapped_vector, // 6  EEFC0
    unmapped_vector, // 7  EEFC1
    unmapped_vector, // 8  UART0 
    unmapped_vector, // 9  SMC 
    unmapped_vector, // 10 reserved
    unmapped_vector, // 11 PIOA
    unmapped_vector, // 12 PIOB
    unmapped_vector, // 13 PIOC
    unmapped_vector, // 14 PIOD
    unmapped_vector, // 15 PIOE (not used)
    unmapped_vector, // 16 PIOF (not used)
    unmapped_vector, // 17 USART0
    unmapped_vector, // 18 USART1
    unmapped_vector, // 19 USART2 
    unmapped_vector, // 20 USART3 
    unmapped_vector, // 21 HSMCI
    unmapped_vector, // 22 TWI0
    unmapped_vector, // 23 TWI1
    unmapped_vector, // 24 SPI0
    unmapped_vector, // 25 SPI1
    unmapped_vector, // 26 SSC 
    unmapped_vector, // 27 TC0
    unmapped_vector, // 28 TC1
    unmapped_vector, // 29 TC2
    unmapped_vector, // 30 TC3
    unmapped_vector, // 31 TC4
    unmapped_vector, // 32 TC5
    unmapped_vector, // 33 TC6
    unmapped_vector, // 34 TC7
    unmapped_vector, // 35 TC8
    unmapped_vector, // 36 PWM
    unmapped_vector, // 37 ADC
    unmapped_vector, // 38 DACC
    unmapped_vector, // 39 DMAC
    unmapped_vector, // 40 UOTGHS
    unmapped_vector, // 41 TRNG
    enet_vector,     // 42 EMAC
    unmapped_vector, // 43 CAN0
    unmapped_vector, // 44 CAN1
};

void reset_vector( void )
{
  uint32_t *pSrc, *pDest ;
  //LowLevelInit() ;
  //EFC0->EEFC_FMR = EEFC_FMR_FWS(3);
  //EFC1->EEFC_FMR = EEFC_FMR_FWS(3);
  clocks_init();
  // set up data segment
  pSrc = &_etext ;
  pDest = &_srelocate ;
  if ( pSrc != pDest )
    for ( ; pDest < &_erelocate ; )
      *pDest++ = *pSrc++ ;
  // set up bss segment
  for ( pDest = &_szero ; pDest < &_ezero ; )
    *pDest++ = 0;
  // set vector table base address
  pSrc = (uint32_t *)&_sfixed;
  SCB->VTOR = ( (uint32_t)pSrc & SCB_VTOR_TBLOFF_Msk ) ;
  if ( ((uint32_t)pSrc >= IRAM_ADDR) && ((uint32_t)pSrc < IRAM_ADDR+IRAM_SIZE) )
    SCB->VTOR |= 1 << SCB_VTOR_TBLBASE_Pos ;
  __libc_init_array() ;
  main() ;
  while (1) { } // never gets here...
}

