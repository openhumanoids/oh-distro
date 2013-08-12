@ Software License Agreement (Apache License)
@
@ Copyright 2013 Open Source Robotics Foundation
@ Author: Morgan Quigley
@
@ Licensed under the Apache License, Version 2.0 (the "License");
@ you may not use this file except in compliance with the License.
@ You may obtain a copy of the License at
@
@     http://www.apache.org/licenses/LICENSE-2.0
@
@ Unless required by applicable law or agreed to in writing, software
@ distributed under the License is distributed on an "AS IS" BASIS,
@ WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
@ See the License for the specific language governing permissions and
@ limitations under the License.

.global _bootloader_start

_bootloader_vectors:
  .word(0x2000bffc)
  .word(_bootloader_start+1)
  .word(_bootloader_unhandled+1) @ nmi
  .word(_bootloader_unhandled+1) @ hardfault
  .word(_bootloader_unhandled+1) @ memmanage 
  .word(_bootloader_unhandled+1) @ busfault
  .word(_bootloader_unhandled+1) @ usagefault
  .word(0) @ reserved
  .word(0) @ reserved
  .word(0) @ reserved
  .word(0) @ reserved
  .word(_bootloader_unhandled+1) @ svc
  .word(0)                       @ debugmon
  .word(_bootloader_unhandled+1) @ reserved
  .word(_bootloader_unhandled+1) @ pendsv
  .word(_bootloader_unhandled+1) @ systick
  # SAM3S interrupts begin...
  .word(_bootloader_unhandled+1) @ SUPC
  .word(_bootloader_unhandled+1) @ RSTC
  .word(_bootloader_unhandled+1) @ RTC
  .word(_bootloader_unhandled+1) @ RTT
  .word(_bootloader_unhandled+1) @ WDT
  .word(_bootloader_unhandled+1) @ PMC
  .word(_bootloader_unhandled+1) @ EEFC0
  .word(_bootloader_unhandled+1) @ EEFC1
  .word(_bootloader_unhandled+1) @ UART0
  .word(_bootloader_unhandled+1) @ SMC
  .word(_bootloader_unhandled+1) @ reserved
  .word(_bootloader_unhandled+1) @ PIOA
  .word(_bootloader_unhandled+1) @ PIOB
  .word(_bootloader_unhandled+1) @ PIOC
  .word(_bootloader_unhandled+1) @ PIOD
  .word(_bootloader_unhandled+1) @ reserved
  .word(_bootloader_unhandled+1) @ reserved
  .word(_bootloader_unhandled+1) @ USART0
  .word(_bootloader_unhandled+1) @ USART1
  .word(_bootloader_unhandled+1) @ USART2
  .word(_bootloader_unhandled+1) @ USART3
  .word(_bootloader_unhandled+1) @ HSMCI
  .word(_bootloader_unhandled+1) @ TWI 0
  .word(_bootloader_unhandled+1) @ TWI 1
  .word(_bootloader_unhandled+1) @ SPI
  .word(_bootloader_unhandled+1) @ reserved
  .word(_bootloader_unhandled+1) @ SSC
  .word(_bootloader_unhandled+1) @ TC0
  .word(_bootloader_unhandled+1) @ TC1
  .word(_bootloader_unhandled+1) @ TC2
  .word(_bootloader_unhandled+1) @ TC3
  .word(_bootloader_unhandled+1) @ TC4
  .word(_bootloader_unhandled+1) @ TC5
  .word(_bootloader_unhandled+1) @ TC6
  .word(_bootloader_unhandled+1) @ TC7
  .word(_bootloader_unhandled+1) @ TC8
  .word(_bootloader_unhandled+1) @ PWM
  .word(_bootloader_unhandled+1) @ ADC
  .word(_bootloader_unhandled+1) @ DACC
  .word(_bootloader_unhandled+1) @ DMAC
  .word(_bootloader_unhandled+1) @ UOTGHS
  .word(_bootloader_unhandled+1) @ TRNG
  .word(_bootloader_unhandled+1) @ EMAC
  .word(_bootloader_unhandled+1) @ CAN0
  .word(_bootloader_unhandled+1) @ CAN1
.thumb
_bootloader_start:
  bl bl_main
  ldr r0,=0x00088000 @ look up the stack pointer of the application code
  ldr r1,[r0]
  mov sp,r1
  ldr r0,=0x00088004 @ look up the reset vector of the application code
  ldr r1,[r0]        @ get ready for the jump
  bx r1              @ go for it

_bootloader_unhandled:
  b _bootloader_unhandled  @ spin here so we can debug what happened

