.globl _bootloader_start

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
  .word(0)                     @ reserved for debug (?)
  .word(_bootloader_unhandled+1) @ pendsv
  .word(_bootloader_unhandled+1) @ systick
  .word(_bootloader_unhandled+1) @ systick
  # SAM3S interrupts begin...
  .word(_bootloader_unhandled+1) @ SUPC
  .word(_bootloader_unhandled+1) @ RSTC
  .word(_bootloader_unhandled+1) @ RTC
  .word(_bootloader_unhandled+1) @ RTT
  .word(_bootloader_unhandled+1) @ WDT
  .word(_bootloader_unhandled+1) @ PMC
  .word(_bootloader_unhandled+1) @ EEFC
  .word(_bootloader_unhandled+1) @ reserved
  .word(_bootloader_unhandled+1) @ UART0
  .word(_bootloader_unhandled+1) @ UART1
  .word(_bootloader_unhandled+1) @ SMC
  .word(_bootloader_unhandled+1) @ PIOA
  .word(_bootloader_unhandled+1) @ PIOB
  .word(_bootloader_unhandled+1) @ PIOC
  .word(_bootloader_unhandled+1) @ USART0
  .word(_bootloader_unhandled+1) @ USART1
  .word(_bootloader_unhandled+1) @ reserved
  .word(_bootloader_unhandled+1) @ reserved
  .word(_bootloader_unhandled+1) @ MCI
  .word(_bootloader_unhandled+1) @ TWI 0
  .word(_bootloader_unhandled+1) @ TWI 1
  .word(_bootloader_unhandled+1) @ SPI
  .word(_bootloader_unhandled+1) @ SSC
  .word(_bootloader_unhandled+1) @ TC0
  .word(_bootloader_unhandled+1) @ TC1
  .word(_bootloader_unhandled+1) @ TC2
  .word(_bootloader_unhandled+1) @ TC3
  .word(_bootloader_unhandled+1) @ TC4
  .word(_bootloader_unhandled+1) @ TC5
  .word(_bootloader_unhandled+1) @ ADC
  .word(_bootloader_unhandled+1) @ DAC
  .word(_bootloader_unhandled+1) @ PWM
  .word(_bootloader_unhandled+1) @ CRCCU
  .word(_bootloader_unhandled+1) @ ACC
  .word(_bootloader_unhandled+1) @ USB
  .word(_bootloader_unhandled+1) @ unused
_bootloader_start:
  bl  main 
  ldr r0,=0x00402000 @ look up the stack pointer of the application code
  ldr r1,[r0]
  mov sp,r1
  ldr r0,=0x00402004 @ look up the reset vector of the application code
  ldr r1,[r0]        @ get ready for the jump
  bx r1              @ go for it

_bootloader_unhandled:
  b _bootloader_unhandled

